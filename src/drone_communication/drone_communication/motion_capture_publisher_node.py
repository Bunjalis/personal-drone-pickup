import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from interfaces.msg import MotionCaptureState  # Import the new message type
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
import socket
from dataclasses import dataclass
from typing import Tuple, Optional
import sys
import time
import numpy as np

@dataclass
class ObjectData:
    id: str
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]

class MotionCapturePublisher(Node):
    def __init__(self):
        super().__init__('udp_to_pose_node')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(MotionCaptureState, 'motion_capture_state', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/rviz_pose', 10)
        
        # UDP Setup
        self.HOST = "192.168.1.105"
        self.PORT = 1511
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.HOST, self.PORT))
        
        self.get_logger().info(f'Listening for UDP on {self.HOST}:{self.PORT}')

        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

    def clean_message(self, message: str) -> str:
        message = message.replace('-(', '|').replace(')-', '|')
        message = message.replace('(', '').replace(')', '')
        message = message.strip()
        message = message.replace('||', '|')
        return message
    
    def normalize_quaternion_positive_w(self, x, y, z, w):
        """Normalize quaternion and ensure w is positive."""
        print(f"Normalizing quaternion: {x}, {y}, {z}, {w}")
        if w < 0:
            print(f"Quaternion w is negative, negating all components.")
            return -x, -y, -z, -w
        print(f"Quaternion w is positive, no change needed.")
        return x, y, z, w

    def parse_packet(self, data: str) -> Optional[ObjectData]:
        try:
            print(f"trying to parse: {data}")
            if not data or '|' not in data:
                print("Invalid data format, skipping packet.")
                return None

            parts = [p.strip() for p in data.split('|') if p.strip()]
            if len(parts) != 3:
                print("Invalid parts length.")
                return None

            obj_id, pos_str, rot_str = parts
            
            pos_parts = [p.strip() for p in pos_str.split(',')]
            if len(pos_parts) != 3:
                print("Invalid position format.")
                return None
            x, y, z =  map(float,pos_parts)
            
            rot_parts = [p.strip() for p in rot_str.split(',')]
            if len(rot_parts) != 4:
                print("Invalid rotation format.")
                return None
                
            qx, qy, qz, qw  =  map(float,rot_parts)

            current_time = time.time()
            if self.last_pose is None:
                # Initialize the last pose, orientation, and time
                self.last_pose = np.array([x, y, z])
                self.last_orientation = np.array([qx, qy, qz, qw])
                self.last_time = current_time
                return None

            # Calculate time difference (dt)
            dt = current_time - self.last_time
            self.last_time = current_time

            if dt <= 0:
                self.last_pose = np.array([x, y, z])
                self.last_orientation = np.array([qx, qy, qz, qw])
                self.last_time = current_time
                return None
            



            # Calculate linear velocity in the world frame
            dx, dy, dz = x - self.last_pose[0], y - self.last_pose[1], z - self.last_pose[2]
            linear_velocity_world = np.array([dx / dt, dy / dt, dz / dt])

            # Calculate angular velocity
            q1 = self.last_orientation
            q2 = np.array([qx, qy, qz, qw])
            q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
            angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity
            rotation_matrix = quaternion_matrix(q2)[:3, :3]  # Extract 3x3 rotation part

            # Transform velocity from world frame to body frame
            angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

            # Update last pose, orientation, and time
            self.last_pose = np.array([x, y, z])
            self.last_orientation = np.array([qx, qy, qz, qw])
            

            pose_for_rviz = PoseStamped()
            pose_for_rviz.header.stamp = self.get_clock().now().to_msg()
            pose_for_rviz.header.frame_id = "map"
            pose_for_rviz.pose.position.x = x
            pose_for_rviz.pose.position.y = y
            pose_for_rviz.pose.position.z = z
            pose_for_rviz.pose.orientation.x = qx
            pose_for_rviz.pose.orientation.y = qy
            pose_for_rviz.pose.orientation.z = qz
            pose_for_rviz.pose.orientation.w = qw

            self.pose_publisher.publish(pose_for_rviz)
            
            return ObjectData(
                id=obj_id,
                position=(x, y, z),
                rotation=(qw, qx, qy, qz),
                velocity=(linear_velocity_world[0], linear_velocity_world[1], linear_velocity_world[2]),
                angular_velocity=(angular_velocity_body[0], angular_velocity_body[1], angular_velocity_body[2])
            )
        except Exception:
            return None

    def create_motion_capture_state_msg(self, obj_data: ObjectData) -> MotionCaptureState:
        msg = MotionCaptureState()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = obj_data.id
        
        # Set pose
        msg.pose = Pose()
        msg.pose.position.x = obj_data.position[0]
        msg.pose.position.y = obj_data.position[1]
        msg.pose.position.z = obj_data.position[2]
        msg.pose.orientation.w = obj_data.rotation[0]
        msg.pose.orientation.x = obj_data.rotation[1]
        msg.pose.orientation.y = obj_data.rotation[2]
        msg.pose.orientation.z = obj_data.rotation[3]
        
        # Set twist (velocity)
        msg.twist = Twist()
        msg.twist.linear.x = obj_data.velocity[0]
        msg.twist.linear.y = obj_data.velocity[1]
        msg.twist.linear.z = obj_data.velocity[2]
        msg.twist.angular.x = obj_data.angular_velocity[0]
        msg.twist.angular.y = obj_data.angular_velocity[1]
        msg.twist.angular.z = obj_data.angular_velocity[2]
        
        return msg

    def run(self):
        try:
            while rclpy.ok():
                data, _ = self.sock.recvfrom(255)
                message = data.decode().strip()
                
                cleaned_message = self.clean_message(message)
                obj_data = self.parse_packet(cleaned_message)

                print(f"Do we hqve obj_data? {obj_data}")
                
                if obj_data:
                    motion_capture_msg = self.create_motion_capture_state_msg(obj_data)
                    self.publisher.publish(motion_capture_msg)

        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
        finally:
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotionCapturePublisher()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()