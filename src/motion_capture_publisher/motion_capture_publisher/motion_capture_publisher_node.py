import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from interfaces.msg import MotionCaptureState  # Import the new message type
import socket
from dataclasses import dataclass
from typing import Tuple, Optional
import sys
import time

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
        
        # UDP Setup
        self.HOST = "192.168.1.105"
        self.PORT = 1511
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.HOST, self.PORT))
        
        self.get_logger().info(f'Listening for UDP on {self.HOST}:{self.PORT}')

    def clean_message(self, message: str) -> str:
        message = message.replace('-(', '|').replace(')-', '|')
        message = message.replace('(', '').replace(')', '')
        message = message.strip()
        message = message.replace('||', '|')
        return message

    def parse_packet(self, data: str) -> Optional[ObjectData]:
        try:
            if not data or '|' not in data:
                return None

            parts = [p.strip() for p in data.split('|') if p.strip()]
            if len(parts) != 5:
                return None

            obj_id, pos_str, rot_str, vel_str, ang_vel_str = parts
            
            pos_parts = [p.strip() for p in pos_str.split(',')]
            if len(pos_parts) != 3:
                return None
            x, y, z = map(float, pos_parts)
            
            rot_parts = [p.strip() for p in rot_str.split(',')]
            if len(rot_parts) != 4:
                return None
            qx, qy, qz, qw = map(float, rot_parts)

            vel_parts = [p.strip() for p in vel_str.split(',')]
            if len(vel_parts) != 3:
                return None
            vx, vy, vz = map(float, vel_parts)


            ang_vel_parts = [p.strip() for p in ang_vel_str.split(',')]

            
            if len(ang_vel_parts) != 3:
                return None
            

            wx, wy, wz = map(float, ang_vel_parts)


            #wx_out = f"{round(wx, 1):6.1f}"
            #wy_out = f"{round(wy, 1):6.1f}"
            #wz_out = f"{round(wz, 1):6.1f}"
            #print(f"wx: {wx_out}, wy: {wy_out}, wz: {wz_out}")

            return ObjectData(
                id=obj_id,
                position=(x, y, z),
                rotation=(qw, qx, qy, qz),
                velocity=(vx, vy, vz),
                angular_velocity=(wx, wy, wz)
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