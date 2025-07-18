import socket
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry  # Import the Telemetry message
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time

class BetaflightInterfaceNode(Node):
    def __init__(self):
        super().__init__('betaflight_interface')



        self.subscription_motion_capture = self.create_subscription(PoseArray, '/model/x3/pose', self.pose_callback, 10)
        self.subscription_control = self.create_subscription(ELRSCommand, 'ELRSCommand', self.controller_commands_callback, 10)
        self.publisher = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)


        self.set_point = None
        self.current_pose = None

        self.kp = 0.5
        self.ki = 0.001
        self.kd = 0.01
        
        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

        self.databuffer = []
        
    def normalize_quaternion_positive_w(self, x, y, z, w):
        """Normalize quaternion and ensure w is positive."""
        # If w is negative, negate the quaternion
        if w < 0:
            return -x, -y, -z, -w
        return x, y, z, w

    def pose_callback(self, msg):
        # Extract current pose and time
        current_position = msg.poses[5].position
        current_orientation = msg.poses[5].orientation
       
        # Ensure w is positive
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        
        current_time = self.get_clock().now().to_msg()

        if self.last_pose is None:

            print("Initializing last pose, orientation, and time.")
            # Initialize the last pose, orientation, and time
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate time difference (dt)
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)
        
        if dt <= 0:
            print("Time difference is non-positive, skipping callback.")
            # Skip this callback if the time difference is non-positive
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate linear velocity in the world frame
        dx = current_position.x - self.last_pose.x
        dy = current_position.y - self.last_pose.y
        dz = current_position.z - self.last_pose.z
        linear_velocity_world = np.array([dx / dt, dy / dt, dz / dt])

        # Calculate angular velocity
        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity
        q_current = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        rotation_matrix = quaternion_matrix(q1)[:3, :3]  # Extract 3x3 rotation part

        # Transform  velocity from world frame to body frame
        angular_velocity_body = np.dot(rotation_matrix.T, angular_velocity)

      
        # Update last pose, orientation, and time
        self.last_pose = current_position
        self.last_orientation = current_orientation
        self.last_time = current_time


        #print(f" {dt:.6f}")

        # Convert angular velocity from rad/s to deg/s
        angular_velocity_body_deg = np.degrees(angular_velocity_body)
        #print(f"Angular Velocity (Body Frame, deg/s): {angular_velocity_body_deg}")

        # Calculate motor speeds based on angular velocity
        if self.set_point is not None:
            motor_speeds = self.calculate_motor_speeds(angular_velocity_body_deg)

            actuator_msg = Actuators()
            actuator_msg.header.stamp = self.get_clock().now().to_msg()
            actuator_msg.velocity = motor_speeds.tolist()  # Convert to list for Actuators message

            
            self.publisher.publish(actuator_msg)

            




    def calculate_motor_speeds(self, angular_velocity_body_deg):
        # Calculate error (difference between setpoint and current angular velocity)
        error = [self.set_point[0],self.set_point[1],self.set_point[3]] - angular_velocity_body_deg
        # PID control

        print(f"Current roll: {angular_velocity_body_deg[0]}, desired roll: {self.set_point[0]}")

        proportional = self.kp * error

        # Integral term
        if not hasattr(self, 'integral_error'):
            self.integral_error = np.zeros_like(error)  # Initialize integral error if not present
        self.integral_error += error  # Accumulate error
        integral = self.ki * self.integral_error

        # Derivative term
        if not hasattr(self, 'previous_error'):
            self.previous_error = np.zeros_like(error)  # Initialize previous error if not present
        derivative = self.kd * (error - self.previous_error)
        self.previous_error = error  # Update previous error

        offset = proportional + integral + derivative

        throttle = self.set_point[2]  

        # Quadcopter mixer (assuming X-configuration)
        motor_speeds = np.zeros(4)
        motor_speeds[0] = throttle - offset[0] + offset[1] + offset[2]
        motor_speeds[1] = throttle - offset[0] - offset[1] - offset[2]
        motor_speeds[2] = throttle + offset[0] + offset[1] - offset[2]
        motor_speeds[3] = throttle + offset[0] - offset[1] + offset[2]

        # Ensure motor speeds are within a valid range (e.g., 0 to 2000)
        print(f"Motor speeds before clipping: {motor_speeds}")
        motor_speeds = np.clip(motor_speeds, 0, 4631)


        return motor_speeds


    def controller_commands_callback(self, msg):
        self.set_point = [msg.channel_0 * 100, msg.channel_1 *100, (msg.channel_2 + 1)/2 * 4631, -msg.channel_3 * 100]





def main(args=None):
    rclpy.init(args=args)
    node = BetaflightInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
