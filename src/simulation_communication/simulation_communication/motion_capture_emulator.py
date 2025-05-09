#!/usr/bin/env python3

import numpy as np

# Patch np.float to avoid issues with deprecated alias
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time
import signal
import pandas as pd
import time
import math
import matplotlib.pyplot as plt
from interfaces.msg import MotionCaptureState 

class PentaVerify(Node):
    def __init__(self):
        super().__init__('penta_verify')
        self.worldPoseSub_ = self.create_subscription(PoseArray, '/world/quadcopter/dynamic_pose/info', self.worldPoseCallback, 10)
        self.publisher = self.create_publisher(MotionCaptureState, '/motion_capture_state', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/rviz_pose', 10)

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

    def worldPoseCallback(self, msg):
        print("START")
        # Extract current pose and time
        current_position = msg.poses[0].position
        current_orientation = msg.poses[0].orientation
       
        # Ensure w is positive
        current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w = self.normalize_quaternion_positive_w(
            current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w
        )
        
        current_time = msg.header.stamp

        if self.last_pose is None:
            # Initialize the last pose, orientation, and time
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate time difference (dt)
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)

        print(f"Time difference (dt): {dt:.6f} seconds")

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

        # Publish MotionCaptureState
        mcs = MotionCaptureState()
        mcs.pose = Pose()
        mcs.pose.position.x = round(float(current_position.x), 3)
        mcs.pose.position.y = round(float(current_position.y), 3)
        mcs.pose.position.z = round(float(current_position.z), 3)
        mcs.pose.orientation.x = round(float(current_orientation.x), 3)
        mcs.pose.orientation.y = round(float(current_orientation.y), 3)
        mcs.pose.orientation.z = round(float(current_orientation.z), 3)
        mcs.pose.orientation.w = round(float(current_orientation.w), 3)

        mcs.twist = Twist()
        mcs.twist.linear.x = round(float(linear_velocity_world[0]), 3)  # Use body frame velocity
        mcs.twist.linear.y = round(float(linear_velocity_world[1]), 3)  # Use body frame velocity
        mcs.twist.linear.z = round(float(linear_velocity_world[2]), 3)  # Use body frame velocity
        mcs.twist.angular.x = round(float(angular_velocity_body[0]), 3)
        mcs.twist.angular.y = round(float(angular_velocity_body[1]), 3)
        mcs.twist.angular.z = round(float(angular_velocity_body[2]), 3)

        # Print mcs in a nicely formatted and normalized spacing
        print(f"Pose: Position(x={mcs.pose.position.x:7.3f}, y={mcs.pose.position.y:7.3f}, z={mcs.pose.position.z:7.3f}), ")
        print(f"Orientation(x={mcs.pose.orientation.x:7.3f}, y={mcs.pose.orientation.y:7.3f}, z={mcs.pose.orientation.z:7.3f}, w={mcs.pose.orientation.w:7.3f})")
        print(f"Twist: Linear(x={mcs.twist.linear.x:7.3f}, y={mcs.twist.linear.y:7.3f}, z={mcs.twist.linear.z:7.3f}), ")
        print(f"Angular(x={mcs.twist.angular.x:7.3f}, y={mcs.twist.angular.y:7.3f}, z={mcs.twist.angular.z:7.3f})")

        # Create a PoseStamped message for RViz visualization
        pose_for_rviz = PoseStamped()
        pose_for_rviz.header.stamp = self.get_clock().now().to_msg()
        pose_for_rviz.header.frame_id = "map"
        pose_for_rviz.pose.position.x = float(current_position.x)
        pose_for_rviz.pose.position.y = float(current_position.y)
        pose_for_rviz.pose.position.z = float(current_position.z)
        pose_for_rviz.pose.orientation.x = float(current_orientation.x)
        pose_for_rviz.pose.orientation.y = float(current_orientation.y)
        pose_for_rviz.pose.orientation.z = float(current_orientation.z)
        pose_for_rviz.pose.orientation.w = float(current_orientation.w)

        self.pose_publisher.publish(pose_for_rviz)

        simulate_delay = False  # Set to True to simulate delay
        if simulate_delay == True:

            self.databuffer.append(mcs)
            if len(self.databuffer) > 5 :
                mcs = self.databuffer.pop(0)
                self.publisher.publish(mcs)
        else:
            self.publisher.publish(mcs)

        

        print("END")

        # Update last pose, orientation, and time
        self.last_pose = current_position
        self.last_orientation = current_orientation
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pentaVerify = PentaVerify()
    rclpy.spin(pentaVerify)
    pentaVerify.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
