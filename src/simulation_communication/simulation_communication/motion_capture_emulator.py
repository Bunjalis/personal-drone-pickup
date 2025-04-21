#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
from builtin_interfaces.msg import Time
import numpy as np
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

        self.last_pose = None
        self.last_orientation = None
        self.last_time = None

    def worldPoseCallback(self, msg):
        print("START")
        # Extract current pose and time
        current_position = msg.poses[0].position
        current_orientation = msg.poses[0].orientation
        current_time = msg.header.stamp

        if self.last_pose is None:
            # Initialize the last pose, orientation, and time
            self.last_pose = current_position
            self.last_orientation = current_orientation
            self.last_time = current_time
            return

        # Calculate time difference (dt)
        dt = (current_time.sec + current_time.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)

        if dt <= 0:
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

        # Transform linear velocity to the body frame
        q2 = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        rotation_matrix = quaternion_matrix(q2)[:3, :3]  # Extract the 3x3 rotation matrix
        linear_velocity_body = np.dot(rotation_matrix.T, linear_velocity_world)  # Transform to body frame

        # Calculate angular velocity
        q1 = [self.last_orientation.x, self.last_orientation.y, self.last_orientation.z, self.last_orientation.w]
        q_relative = quaternion_multiply(q2, quaternion_inverse(q1))  # Relative rotation
        angular_velocity = 2 * np.array([q_relative[0], q_relative[1], q_relative[2]]) / dt  # Angular velocity

        # Publish MotionCaptureState
        mcs = MotionCaptureState()
        mcs.pose = Pose()
        mcs.pose.position.x = float(current_position.x)
        mcs.pose.position.y = float(current_position.y)
        mcs.pose.position.z = float(current_position.z)
        mcs.pose.orientation.x = float(current_orientation.x)
        mcs.pose.orientation.y = float(current_orientation.y)
        mcs.pose.orientation.z = float(current_orientation.z)
        mcs.pose.orientation.w = float(current_orientation.w)

        mcs.twist = Twist()
        mcs.twist.linear.x = float(linear_velocity_body[0])
        mcs.twist.linear.y = float(linear_velocity_body[1])
        mcs.twist.linear.z = float(linear_velocity_body[2])
        mcs.twist.angular.x = float(angular_velocity[0])
        mcs.twist.angular.y = float(angular_velocity[1])
        mcs.twist.angular.z = float(angular_velocity[2])

        print(mcs)

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
