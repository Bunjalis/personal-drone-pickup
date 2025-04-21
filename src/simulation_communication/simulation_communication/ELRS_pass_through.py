


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist, PoseArray, Pose
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Time
import numpy as np
import signal
import pandas as pd
import time
import math
import matplotlib.pyplot as plt
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry  # Import the Telemetry message


class ELRSPassThrough(Node):
    def __init__(self):
        super().__init__('ELRS_pass_through')
        self.publisher = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)
        self.subscription_floats = self.create_subscription(ELRSCommand, 'ELRSCommand', self.controller_commands_callback, 10)



    def controller_commands_callback(self, msg):
        self.speed = [msg.channel_0*1000.0,msg.channel_1*1000.0,msg.channel_2*1000.0,msg.channel_3*1000.0]
        actuator_msg = Actuators()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.velocity = self.speed

        print("Actuator command: ", actuator_msg.velocity)
        self.publisher.publish(actuator_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ELRSPassThrough()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
