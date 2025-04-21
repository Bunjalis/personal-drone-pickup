import rclpy
import signal
import sys
import numpy as np
import copy
import math
from rclpy.node import Node
from datetime import datetime

from .acados import generate_ocp_controller
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.current_pose = None

        self.steps = 90 * 60
        self.dt = 1.0 / 60.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.ocp = generate_ocp_controller()

        time_space = np.linspace(0, self.steps * self.dt, self.steps)
        self.x_traj = 0 + 0.0 * np.sin(1 * np.pi * 0.0 * time_space)
        self.y_traj = 0 + 0.0 * np.sin(1 * np.pi * 0.0 * time_space)
        self.z_traj = 2.0 + 0.5 * np.sin(1 * np.pi * 0.2 * time_space)

        self.gui = GUI(self)
        self.armed = False
        


    def pose_callback(self, msg: MotionCaptureState):
        position = msg.pose.position
        orientation = msg.pose.orientation
        linear_velocity = msg.twist.linear
        angular_velocity = msg.twist.angular


        self.current_pose = np.array([position.x, position.y, position.z,
                                        orientation.w, orientation.x, orientation.y, orientation.z,
                                        linear_velocity.x, linear_velocity.y, linear_velocity.z,
                                        angular_velocity.x, angular_velocity.y, angular_velocity.z])

    def control_loop(self):

        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0


        if self.armed == True:
            
            for j in range(180):
                if self.step_counter + j < self.steps:
                    yref = np.array([self.x_traj[self.step_counter + j], self.y_traj[self.step_counter + j], self.z_traj[self.step_counter + j], 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3, 0.3, 0.3, 0.3])
                else:
                    yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3, 0.3, 0.3, 0.3])
                
                self.ocp.set(j, "yref", yref)

            self.ocp.set(0, "lbx", self.current_pose)
            self.ocp.set(0, "ubx", self.current_pose)


            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')

            u = self.ocp.get(0, "u")

            print(f"u: {u}")

            u_command = u.tolist()


            msg.armed = True
            msg.channel_0 = u_command[0]
            msg.channel_1 = u_command[1]
            msg.channel_2 = u_command[2]
            msg.channel_3 = u_command[3]

            self.step_counter += 1
        
        else:
            step_counter = 0
            u_command = [0, 0, 0, 0]

        self.cmd_publisher_.publish(msg)


    def signal_handler(self, sig, frame):
        self.on_close()

    def on_close(self):
        self.gui.quit()
        rclpy.shutdown()
        sys.exit(0)




def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    while rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=0.1)
        controller.gui.handle_events()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
