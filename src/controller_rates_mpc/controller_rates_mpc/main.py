import rclpy
import signal
import sys
import numpy as np
import copy
import math
import csv
import os
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from .acados import generate_ocp_controller
from .gui import GUI
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory
from .l1_augmentation import L1Controller
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.enable_L1_augmentation = False  # Enable L1 augmentation by default

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.trajectory_publisher_ = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.current_pose = None

        self.ocp, self.sim_integrator = generate_ocp_controller()

        self.dt = 1.0 / 30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        

        #self.traj = circle_trajectory(self.dt)
        self.traj = hover_trajectory(self.dt)   


        self.steps = self.traj.shape[1] - 1  # Number of steps in the trajectory


        self.gui = GUI(self)
        self.armed = False
        
        self.pre_start_duration = 2.0
        self.pre_start_counter = 0 
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 20
        self.skip_steps = 3


        # Initialize CSV file at the start of the program
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header row
            self.csv_writer.writerow([
                'Step', 'u0', 'u1', 'u2', 'u3',
                'px', 'py', 'pz', 'rw', 'rx', 'ry', 'rz',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz',
                'sp_px', 'sp_py', 'sp_pz', 'sp_rw', 'sp_rx', 'sp_ry', 'sp_rz',
                'sp_vx', 'sp_vy', 'sp_vz', 'sp_wx', 'sp_wy', 'sp_wz',
        ])

        self.predicted_state = None  # Initialize in the constructor

        self.l1_controller = L1Controller(filter_size=2, adaptation_gain=0.01, dt=self.dt)

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.current_pose = np.round(np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        ]), 3)

    def control_loop(self):

        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:

            if self.step_counter + self.N * self.skip_steps > self.steps:
                self.step_counter = 0
                self.armed = False
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cmd_publisher_.publish(msg)
                self.on_close()

            for j in range(self.N):
                sc = self.step_counter + j * self.skip_steps
                yref = np.concatenate((self.traj[:, sc], [0.0, 0.0, 0.2, 0.0]))
                self.ocp.set(j, "yref", yref)

            sn = self.step_counter + self.N * self.skip_steps
            yref_N = self.traj[:, sn]
            self.ocp.set(self.N, "yref", yref_N)

            self.ocp.set(0, "lbx", self.current_pose)
            self.ocp.set(0, "ubx", self.current_pose)

            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')

            u = self.ocp.get(0, "u")

            # Use L1 adaptive augmentation to adjust the control output
            if self.predicted_state is not None and self.enable_L1_augmentation:
                error = self.current_pose - self.predicted_state
                velocity_error = error[7:10]  # Extract velocity components (vx, vy, vz)
                rounded_velocity_error = np.round(velocity_error, 3)
                print(f"Error between observed and previous predicted state (velocity): {rounded_velocity_error}")


                augmented_u = self.l1_controller.update(error)  # Pass only the error
                print(f"Augmented control output: {augmented_u}")
                u[2] -= augmented_u  # Adjust the throttle (channel 2)

                
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)


            # Predict the next state using the integrator
            self.sim_integrator.set("x", self.current_pose)
            self.sim_integrator.set("u", u)
            self.sim_integrator.solve()
            self.predicted_state = self.sim_integrator.get("x")



            if self.step_counter > 0 and self.step_counter < self.steps:

                self.csv_writer.writerow(
                    [self.step_counter,
                     msg.channel_0,
                     msg.channel_1,
                     msg.channel_2,
                     msg.channel_3] +
                    list(self.current_pose) +
                    list(self.traj[:, self.step_counter])
                )

            self.step_counter += 1

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0

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
