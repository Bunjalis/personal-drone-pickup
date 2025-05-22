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
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.trajectory_publisher_ = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.current_pose = None

        self.steps = 90 * 30
        self.dt = 1.0 / 30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Get both the OCP solver and the integrator
        self.ocp, self.sim_integrator = generate_ocp_controller()

        time_space = np.linspace(0, self.steps * self.dt, self.steps)
        # Original trajectories
        self.x_traj = np.zeros_like(time_space)
        self.y_traj = np.zeros_like(time_space)
        #self.z_traj = 1.0 * np.ones_like(time_space)

        # New oscillating trajectories
        self.x_traj = 0.75 * np.sin(2 * np.pi * 0.1 * time_space)  # Sine wave with frequency 0.1 Hz
        self.y_traj = 0.75 * np.sin(2 * np.pi * 0.3 * time_space)  # Sine wave with frequency 0.2 Hz
        self.z_traj = 1.0 + 0.5 * np.sin(2 * np.pi * 0.1 * time_space)  # Sine wave with frequency 0.05 Hz

        roll_traj = np.zeros_like(time_space)  # Roll remains 0
        pitch_traj = np.zeros_like(time_space)  # Pitch remains 0
        yaw_traj = np.zeros_like(time_space)  # Pitch remains 0

        rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
        quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Converts to [q_x, q_y, q_z, q_w]

        self.qx_traj = quaternions[:, 0]
        self.qy_traj = quaternions[:, 1]
        self.qz_traj = quaternions[:, 2]
        self.qw_traj = quaternions[:, 3]

        # Calculate world frame velocities for the trajectory
        self.vx_traj = np.gradient(self.x_traj, self.dt)
        self.vy_traj = np.gradient(self.y_traj, self.dt)
        self.vz_traj = np.gradient(self.z_traj, self.dt)

        # Calculate desired angular velocities for the orientation trajectory
        self.ax_traj = np.zeros_like(time_space)  # Roll rate remains 0
        self.ay_traj = np.zeros_like(time_space)  # Pitch rate remains 0
        self.az_traj = np.zeros_like(time_space)  # Yaw rate trajectory

        self.gui = GUI(self)
        self.armed = False
        self.executing_actions = False
        self.executed_steps = 0
        self.saved_states = []
        self.saved_controls = []
        self.recorded_states = []  # To store the recorded states
        self.initial_solve_state = None
        self.initial_solve_controls = None

        self.pre_start_duration = 2.0  # Duration for the pre-start state in seconds
        self.pre_start_counter = 0  # Counter to track pre-start steps
        self.pre_start_steps = int(self.pre_start_duration / self.dt)  # Steps for pre-start state

        self.N = 20


        # Initialize CSV file at the start of the program
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header row
            self.csv_writer.writerow([
                    'Step', 'u0', 'u1', 'u2', 'u3',
                        'px', 'py', 'pz',
                        'rw', 'rx', 'ry', 'rz',
                        'vx', 'vy', 'vz',
                        'wx', 'wy', 'wz',
                        'sp_px', 'sp_py', 'sp_pz',
                        'sp_rw', 'sp_rx', 'sp_ry', 'sp_rz',
                        'sp_vx', 'sp_vy', 'sp_vz',
                        'sp_wx', 'sp_wy', 'sp_wz',
        ])


        if not hasattr(self, 'motion_capture_csv_initialized'):
            self.motion_capture_csv_initialized = True
            self.motion_capture_csv_file = open('motion_capture_results.csv', mode='w', newline='')
            self.motion_capture_csv_writer = csv.writer(self.motion_capture_csv_file)
            # Write header row
            self.motion_capture_csv_writer.writerow([
                        'px', 'py', 'pz',
                        'rw', 'rx', 'ry', 'rz',
                        'vx', 'vy', 'vz',
                        'wx', 'wy', 'wz',
        ])


    def pose_callback(self, msg: MotionCaptureState):
        position = msg.pose.position
        orientation = msg.pose.orientation
        linear_velocity = msg.twist.linear
        angular_velocity = msg.twist.angular

        noisy_position = np.array([position.x, position.y, position.z]) #+ noise_pos
        noisy_orientation = np.array([orientation.w, orientation.x, orientation.y, orientation.z]) #+ noise_rot
        noisy_linear_velocity = np.array([linear_velocity.x, linear_velocity.y, linear_velocity.z]) #+ noise_lin_vel
        noisy_angular_velocity = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z]) #+ noise_rot_vel

        # Combine all components into the current pose
        self.current_pose = np.round(np.concatenate((noisy_position, noisy_orientation, noisy_linear_velocity, noisy_angular_velocity)), 3)

        if self.armed:
            self.motion_capture_csv_writer.writerow(np.round(np.concatenate(( noisy_position, noisy_orientation, noisy_linear_velocity, noisy_angular_velocity  )), 3))

    def control_loop(self):
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0

        # Pre-start state: Send 0.1 on all channels for one second
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg.armed = True
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = 0.0
            msg.channel_3 = 0.0
            self.cmd_publisher_.publish(msg)

            self.pre_start_counter += 1
        elif self.armed and self.current_pose is not None:

            skip_steps = 3
            for j in range(self.N):
                if self.step_counter + j*skip_steps < self.steps:
                    yref = np.array([self.x_traj[self.step_counter + j*skip_steps], self.y_traj[self.step_counter + j*skip_steps],
                                     self.z_traj[self.step_counter + j*skip_steps], self.qw_traj[self.step_counter + j*skip_steps],
                                     self.qx_traj[self.step_counter + j*skip_steps], self.qy_traj[self.step_counter + j*skip_steps],
                                     self.qz_traj[self.step_counter + j*skip_steps], 
                                     self.vx_traj[self.step_counter + j*skip_steps], self.vy_traj[self.step_counter + j*skip_steps], self.vz_traj[self.step_counter + j*skip_steps], 
                                     self.ax_traj[self.step_counter + j*skip_steps], self.ay_traj[self.step_counter + j*skip_steps], self.az_traj[self.step_counter + j*skip_steps], 
                                      0.2, 0.2, 0.2, 0.2])
                else:
                    yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0,0, 0, 0,0, 0.0, 0.2, 0.2, 0.2, 0.2])
                self.ocp.set(j, "yref", yref)


            yref_N = np.array([self.x_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.y_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.z_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],
                                    self.qw_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.qx_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.qy_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.qz_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],
                                    self.vx_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.vy_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.vz_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], 
                                    self.ax_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.ay_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.az_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)]])
            self.ocp.set(self.N, "yref", yref_N)

            self.ocp.set(0, "lbx", self.current_pose)
            self.ocp.set(0, "ubx", self.current_pose)

            # Solve the OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')

            # Retrieve the control inputs
            u = self.ocp.get(0, "u")
            msg.armed = True
            msg.channel_0 = round(u[0], 3)
            msg.channel_1 = round(u[1], 3)
            msg.channel_2 = round(u[2], 3)
            msg.channel_3 = round(u[3], 3)
            
            self.cmd_publisher_.publish(msg)

            print(f"U = {np.round(u, 3)}")


            # Store current state and control for next step prediction
            if self.step_counter > 0 and self.step_counter < self.steps:

                self.csv_writer.writerow([
                    self.step_counter,
                    msg.channel_0, msg.channel_1, msg.channel_2, msg.channel_3,
                    self.current_pose[0], self.current_pose[1], self.current_pose[2],
                    self.current_pose[3], self.current_pose[4], self.current_pose[5], self.current_pose[6],
                    self.current_pose[7], self.current_pose[8], self.current_pose[9],
                    self.current_pose[10], self.current_pose[11], self.current_pose[12],
                    self.x_traj[self.step_counter], self.y_traj[self.step_counter], self.z_traj[self.step_counter],
                    self.qw_traj[self.step_counter], self.qx_traj[self.step_counter], self.qy_traj[self.step_counter], self.qz_traj[self.step_counter],
                    self.vx_traj[self.step_counter], self.vy_traj[self.step_counter], self.vz_traj[self.step_counter],
                    self.ax_traj[self.step_counter], self.ay_traj[self.step_counter], self.az_traj[self.step_counter],
                ])

            self.step_counter += 1


        else:
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
