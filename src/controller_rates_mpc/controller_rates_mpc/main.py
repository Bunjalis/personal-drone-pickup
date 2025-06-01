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
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

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
        msg.channel_2 = -1.0
        msg.channel_3 = 0.0

        # Pre-start state: Send 0.1 on all channels for one second
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg.armed = True
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = -1.0
            msg.channel_3 = 0.0
            self.cmd_publisher_.publish(msg)

            self.pre_start_counter += 1
        elif self.armed and self.current_pose is not None:


            # load current pose
            # pass pose into model


            skip_steps = 3
            if self.step_counter + self.N*skip_steps > self.steps:
                self.step_counter = 0
                self.executing_actions = False
                self.armed = False
                msg.armed = False
                msg.channel_0 = 0.0
                msg.channel_1 = 0.0
                msg.channel_2 = -1.0
                msg.channel_3 = 0.0
                self.cmd_publisher_.publish(msg)
                return
            
            for j in range(self.N):
                sc = self.step_counter + j*skip_steps

                yref = np.array([self.traj[0][sc], self.traj[1][sc], self.traj[2][sc],
                                    self.traj[3][sc],self.traj[4][sc],self.traj[5][sc],self.traj[6][sc],
                                    self.traj[7][sc], self.traj[8][sc], self.traj[9][sc],
                                    self.traj[10][sc], self.traj[11][sc], self.traj[12][sc],
                                    0.0, 0.0, 0.2, 0.0])
                self.ocp.set(j, "yref", yref)

            sn = self.step_counter + self.N*skip_steps
            yref_N = np.array([self.traj[0][sn], self.traj[1][sn], self.traj[2][sn],
                                    self.traj[3][sn],self.traj[4][sn],self.traj[5][sn],self.traj[6][sn],
                                    self.traj[7][sn], self.traj[8][sn], self.traj[9][sn],
                                    self.traj[10][sn], self.traj[11][sn], self.traj[12][sn]])
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
            msg.channel_2 = round((u[2]*2)-1, 3)
            msg.channel_3 = round(u[3], 3)
            
            self.cmd_publisher_.publish(msg)


            # Store current state and control for next step prediction
            if self.step_counter > 0 and self.step_counter < self.steps:

                self.csv_writer.writerow([
                    self.step_counter,
                    msg.channel_0, msg.channel_1, msg.channel_2, msg.channel_3,
                    self.current_pose[0], self.current_pose[1], self.current_pose[2],
                    self.current_pose[3], self.current_pose[4], self.current_pose[5], self.current_pose[6],
                    self.current_pose[7], self.current_pose[8], self.current_pose[9],
                    self.current_pose[10], self.current_pose[11], self.current_pose[12],
                    self.traj[0][self.step_counter], self.traj[1][self.step_counter], self.traj[2][self.step_counter],
                    self.traj[3][self.step_counter], self.traj[4][self.step_counter], self.traj[5][self.step_counter], self.traj[6][self.step_counter],
                    self.traj[7][self.step_counter], self.traj[8][self.step_counter], self.traj[9][self.step_counter],
                    self.traj[10][self.step_counter], self.traj[11][self.step_counter], self.traj[12][self.step_counter],
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
