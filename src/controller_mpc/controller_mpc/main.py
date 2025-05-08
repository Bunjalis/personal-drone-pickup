import rclpy
import signal
import sys
import numpy as np
import copy
import math
import csv
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from .acados import generate_ocp_controller
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.current_pose = None

        self.steps = 90 * 30
        self.dt = 1.0 / 30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Get both the OCP solver and the integrator
        self.ocp, self.sim_integrator = generate_ocp_controller()

        time_space = np.linspace(0, self.steps * self.dt, self.steps)
        # Original trajectories
        # self.x_traj = np.zeros_like(time_space)
        # self.y_traj = np.zeros_like(time_space)
        # self.z_traj = 1.5 * np.ones_like(time_space)

        # New oscillating trajectories
        self.x_traj = 0.5 * np.sin(2 * np.pi * 0.1 * time_space)  # Sine wave with frequency 0.1 Hz
        self.y_traj = 0.5 * np.sin(2 * np.pi * 0.2 * time_space)  # Sine wave with frequency 0.2 Hz
        self.z_traj = 1.5 + 0.5 * np.sin(2 * np.pi * 0.05 * time_space)  # Sine wave with frequency 0.05 Hz

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



        # Initialize CSV file at the start of the program
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('state_errors.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header row
            self.csv_writer.writerow([
                    'Step', 'Last_Control_0', 'Last_Control_1', 'Last_Control_2', 'Last_Control_3',
                        'Last_Position_X', 'Last_Position_Y', 'Last_Position_Z',
                        'Predicted_Position_X', 'Predicted_Position_Y', 'Predicted_Position_Z',
                        'Actual_Position_X', 'Actual_Position_Y', 'Actual_Position_Z',
                        'Last_Linear_Velocity_X', 'Last_Linear_Velocity_Y', 'Last_Linear_Velocity_Z',
                        'Predicted_Linear_Velocity_X', 'Predicted_Linear_Velocity_Y', 'Predicted_Linear_Velocity_Z',
                        'Actual_Linear_Velocity_X', 'Actual_Linear_Velocity_Y', 'Actual_Linear_Velocity_Z',
                        'Last_Orientation_W', 'Last_Orientation_X', 'Last_Orientation_Y', 'Last_Orientation_Z',
                        'Predicted_Orientation_W', 'Predicted_Orientation_X', 'Predicted_Orientation_Y', 'Predicted_Orientation_Z',
                        'Actual_Orientation_W', 'Actual_Orientation_X', 'Actual_Orientation_Y', 'Actual_Orientation_Z',
                        'Last_Angular_Velocity_X', 'Last_Angular_Velocity_Y', 'Last_Angular_Velocity_Z',
                        'Predicted_Angular_Velocity_X', 'Predicted_Angular_Velocity_Y', 'Predicted_Angular_Velocity_Z',
                        'Actual_Angular_Velocity_X', 'Actual_Angular_Velocity_Y', 'Actual_Angular_Velocity_Z',
                        'Setpoint_X', 'Setpoint_Y', 'Setpoint_Z',
                        'Setpoint_Orientation_W', 'Setpoint_Orientation_X', 'Setpoint_Orientation_Y', 'Setpoint_Orientation_Z',
                        'Setpoint_VX', 'Setpoint_VY', 'Setpoint_VZ',
                        'Setpoint_AX', 'Setpoint_AY', 'Setpoint_AZ'
        ])


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

        if self.armed and self.current_pose is not None:


            N = 60

            skip_steps = 1
            for j in range(N):
                if self.step_counter + j*skip_steps < self.steps:
                    yref = np.array([self.x_traj[self.step_counter + j*skip_steps], self.y_traj[self.step_counter + j*skip_steps],
                                     self.z_traj[self.step_counter + j*skip_steps], self.qw_traj[self.step_counter + j*skip_steps],
                                     self.qx_traj[self.step_counter + j*skip_steps], self.qy_traj[self.step_counter + j*skip_steps],
                                     self.qz_traj[self.step_counter + j*skip_steps], 
                                     self.vx_traj[self.step_counter + j*skip_steps], self.vy_traj[self.step_counter + j*skip_steps], self.vz_traj[self.step_counter + j*skip_steps], 
                                     self.ax_traj[self.step_counter + j*skip_steps], self.ay_traj[self.step_counter + j*skip_steps], self.az_traj[self.step_counter + j*skip_steps], 
                                     0.0, 0.0, 0.0, 0.0,
                                     0.3, 0.3, 0.3, 0.3])
                else:
                    yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0,0, 0, 0,0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.3])
                self.ocp.set(j, "yref", yref)

            yref_N = np.array([self.x_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.y_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.z_traj[min(self.step_counter + N*skip_steps, self.steps - 1)],
                                    self.qw_traj[min(self.step_counter + N*skip_steps, self.steps - 1)],
                                    self.qx_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.qy_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.qz_traj[min(self.step_counter + N*skip_steps, self.steps - 1)],
                                    self.vx_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.vy_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.vz_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.ax_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.ay_traj[min(self.step_counter + N*skip_steps, self.steps - 1)], 
                                    self.az_traj[min(self.step_counter + N*skip_steps, self.steps - 1)],
                                    0.0, 0.0, 0.0, 0.0 ])

            self.ocp.set(N, "yref", yref_N)

            # Modify the state vector to include estimated motor speeds (omega_est)
            if not hasattr(self, 'omega_est'):
                self.omega_est = np.zeros(4)  # Initialize omega_est if not already present

            # Update the state vector to include omega_est
            current_state_with_omega = np.concatenate((self.current_pose, self.omega_est))

            # Pass the updated state vector to the OCP solver
            self.ocp.set(0, "lbx", current_state_with_omega)
            self.ocp.set(0, "ubx", current_state_with_omega)

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

            # Update omega_est using the motor dynamics
            self.sim_integrator.set("x", current_state_with_omega)
            self.sim_integrator.set("u", u)
            status = self.sim_integrator.solve()
            if status != 0:
                raise Exception(f'acados integrator returned status {status}.')

            # Extract the updated omega_est from the integrator's state
            updated_state = self.sim_integrator.get("x")
            self.omega_est = updated_state[-4:]  # Extract the last 4 elements as omega_est


            print(f"U = {np.round(u, 3)} omega_est = {np.round(self.omega_est, 3)}")

            # Store the current state and control for the next step
            self.last_state = current_state_with_omega.copy()
            self.last_control = u.copy()

            # Store current state and control for next step prediction
            if self.step_counter > 0 and self.step_counter < self.steps and hasattr(self, 'last_state') and hasattr(self, 'last_control'):
                # Integrate the previous state with the last control inputs to predict current state

                self.sim_integrator.set("x", self.last_state)
                self.sim_integrator.set("u", self.last_control)
                
                status = self.sim_integrator.solve()
                predicted_state = self.sim_integrator.get("x")

                self.csv_writer.writerow([
                    self.step_counter,
                    *np.round(self.last_control, 3),
                    *np.round(self.last_state[:3], 3),
                    *np.round(predicted_state[:3], 3),
                    *np.round(self.current_pose[:3], 3),
                    *np.round(self.last_state[7:10], 3),
                    *np.round(predicted_state[7:10], 3),
                    *np.round(self.current_pose[7:10], 3),
                    *np.round(self.last_state[3:7], 3),
                    *np.round(predicted_state[3:7], 3),
                    *np.round(self.current_pose[3:7], 3),
                    *np.round(self.last_state[10:13], 3),
                    *np.round(predicted_state[10:13], 3),
                    *np.round(self.current_pose[10:13], 3),
                    self.x_traj[self.step_counter], self.y_traj[self.step_counter], self.z_traj[self.step_counter],
                    self.qw_traj[self.step_counter], self.qx_traj[self.step_counter], self.qy_traj[self.step_counter], self.qz_traj[self.step_counter],
                    self.vx_traj[self.step_counter], self.vy_traj[self.step_counter], self.vz_traj[self.step_counter],
                    self.ax_traj[self.step_counter], self.ay_traj[self.step_counter], self.az_traj[self.step_counter],
                ])

            self.step_counter += 1


        else:
            self.step_counter = 0
            # Reset stored states when disarmed
            if hasattr(self, 'last_state'):
                delattr(self, 'last_state')
            if hasattr(self, 'last_control'):
                delattr(self, 'last_control')
            if hasattr(self, 'prediction_errors'):
                delattr(self, 'prediction_errors')

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
