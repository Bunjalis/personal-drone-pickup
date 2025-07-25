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
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, light_circle_trajectory, backflip_trajectory
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray
from scipy.linalg import cholesky
import matplotlib.pyplot as plt

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.orb_slam_state_subscription_ = self.create_subscription(MotionCaptureState, '/orb_slam_state', self.orb_slam_state_callback, 10)
        self.trajectory_publisher_ = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.last_measured_pose = None
        self.motion_capture_pose = None

        self.ocp, self.sim_integrator = generate_ocp_controller()

        self.dt = 1.0 /30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)



        self.delay_estimation_timer = self.create_timer(1/10.0, self.delay_estimation_timer)

        self.traj = hover_trajectory(self.dt)  
        self.traj = z_sin_trajectory(self.dt)  
        self.traj = xyz_sine_trajectory(self.dt)  
        self.traj = circle_trajectory(self.dt)   
        #self.traj = light_circle_trajectory(self.dt)
        #self.traj = backflip_trajectory(self.dt)  # Use the backflip trajectory


        trial_name = "test"


        self.est_params = np.array([38.0])  # Initialize thrust ratio parameter to a reasonable value



        self.steps = self.traj.shape[1] - 1  # Number of steps in the trajectory

        self.gui = GUI(self)
        self.armed = False
        
        self.pre_start_duration = 2.0
        self.pre_start_counter = 0 
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 20
        self.skip_steps = 3

            


        # Create a folder to save all CSV files
        base_experiment_folder = "/home/mitchell/Documents/PhD/drone_cage_control/ORBSLAM_PAPER_EXPERIMENTS"
        self.output_folder = os.path.join(base_experiment_folder, trial_name)
        os.makedirs(self.output_folder, exist_ok=True)

        # Initialize CSV writers in the constructor
        self.control_history_file = open(os.path.join(self.output_folder, 'control_history.csv'), mode='w', newline='')
        self.control_history_writer = csv.writer(self.control_history_file)

        self.observed_state_history_file = open(os.path.join(self.output_folder, 'observed_state_history.csv'), mode='w', newline='')
        self.observed_state_history_writer = csv.writer(self.observed_state_history_file)

        self.motion_capture_history_file = open(os.path.join(self.output_folder, 'motion_capture_history.csv'), mode='w', newline='')
        self.motion_capture_history_writer = csv.writer(self.motion_capture_history_file)

        self.parameter_estimation_history_file = open(os.path.join(self.output_folder, 'parameter_estimation_history.csv'), mode='w', newline='')
        self.parameter_estimation_history_writer = csv.writer(self.parameter_estimation_history_file)

        self.estimated_state_history_file = open(os.path.join(self.output_folder, 'estimated_state_history.csv'), mode='w', newline='')
        self.estimated_state_history_writer = csv.writer(self.estimated_state_history_file)

        self.delay_state_estimation_history_file = open(os.path.join(self.output_folder, 'delay_state_estimation_history.csv'), mode='w', newline='')
        self.delay_state_estimation_history_writer = csv.writer(self.delay_state_estimation_history_file)

        self.UKF_state_estimation_history_file = open(os.path.join(self.output_folder, 'UKF_state_estimation_history.csv'), mode='w', newline='')
        self.UKF_state_estimation_history_writer = csv.writer(self.UKF_state_estimation_history_file)

        self.trajectory_file = open(os.path.join(self.output_folder, 'trajectory.csv'), mode='w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerows(self.traj.T)  # Save trajectory as rows

        
        self.alpha, self.beta, self.kappa = 0.1, 2, 0

        self.x_est = np.array([0.0, 0.0, 0.0, 
                                1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 
                                self.est_params[0]])
        self.P = np.diag([0.1, 0.1, 0.1,  # Increase initial uncertainty for position
                          0.1, 0.1, 0.1, 0.1,  # Quaternion
                          0.1, 0.1, 0.1,  # Velocity
                          0.1, 0.1, 0.1,  # Angular rates
                          0.5])  # Thrust ratio parameter uncertainty
        self.Q = np.diag([1e-4, 1e-4, 1e-4,  # Position process noise
                          1e-5, 1e-5, 1e-5, 1e-5,  # Quaternion process noise
                          1e-3, 1e-3, 1e-3,  # Velocity process noise
                          1e-3, 1e-3, 1e-3,  # Angular rates process noise
                          1e-6])  # Thrust ratio process noise
        self.R = np.diag([0.05]*13)  # Measurement noise for all 13 state elements


        self.delay_states = 1
        self.delay_states_float = float(self.delay_states)

        

        self.control_history = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]] * 10  # Initialize with zeros

        self.observed_state_history = []




        self.motion_capture_history = []
        self.parameter_estimation_history = []
        self.estimated_state_history = []

        self.delay_state_estimation_history = []
        self.UKF_state_estimation_history = []

        # Initialize CSV writers in the constructor
        self.control_history_file = open(os.path.join(self.output_folder, 'control_history.csv'), mode='w', newline='')
        self.control_history_writer = csv.writer(self.control_history_file)

        self.observed_state_history_file = open(os.path.join(self.output_folder, 'observed_state_history.csv'), mode='w', newline='')
        self.observed_state_history_writer = csv.writer(self.observed_state_history_file)

        self.motion_capture_history_file = open(os.path.join(self.output_folder, 'motion_capture_history.csv'), mode='w', newline='')
        self.motion_capture_history_writer = csv.writer(self.motion_capture_history_file)

        self.parameter_estimation_history_file = open(os.path.join(self.output_folder, 'parameter_estimation_history.csv'), mode='w', newline='')
        self.parameter_estimation_history_writer = csv.writer(self.parameter_estimation_history_file)

        self.estimated_state_history_file = open(os.path.join(self.output_folder, 'estimated_state_history.csv'), mode='w', newline='')
        self.estimated_state_history_writer = csv.writer(self.estimated_state_history_file)

        self.delay_state_estimation_history_file = open(os.path.join(self.output_folder, 'delay_state_estimation_history.csv'), mode='w', newline='')
        self.delay_state_estimation_history_writer = csv.writer(self.delay_state_estimation_history_file)

        self.UKF_state_estimation_history_file = open(os.path.join(self.output_folder, 'UKF_state_estimation_history.csv'), mode='w', newline='')
        self.UKF_state_estimation_history_writer = csv.writer(self.UKF_state_estimation_history_file)

        self.trajectory_file = open(os.path.join(self.output_folder, 'trajectory.csv'), mode='w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.trajectory_writer.writerows(self.traj.T)  # Save trajectory as rows

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.motion_capture_pose = np.round(np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        ]), 3) #motion_capture_pose
        #self.current_pose = np.round(np.array([
        #    p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        #]), 3)


    def orb_slam_state_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.current_pose = np.round(np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        ]), 3) #orb_slam_pose


    def delay_estimation_timer(self):
        ### This function estimates the delay in the system by simulating different delay values and calculating the error based on position

        if len(self.observed_state_history) < 30 or len(self.control_history) < 30:
            # Ensure we have enough history to perform the estimation
            return

        min_error = float('inf')
        optimal_delay = self.delay_states
        error_latencies = []  # Array to store error latencies for each delay

        # Iterate over possible delay values to find the one that minimizes the position error
        for delay in range(1, 12):  # Test delays from 1 to 10
            total_position_error = 0
            estimated_state = copy.deepcopy(self.observed_state_history[-30])  # Start with the oldest state in the last 30
            delayed_control_history = self.control_history[-(30 + delay):-delay]  # Use delayed controls

            for i, control in enumerate(delayed_control_history):
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(control[0:4]).flatten())))
                self.sim_integrator.set("u", np.array(control[4:8]))
                self.sim_integrator.set("p", self.est_params)
                status_sim = self.sim_integrator.solve()
                if status_sim != 0:
                    raise Exception(f"Simulation integrator failed with status {status_sim}.")
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]

                # Accumulate the position error over all sample points
                position_error = np.linalg.norm(estimated_state[:3] - self.observed_state_history[-(30 - i)][:3])
                total_position_error += position_error

            # Calculate the average position error for the current delay
            avg_position_error = total_position_error / len(delayed_control_history)
            error_latencies.append((delay, round(avg_position_error, 3)))  # Save delay and its corresponding rounded error

            if avg_position_error < min_error:
                min_error = avg_position_error
                optimal_delay = delay

        # Apply a low-pass filter to smooth the delay value
        alpha = 0.05  # Reduced low-pass filter coefficient for slower updates
        #self.delay_states_float = getattr(self, 'delay_states_float', float(self.delay_states))  # Initialize if not present
        self.delay_states_float = (1 - alpha) * self.delay_states_float + alpha * optimal_delay
        self.delay_states = round(self.delay_states_float)

        #print(f"Updated delay_states to {self.delay_states} with minimum average position error {round(min_error, 3)}")
        #print("Error latencies:", error_latencies)
        



    def control_loop(self):


        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1


        elif self.armed and self.current_pose is not None:

            ### CHECK FOR END OF TRAJECTORY
            if self.step_counter + self.N * self.skip_steps > self.steps:
                self.step_counter = 0
                self.armed = False
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cmd_publisher_.publish(msg)
                self.on_close()


            ### SET REFERENCE TRAJECTORY
            for j in range(self.N):
                sc = self.step_counter + j * self.skip_steps
                yref = np.concatenate((self.traj[:, sc], [0.0, 0.0, 0.0, 0.0]))
                self.ocp.set(j, "yref", yref)
                self.ocp.set(j, "p", self.est_params)

            sn = self.step_counter + self.N * self.skip_steps
            yref_N = self.traj[:, sn]
            self.ocp.set(self.N, "yref", yref_N)
            self.ocp.set(self.N, "p", self.est_params)


            ### ESTIMATE CURRENT STATE AFTER DELAY
            #estimated_state = copy.deepcopy(self.x_est[:13])
            estimated_state = copy.deepcopy(self.current_pose[:13])


            
            delayed_control_history = self.control_history[-self.delay_states:-1]

            for i, val in enumerate(delayed_control_history):
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(val[0:4]).flatten())))  # Ensure state dimension matches expected size
                self.sim_integrator.set("u", np.array(val[4:8]))  # Ensure control input dimension matches expected size
                self.sim_integrator.set("p", self.est_params)
                status_sim = self.sim_integrator.solve()
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]

            #print(f"Step {self.step_counter + 1}/{self.steps} **************************************************************8")
            #print(f"Observed state: {[round(val, 3) for val in self.current_pose[0:13]]}")
            #print(f"Estimated state: {[round(val, 3) for val in estimated_state[0:13]]}")
            #print(f"Actual state: {[round(val, 3) for val in self.motion_capture_pose[0:13]]}")

            # Ensure both inputs to np.concatenate are 1D arrays
            estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4])))  # Use the last control input for estimation

            
            relaxation_factor = 0.25
            relaxed_lbx = estimated_state_with_control * (1 - relaxation_factor)
            relaxed_ubx = estimated_state_with_control * (1 + relaxation_factor)
            self.ocp.set(0, "lbx", relaxed_lbx)
            self.ocp.set(0, "ubx", relaxed_ubx)


            ### SOLVE OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]  # Extract the last 4 elements as control inputs
            u_rate = self.ocp.get(0, "u")

            





            ### UKF predict

            old_u = np.array(self.control_history[-self.delay_states][0:4])
            old_u_rate = np.array(self.control_history[-self.delay_states][4:8])
            sigma_pts, wm, wc = self.generate_sigma_points(self.x_est, self.P, self.alpha, self.beta, self.kappa)
            sigma_pts_pred = np.array([
                self.fx(pt, old_u, old_u_rate) for pt in sigma_pts
            ])
            x_pred, P_pred = self.unscented_transform(sigma_pts_pred, wm, wc, self.Q)

            ### UKF update
            sigma_meas = np.array([self.hx(pt) for pt in sigma_pts_pred])
            z_pred, P_zz = self.unscented_transform(sigma_meas, wm, wc, self.R)
            P_xz = np.zeros((x_pred.size, z_pred.size))
            for i in range(sigma_pts.shape[0]):
                dx = sigma_pts_pred[i] - x_pred
                dz = sigma_meas[i] - z_pred
                P_xz += wc[i] * np.outer(dx, dz)

            K = P_xz @ np.linalg.inv(P_zz)
            self.x_est = x_pred + K @ ((self.current_pose[:13]) - z_pred)
            self.P = P_pred - K @ P_zz @ K.T

            ### Normalize quaternion to ensure it remains a valid unit quaternion
            quat_norm = np.linalg.norm(self.x_est[3:7])
            if quat_norm > 0:
                self.x_est[3:7] = self.x_est[3:7] / quat_norm




            ### Update estimated parameters
            self.est_params = np.array([self.x_est[13]])



            ### Append control inputs and rates to control history as separate elements
            self.control_history.append(u.tolist() + u_rate.tolist())
            self.observed_state_history.append(self.current_pose.tolist())

            if (self.motion_capture_pose is not None):
                self.motion_capture_history.append(self.motion_capture_pose.tolist())
            self.parameter_estimation_history.append(self.est_params.tolist())
            self.estimated_state_history.append(estimated_state.tolist())
            self.delay_state_estimation_history.append(self.delay_states)
            self.UKF_state_estimation_history.append(self.x_est[:13].tolist())


            print(f"Height {round(self.current_pose[2],3)} throttle {round(u[2],3)} estimated thrust ratio {round(self.est_params[0],3)} estimated delay states {self.delay_states}")
            print(f"u: {[round(val, 3) for val in u]} u_rate: {[round(val, 3) for val in u_rate]}")
            ### SEND COMMANDS
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)
            self.step_counter += 1

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0


    # --- UKF Functions ---
    def fx(self, x, u, u_rate):
        # Extract state variables from pt
        pos, quat, vel, ang_vel, ratio = x[:3], x[3:7], x[7:10], x[10:13], x[13] 
        state = np.concatenate((pos, quat, vel, ang_vel, u))
        param = np.array([ratio])

        # Set the state, input, and parameters in the CasADi integrator
        self.sim_integrator.set("x", state)
        self.sim_integrator.set("u", u_rate)
        self.sim_integrator.set("p", param)

        # Perform the integration step
        self.sim_integrator.solve()

        # Retrieve the next state from the integrator
        x_next = self.sim_integrator.get("x")
        return np.concatenate((x_next[:13], param))  # Keep the thrust ratio constant during prediction

    def hx(self, x):
        return x[0:13]

    def generate_sigma_points(self, x, P, alpha, beta, kappa):
        n = len(x)
        lambda_ = alpha**2 * (n + kappa) - n
        sigma_points = [x]
        sqrt_P = cholesky((n + lambda_) * P, lower=True)
        for i in range(n):
            sigma_points.append(x + sqrt_P[:, i])
            sigma_points.append(x - sqrt_P[:, i])
        weights_mean = [lambda_ / (n + lambda_)] + [1 / (2 * (n + lambda_))] * 2 * n
        weights_cov = [lambda_ / (n + lambda_) + (1 - alpha**2 + beta)] + [1 / (2 * (n + lambda_))] * 2 * n
        return np.array(sigma_points), np.array(weights_mean), np.array(weights_cov)

    def unscented_transform(self, sigma_points, weights_mean, weights_cov, noise_cov=None):
        mean = np.sum(weights_mean[:, None] * sigma_points, axis=0)
        cov = np.zeros((mean.size, mean.size))
        for i in range(sigma_points.shape[0]):
            dx = sigma_points[i] - mean
            cov += weights_cov[i] * np.outer(dx, dx)
        if noise_cov is not None:
            cov += noise_cov
        return mean, cov


    def plotSystemResponse(self):
        # Convert control history to a numpy array and remove the first 10 commands
        control_history = np.array(self.control_history[10:])

        # Create a figure with six subplots
        figure, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(10, 24))

        # Plot all control actions on the first subplot
        ax1.plot(control_history[:, 0], label='Roll', color='blue')
        ax1.plot(control_history[:, 1], label='Pitch', color='orange')
        ax1.plot(control_history[:, 2], label='Throttle', color='green')
        ax1.plot(control_history[:, 3], label='Yaw', color='red')
        ax1.set_title('Control Actions over Time')
        ax1.set_ylabel('Control Values')
        ax1.set_xlabel('Time Steps')
        ax1.legend()

        # Extract state_history height, desired trajectory height, motion capture height, estimated state height, and UKF state height
        state_history = np.array(self.observed_state_history)
        trajectory_height = self.traj[2, :len(state_history)]  # Assuming z-axis is the height
        estimated_state_height = np.array(self.estimated_state_history)[:, 2]  # Extract z-axis from estimated state
        UKF_state_height = np.array(self.UKF_state_estimation_history)[:, 2]  # Extract z-axis from UKF state

        # Plot state_history height, desired trajectory height, motion capture height, estimated state height, and UKF state height on the second subplot
        ax2.plot(state_history[:, 2], label='Observed State Height', color='purple')
        ax2.plot(trajectory_height, label='Desired Trajectory Height', color='cyan', linestyle='dashed')
        if self.motion_capture_pose is not None:
            motion_capture_height = np.array(self.motion_capture_history)[:, 2]  # Extract z-axis from motion capture data
            ax2.plot(motion_capture_height, label='Motion Capture Height', color='magenta', linestyle='dotted')
        ax2.plot(estimated_state_height, label='Estimated State Height', color='green', linestyle='dashdot')
        ax2.plot(UKF_state_height, label='UKF State Height', color='orange', linestyle='solid')
        ax2.set_title('Height Comparison over Time')
        ax2.set_ylabel('Height (m)')
        ax2.set_xlabel('Time Steps')
        ax2.legend()

        # Plot the estimated parameter over time on the third subplot
        parameter_estimation = np.array(self.parameter_estimation_history).flatten()
        ax3.plot(parameter_estimation, label='Estimated Parameter', color='brown')
        ax3.set_title('Estimated Parameter over Time')
        ax3.set_ylabel('Parameter Value')
        ax3.set_xlabel('Time Steps')
        ax3.legend()

        # Plot the delay state estimation history on the fourth subplot
        delay_state_estimation = np.array(self.delay_state_estimation_history)
        ax4.plot(delay_state_estimation, label='Delay State Estimation', color='blue')
        ax4.set_title('Delay State Estimation over Time')
        ax4.set_ylabel('Delay States')
        ax4.set_xlabel('Time Steps')
        ax4.legend()

        # Plot the y-axis values on the fifth subplot
        trajectory_y = self.traj[1, :len(state_history)]  # Assuming y-axis is the second row
        estimated_state_y = np.array(self.estimated_state_history)[:, 1]  # Extract y-axis from estimated state
        UKF_state_y = np.array(self.UKF_state_estimation_history)[:, 1]  # Extract y-axis from UKF state

        ax5.plot(state_history[:, 1], label='Observed State Y', color='purple')
        ax5.plot(trajectory_y, label='Desired Trajectory Y', color='cyan', linestyle='dashed')
        if self.motion_capture_pose is not None:
            motion_capture_y = np.array(self.motion_capture_history)[:, 1]  # Extract y-axis from motion capture data
            ax5.plot(motion_capture_y, label='Motion Capture Y', color='magenta', linestyle='dotted')
        ax5.plot(estimated_state_y, label='Estimated State Y', color='green', linestyle='dashdot')
        ax5.plot(UKF_state_y, label='UKF State Y', color='orange', linestyle='solid')
        ax5.set_title('Y-Axis Comparison over Time')
        ax5.set_ylabel('Y-Axis (m)')
        ax5.set_xlabel('Time Steps')
        ax5.legend()

        # Plot the x-axis values on the sixth subplot
        trajectory_x = self.traj[0, :len(state_history)]  # Assuming x-axis is the first row
        estimated_state_x = np.array(self.estimated_state_history)[:, 0]  # Extract x-axis from estimated state
        UKF_state_x = np.array(self.UKF_state_estimation_history)[:, 0]  # Extract x-axis from UKF state

        ax6.plot(state_history[:, 0], label='Observed State X', color='purple')
        ax6.plot(trajectory_x, label='Desired Trajectory X', color='cyan', linestyle='dashed')
        if self.motion_capture_pose is not None:
            motion_capture_x = np.array(self.motion_capture_history)[:, 0]  # Extract x-axis from motion capture data
            ax6.plot(motion_capture_x, label='Motion Capture X', color='magenta', linestyle='dotted')
        ax6.plot(estimated_state_x, label='Estimated State X', color='green', linestyle='dashdot')
        ax6.plot(UKF_state_x, label='UKF State X', color='orange', linestyle='solid')
        ax6.set_title('X-Axis Comparison over Time')
        ax6.set_ylabel('X-Axis (m)')
        ax6.set_xlabel('Time Steps')
        ax6.legend()

        plt.tight_layout()
        plt.show()


    def signal_handler(self, sig, frame):
        self.on_close()

    def on_close(self):
        # Check if on_close has already been called
        if getattr(self, 'on_close_called', False):
            return

        self.on_close_called = True  # Set the flag to True

        # Save data to files at the end
        self.control_history_writer.writerows(self.control_history)
        self.observed_state_history_writer.writerows(self.observed_state_history)
        self.motion_capture_history_writer.writerows(self.motion_capture_history)
        self.parameter_estimation_history_writer.writerows(self.parameter_estimation_history)
        self.estimated_state_history_writer.writerows(self.estimated_state_history)
        self.delay_state_estimation_history_writer.writerows([[d] for d in self.delay_state_estimation_history])
        self.UKF_state_estimation_history_writer.writerows(self.UKF_state_estimation_history)

        # Close all files
        self.control_history_file.close()
        self.observed_state_history_file.close()
        self.motion_capture_history_file.close()
        self.parameter_estimation_history_file.close()
        self.estimated_state_history_file.close()
        self.delay_state_estimation_history_file.close()
        self.UKF_state_estimation_history_file.close()
        self.trajectory_file.close()

        # Plot system response and shutdown
        #self.plotSystemResponse()
        #self.gui.quit()
        #rclpy.shutdown()
        #sys.exit(0)
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
