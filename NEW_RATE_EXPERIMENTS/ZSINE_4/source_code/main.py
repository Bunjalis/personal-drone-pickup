import rclpy
import signal
import sys
import numpy as np
import copy
import math
import os
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
from .gui import GUI
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, light_circle_trajectory, yaw_trajectory, power_loop_trajectory, christmas_tree_spiral_trajectory
from .visualization import TrajectoryVisualizer
from .data_logger import DataLogger
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from geometry_msgs.msg import Pose, PoseArray
from scipy.linalg import cholesky
import matplotlib.pyplot as plt

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.orb_slam_state_subscription_ = self.create_subscription(MotionCaptureState, '/orb_slam_state', self.orb_slam_state_callback, 10)
        self.telemetry_subscription_ = self.create_subscription(Telemetry, '/telemetry', self.telemetry_callback, 10)
        self.trajectory_publisher_ = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.last_measured_pose = None
        self.motion_capture_pose = None
        self.orb_slam_pose = None
        self.current_pose = None
        self.battery_voltage = 0.0 

        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")

        self.ocp, self.sim_integrator = generate_ocp_controller()

        self.dt = 1.0 /30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.visualization_timer = self.create_timer(0.5, self.publish_trajectory_visualization)

        self.delay_estimation_timer = self.create_timer(1/10.0, self.delay_estimation_timer)

        self.traj = xyz_sine_trajectory(self.dt)  

        self.trajectory_visualizer.publish_all_visualizations( self.traj,  pose_subsample=10, show_velocity=True, velocity_scale=0.5,color_by_time=True  )

        self.USE_MOTION_CAPTURE = True 
        
        trial_name = "ZSINE_4"

        self.est_params = np.array([42.0, 0.5, 0.07, 100.0, 300.0, 0.5])

        self.steps = self.traj.shape[1] - 1

        self.gui = GUI(self)
        self.armed = False
        
        self.pre_start_duration = 2.0
        self.pre_start_counter = 0 
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 20
        self.skip_steps = 3

        self.first_solve = True
        self.data_logger = DataLogger(trial_name)
        self.copy_source_files_to_output()
        self.data_logger.save_trajectory(self.traj)

        
        self.alpha, self.beta, self.kappa = 0.1, 2, 0

        self.x_est = np.array([0.0, 0.0, 0.0, 
                                1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 
                                self.est_params[0], self.est_params[1], self.est_params[2], self.est_params[3], self.est_params[4], self.est_params[5]])
        self.P = np.diag([0.1, 0.1, 0.1,  # Increase initial uncertainty for position
                          0.1, 0.1, 0.1, 0.1,  # Quaternion
                          0.1, 0.1, 0.1,  # Velocity
                          0.1, 0.1, 0.1,  # Angular rates
                          0.5, 0.05, 0.2, 0.2, 0.2, 0.2])  # thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo uncertainties
        self.Q = np.diag([1e-4, 1e-4, 1e-4,  # Position process noise
                          1e-5, 1e-5, 1e-5, 1e-5,  # Quaternion process noise
                          1e-3, 1e-3, 1e-3,  # Velocity process noise
                          1e-3, 1e-3, 1e-3,  # Angular rates process noise
                          1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-6])  # thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo process noise
        self.R = np.diag([0.05]*13)


        self.delay_states = 1
        self.delay_states_float = float(self.delay_states)

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.motion_capture_pose = np.round(np.array([ p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z ]), 3)
        if self.USE_MOTION_CAPTURE:
            self.current_pose = self.motion_capture_pose
        

    def orb_slam_state_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.orb_slam_pose = np.round(np.array([ p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z ]), 3)
        if not self.USE_MOTION_CAPTURE:
            self.current_pose = self.orb_slam_pose
        

    def telemetry_callback(self, msg: Telemetry):
        self.battery_voltage = msg.battery_voltage

    def publish_trajectory_visualization(self):
        if hasattr(self, 'traj'):
            self.trajectory_visualizer.publish_all_visualizations( self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

    def copy_source_files_to_output(self):
        current_file = __file__
        if current_file.endswith('.pyc'):
            current_file = current_file[:-1] 
        
        source_files_info = {
            'main.py': current_file,
            'acados.py': os.path.join(os.path.dirname(current_file), 'acados.py'),
            'trajectories.py': os.path.join(os.path.dirname(current_file), 'trajectories.py'),
            'dynamics.py': os.path.join(os.path.dirname(current_file), 'dynamics.py'),
            'data_logger.py': os.path.join(os.path.dirname(current_file), 'data_logger.py')
        }
        
        self.data_logger.copy_source_files_to_output(source_files_info)


    def delay_estimation_timer(self):
        if len(self.data_logger.observed_state_history) < 30 or len(self.data_logger.control_history) < 30:
            return

        min_error = float('inf')
        optimal_delay = self.delay_states
        error_latencies = []

        # Iterate over possible delay values to find the one that minimizes the position error
        for delay in range(1, 12):  # Test delays from 1 to 10
            total_position_error = 0
            estimated_state = copy.deepcopy(self.data_logger.observed_state_history[-30])  # Start with the oldest state in the last 30
            delayed_control_history = self.data_logger.control_history[-(30 + delay):-delay]  # Use delayed controls

            for i, control in enumerate(delayed_control_history):
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(control[0:4]).flatten())))
                self.sim_integrator.set("u", np.array(control[4:8]))
                sim_p = np.concatenate([self.est_params, np.array([1.0, 0.0, 0.0, 0.0])])  # append q_ref (unused by dynamics)
                self.sim_integrator.set("p", sim_p)
                status_sim = self.sim_integrator.solve()
                if status_sim != 0:
                    raise Exception(f"Simulation integrator failed with status {status_sim}.")
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]

                # Accumulate the position error over all sample points
                position_error = np.linalg.norm(estimated_state[:3] - self.data_logger.observed_state_history[-(30 - i)][:3])
                total_position_error += position_error

            # Calculate the average position error for the current delay
            avg_position_error = total_position_error / len(delayed_control_history)
            error_latencies.append((delay, round(avg_position_error, 3)))  # Save delay and its corresponding rounded error

            if avg_position_error < min_error:
                min_error = avg_position_error
                optimal_delay = delay

        # Apply a low-pass filter to smooth the delay value
        alpha = 0.05  # Reduced low-pass filter coefficient for slower updates
        #self.delay_states_float = (1 - alpha) * self.delay_states_float + alpha * optimal_delay
        #self.delay_states = round(self.delay_states_float)

        print(f"Updated delay_states to {self.delay_states} with minimum average position error {round(min_error, 3)}")
        #print("Error latencies:", error_latencies)
        



    def control_loop(self):
        start_time = time.time()

        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1
            self.data_logger.log_battery_voltage(self.battery_voltage)


        elif self.armed and self.current_pose is not None:

            ### CHECK FOR END OF TRAJECTORY
            if self.step_counter + self.N * self.skip_steps > self.steps:
                self.step_counter = 0
                self.armed = False
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cmd_publisher_.publish(msg)
                self.on_close()

            # Update OCP parameters with current estimates
            update_ocp_parameters(self.ocp, self.est_params, self.N)
            
            set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)

            ### ESTIMATE CURRENT STATE AFTER DELAY
            #estimated_state = copy.deepcopy(self.x_est[:13])
            estimated_state = copy.deepcopy(self.current_pose[:13])


            delayed_control_history = self.data_logger.control_history[-self.delay_states:-1]

            for i, val in enumerate(delayed_control_history):
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(val[0:4]).flatten()))) 
                self.sim_integrator.set("u", np.array(val[4:8])) 
                sim_p = np.concatenate([self.est_params, np.array([1.0, 0.0, 0.0, 0.0])])  # append q_ref (unused by dynamics)
                self.sim_integrator.set("p", sim_p)
                status_sim = self.sim_integrator.solve()
                if status_sim != 0:
                    raise Exception(f"Simulation integrator failed with status {status_sim}.")
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]


            # Ensure both inputs to np.concatenate are 1D arrays
            estimated_state_with_control = np.concatenate((estimated_state, np.array(self.data_logger.control_history[-1][0:4]))) 

            relaxation_factor = 0.01 # 0.25 for orb slam 
            relaxed_lbx = estimated_state_with_control * (1 - relaxation_factor)
            relaxed_ubx = estimated_state_with_control * (1 + relaxation_factor)
            self.ocp.set(0, "lbx", relaxed_lbx)
            self.ocp.set(0, "ubx", relaxed_ubx)

            ### MPC WARM START
            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)


            ### SOLVE OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")


            ### SEND COMMANDS
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))

            print(f"1: {round(u[0], 3)}, 2: {round(u[1], 3)}, 3: {round((u[2]), 3)}, 4: {round(u[3], 3)}")
            print(f"Estimated params - Thrust ratio: {round(self.est_params[0],2)}, Drag coeff z: {round(self.est_params[1],3)}, Tau rate: {round(self.est_params[2],3)}, Centre rate deg: {round(self.est_params[3],1)}, Max rate deg: {round(self.est_params[4],1)}, Rate expo: {round(self.est_params[5],3)}")
            self.cmd_publisher_.publish(msg)
            
            # Extract MPC trajectory for visualization
            mpc_trajectory = np.zeros((13, self.N))
            for i in range(self.N):
                x_i = self.ocp.get(i, "x")
                mpc_trajectory[:, i] = x_i[:13]
            
            # Replace the first state with the current measured pose to eliminate offset
            mpc_trajectory[:, 0] = self.current_pose[:13]
            
            # Publish MPC plan visualization
            self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            self.trajectory_visualizer.publish_transform_frame(self.orb_slam_pose, "drone_orbslam")
            self.trajectory_visualizer.publish_transform_frame(self.motion_capture_pose, "drone_mocap")

            ### UKF predict
            old_u = np.array(self.data_logger.control_history[-self.delay_states][0:4])
            old_u_rate = np.array(self.data_logger.control_history[-self.delay_states][4:8])
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
            
            # Ensure P remains positive definite
            self.P = 0.5 * (self.P + self.P.T)  # Make symmetric
            eigenvals = np.linalg.eigvals(self.P)
            if np.min(eigenvals) < 1e-8:
                print("Warning: Covariance matrix becoming singular, adding regularization")
                self.P += np.eye(self.P.shape[0]) * 1e-5

            ### Normalize quaternion to ensure it remains a valid unit quaternion
            quat_norm = np.linalg.norm(self.x_est[3:7])
            if quat_norm > 0:
                self.x_est[3:7] = self.x_est[3:7] / quat_norm

            ### Constrain parameters to physically reasonable bounds
            self.x_est[13] = np.clip(self.x_est[13], 20.0, 60.0)
            self.x_est[14] = np.clip(self.x_est[14], 0.01, 1.0)
            self.x_est[15] = np.clip(self.x_est[15], 0.04, 0.3)
            self.x_est[16] = np.clip(self.x_est[16], 0.0, 1000.0)
            self.x_est[17] = np.clip(self.x_est[17], 0.0, 1000.0)
            self.x_est[18] = np.clip(self.x_est[18], 0.5, 0.5)

            if self.x_est[17] <= self.x_est[16]:
                self.x_est[17] = self.x_est[16]

            ### Update estimated parameters
            self.est_params = np.array([self.x_est[13], self.x_est[14], self.x_est[15], self.x_est[16], self.x_est[17], self.x_est[18]])

            ### Log data using data logger
            self.data_logger.log_control_data(u, u_rate)
            self.data_logger.log_observed_state(self.current_pose)
            self.data_logger.log_motion_capture_state(self.motion_capture_pose)
            self.data_logger.log_parameter_estimation(self.est_params)
            self.data_logger.log_estimated_state(estimated_state)
            self.data_logger.log_delay_estimation(self.delay_states)
            self.data_logger.log_ukf_state(self.x_est)
            self.data_logger.log_battery_voltage(self.battery_voltage)
            
            # Publish actual path visualization (dotted red line)
            self.trajectory_visualizer.publish_actual_path(self.current_pose)
            self.step_counter += 1

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0
            # Record battery voltage during disarmed phase
            self.data_logger.log_battery_voltage(self.battery_voltage)

        # Calculate and print control loop execution time
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
        #print(f"Control loop execution time: {execution_time:.2f} ms")
        
        # Record the execution time
        self.data_logger.log_control_timing(execution_time)


    # --- UKF Functions ---
    def fx(self, x, u, u_rate):
        # Extract state variables from pt
        pos, quat, vel, ang_vel = x[:3], x[3:7], x[7:10], x[10:13]
        thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo = x[13], x[14], x[15], x[16], x[17], x[18]
        state = np.concatenate((pos, quat, vel, ang_vel, u))
        param = np.array([thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo])

        # Set the state, input, and parameters in the CasADi integrator
        self.sim_integrator.set("x", state)
        self.sim_integrator.set("u", u_rate)
        sim_p = np.concatenate([param, np.array([1.0, 0.0, 0.0, 0.0])])  # use current parameter estimate for this sigma point
        self.sim_integrator.set("p", sim_p)

        # Perform the integration step
        self.sim_integrator.solve()

        # Retrieve the next state from the integrator
        x_next = self.sim_integrator.get("x")
        return np.concatenate((x_next[:13], param))  # Keep all parameters constant during prediction

    def hx(self, x):
        return x[0:13]

    def generate_sigma_points(self, x, P, alpha, beta, kappa):
        n = len(x)
        lambda_ = alpha**2 * (n + kappa) - n
        sigma_points = [x]
        
        # Add numerical stability to the covariance matrix
        P_stable = P + np.eye(n) * 1e-9  # Add small regularization
        
        # Check if matrix is positive definite
        try:
            sqrt_P = cholesky((n + lambda_) * P_stable, lower=True)
        except np.linalg.LinAlgError:
            print("Warning: Covariance matrix not positive definite, using eigenvalue decomposition")
            # Use eigenvalue decomposition as fallback
            eigenvals, eigenvecs = np.linalg.eigh((n + lambda_) * P_stable)
            eigenvals = np.maximum(eigenvals, 1e-9)  # Ensure positive eigenvalues
            sqrt_P = eigenvecs @ np.diag(np.sqrt(eigenvals))
        
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


    def signal_handler(self, sig, frame):
        print("Interrupt received, shutting down...")
        self.on_close()
        sys.exit(0)

    def on_close(self):
        # Check if on_close has already been called
        if getattr(self, 'on_close_called', False):
            return

        self.on_close_called = True  # Set the flag to True

        print("Saving data and shutting down...")
        
        # Disarm the drone first
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
        self.cmd_publisher_.publish(msg)

        # Save all data using the data logger
        self.data_logger.close_and_save()

        # Plot system response and shutdown
        #self.plotSystemResponse()
        
        # Close GUI properly
        try:
            self.gui.quit()
        except:
            pass
            
        # Shutdown ROS properly
        try:
            self.destroy_node()
            rclpy.shutdown()
        except:
            pass




def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.1)
            controller.gui.handle_events()
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        controller.on_close()

if __name__ == '__main__':
    main()
