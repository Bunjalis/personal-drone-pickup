import rclpy
import signal
import sys
import numpy as np
import copy
import math
import os
import threading
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
from .acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
from .trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory, fence_trajectory,power_loop_trajectory
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from scipy.linalg import cholesky


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = True  # Set to False to use ORB-SLAM data instead
FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_ukf'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.cb = CallbackManager(self)

        self.traj, trajectory_name = circle_trajectory(DT)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(self.traj,  pose_subsample=15, show_velocity=False,  velocity_scale=0.3, color_by_time=True )

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False


        # MPC settings
        self.N = 20
        self.skip_steps = 3
        self.first_solve = True
        self.ocp, self.sim_integrator = generate_ocp_controller()


        # ORB-Slam interface 
        self.orb_slam_pose = None
        self.orb_slam_state_subscription_ = self.create_subscription(MotionCaptureState, '/orb_slam_state', self.orb_slam_state_callback, 10)

        
        # UKF settings
        self.est_params = np.array([38.0, 0.0, 0.12,200.0, 600.0, 0.5])

        self.alpha, self.beta, self.kappa = 0.1, 2, 0

        self.x_est = np.array([0.0, 0.0, 0.0, 
                                1.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 
                                self.est_params[0], self.est_params[1], self.est_params[2], self.est_params[3], self.est_params[4], self.est_params[5]])
        self.P = np.diag([0.1, 0.1, 0.1,
                          0.1, 0.1, 0.1, 0.1,
                          0.1, 0.1, 0.1, 
                          0.1, 0.1, 0.1, 
                          0.1, 0.1, 0.1, 0.1, 0.1, 0.1]) 
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 
                          1e-5, 1e-5, 1e-5, 1e-5,
                          1e-3, 1e-3, 1e-3,
                          1e-3, 1e-3, 1e-3,
                          1e-4, 1e-4, 1e-4, 1, 1, 0.1])
        self.R = np.diag([0.05]*13)


        # Delay estimation
        self.delay_states = 1
        self.delay_states_float = float(self.delay_states)
        self.delay_estimation_timer = self.create_timer(1/10.0, self.delay_estimation_timer)


        # Logging
        log_headers = [
            'step', 'timestamp', 'u0', 'u1', 'u2', 'u3',
            'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx', 'pose_qy', 'pose_qz',
            'MPC_setup_time', 'MPC_solve_time', 'Visualisation_time', 'UKF_update_time',
        ]
        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.observed_state_history = []       
        self.control_history = []
        self.estimated_state_history = []
        self.UKF_state_estimation_history = []


    def orb_slam_state_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.orb_slam_pose = np.round(np.array([ p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z ]), 3)
        if not USE_MOTION_CAPTURE:
            self.current_pose = self.orb_slam_pose
            self.last_pose_update_time = time.time() 
        

    def delay_estimation_timer(self):
        print(f"test")
        '''
        if len(self.observed_state_history) < 30 or len(self.control_history) < 30:
            return

        min_error = float('inf')
        optimal_delay = self.delay_states
        error_latencies = []

        # Iterate over possible delay values to find the one that minimizes the position error
        for delay in range(1, 12):  # Test delays from 1 to 10
            total_position_error = 0
            estimated_state = copy.deepcopy(self.observed_state_history[-30])  # Start with the oldest state in the last 30
            delayed_control_history = self.control_history[-(30 + delay):-delay]  # Use delayed controls

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
        #self.delay_states_float = (1 - alpha) * self.delay_states_float + alpha * optimal_delay
        #self.delay_states = round(self.delay_states_float)

        #print(f"Updated delay_states to {self.delay_states} with minimum average position error {round(min_error, 3)}")
        #print("Error latencies:", error_latencies)

        '''
        



    def control_loop(self):


        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return 

        if self.armed and self.current_pose is not None:
            ### CHECK FOR END OF TRAJECTORY
            if self.step_counter + self.N * self.skip_steps > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return
            
            start_time = time.time()

            # Update OCP parameters with current estimates
            update_ocp_parameters(self.ocp, self.est_params, self.N)
            set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params)

            ### ESTIMATE CURRENT STATE AFTER DELAY
            #estimated_state = copy.deepcopy(self.x_est[:13])
            estimated_state = copy.deepcopy(self.current_pose[:13])

            if len(self.control_history) <= 0:
                delayed_control_history = [estimated_state]
            else:
                delayed_control_history = self.control_history[-self.delay_states:-1]   


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

            # Only correct the drones velocity, while keeping the position the same as the observed value
            estimated_state[0:7] = self.current_pose[0:7]
            if len(self.control_history) == 0:
                self.get_logger().warn("Control history is empty - cannot set state bounds accurately")
                estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0]))) 
            else:
                estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4]))) 


            
            relaxation_factor = 0.025 # 0.25 for orb slam 
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

            mpc_setup_time = time.time()

            ### SOLVE OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")


            mpc_solve_time = time.time()




            ### SEND COMMANDS
            # Only execute trajectory if takeoff has been requested
            if self.takeoff_requested:
                msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
                #print(f"r: {round(u[0], 3)}, p: {round(u[1], 3)}, t: {round((u[2]), 3)}, y: {round(u[3], 3)}")
                print(f"EST. params - TR: {round(self.est_params[0],2)}, DC z: {round(self.est_params[1],3)}, Tau: {round(self.est_params[2],3)}, Centre deg: {round(self.est_params[3],1)}, Max deg: {round(self.est_params[4],1)}, expo: {round(self.est_params[5],3)}")
            else:
                # Stay armed but don't send thrust commands until takeoff
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                #print("Armed - Waiting for TAKEOFF command")
            
            self.cb.cmd_publisher_.publish(msg)



            
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
            self.trajectory_visualizer.publish_transform_frame(self.current_pose, "drone_mocap")
            self.trajectory_visualizer.publish_actual_path(self.current_pose)

            send_command_and_visualisation = time.time()


            ### UKF predict and update - only when actually flying
            if self.takeoff_requested:
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


            end_ukf_time = time.time()

            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(self.current_pose[0]), float(self.current_pose[1]), float(self.current_pose[2]),
                float(self.current_pose[3]), float(self.current_pose[4]), float(self.current_pose[5]), float(self.current_pose[6]),
                round(mpc_setup_time - start_time, 4),
                round(mpc_solve_time - mpc_setup_time, 4),
                round(send_command_and_visualisation - mpc_solve_time, 4),
                round(end_ukf_time - send_command_and_visualisation, 4),
            ]
            self.data_logger.append_row(log_row)

            # Only increment step counter if takeoff was requested
            if self.takeoff_requested:
                self.step_counter += 1

            self.control_history.append( np.concatenate( (u, u_rate) ).tolist() )
            self.observed_state_history.append( self.current_pose[:13].tolist() )
            self.estimated_state_history.append( estimated_state[:13].tolist() )
            self.UKF_state_estimation_history.append( self.x_est.tolist() )


            saved_data = time.time()

            print(f"time taken to log {round(saved_data - end_ukf_time, 4)} seconds")

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0


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
        if getattr(self, 'on_close_called', False):
            return
        self.on_close_called = True
        msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
        self.cb.cmd_publisher_.publish(msg)
        self.data_logger.close()


def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        controller.on_close()
        # Final cleanup
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
