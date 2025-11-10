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
from .acados import (
    generate_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_trajectory_reference_aligned,
    update_ocp_parameters,
)
from .trajectories import (
    hover_trajectory,
    z_sin_trajectory,
    xyz_sine_trajectory,
    circle_trajectory,
    power_loop_trajectory,
    figure8_zsine_trajectory,
    fast_xyz_sine_trajectory,
    fence_trajectory,
    power_loop_trajectory,
)
from utility_objects.visualization import TrajectoryVisualizer
from utility_objects.data_logger import DataLogger
from utility_objects.callback_manager import CallbackManager
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry
from scipy.linalg import cholesky
from interfaces.msg import ControlApplied
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32


POSE_TIMEOUT_THRESHOLD = 0.25  # seconds
USE_MOTION_CAPTURE = False      # Set to False to use ORB-SLAM data instead
FREQUENCY_HZ = 15
DT = 1.0 / FREQUENCY_HZ

LOGGING_NAME = 'controller_angle_ukf'

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # General Settings
        self.cb = CallbackManager(self, USE_MOTION_CAPTURE)

        self.traj, trajectory_name = circle_trajectory(DT)
        self.trajectory_visualizer = TrajectoryVisualizer(self, frame_id="map")
        self.trajectory_visualizer.publish_all_visualizations(
            self.traj, pose_subsample=15, show_velocity=False, velocity_scale=0.3, color_by_time=True
        )

        self.timer = self.create_timer(DT, self.control_loop)
        self.step_counter = 0
        self.steps = self.traj.shape[1] - 1

        self.armed = False
        self.takeoff_requested = False
        self.shutdown_requested = False

        # MPC settings
        self.N = 15
        self.skip_steps = 3
        self.first_solve = True
        self.ocp, self.sim_integrator = generate_ocp_controller(dt=DT, N_horizon=self.N, skip_steps=self.skip_steps)

        # ORB-Slam interface
        self.orb_slam_pose = [0,0,0,1,0,0,0,0,0,0,0,0,0]
        self.orb_slam_state_subscription_ = self.create_subscription(
            MotionCaptureState, '/orb_slam_state', self.orb_slam_state_callback, 10
        )

        # ---------------------------
        # UKF settings (UPDATED)
        # ---------------------------
        # est_params order (8):
        # [kT, dragZ, tau_rate, centre_rate_deg, max_rate_deg, rate_expo, angle_max_deg, tau_angle]
        self.est_params = np.array([35.0, 0.2, 0.12, 100.0, 100.0, 0.5, 55.0, 0.08], dtype=float)

        self.alpha, self.beta, self.kappa = 0.1, 2, 0

        # State x_est: [p(3), q(4), v(3), w(3), params(8)] = 21
        self.x_est = np.array([
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            *self.est_params  # 8 params
        ], dtype=float)


        # Covariances sized to 21x21
        self.P = np.diag([
            0.1, 0.1, 0.1,
            0.1, 0.1, 0.1, 0.1,
            0.1, 0.1, 0.1,
            0.1, 0.1, 0.1,
            0.1, 0.001, 0.001,  # kT, dragZ, tau_rate
            0.01, 0.01, 0.01,  # centre, max, expo
            0.01, 0.001        # angle_max_deg, tau_angle
        ]).astype(float)

        self.Q = np.diag([
            # process noise for measured states
            1e-4, 1e-4, 1e-4,           # p
            1e-5, 1e-5, 1e-5, 1e-5,     # q
            1e-3, 1e-3, 1e-3,           # v
            1e-3, 1e-3, 1e-3,           # w
            # params (slower drift)
            1e-3, 1e-5, 1e-5,           # kT, dragZ, tau_rate
            1e-3, 1e-3, 1e-3,           # centre, max, expo
            1e-3, 1e-5                  # angle_max_deg, tau_angle
        ]).astype(float)

        # Measurement: 13 (p,q,v,w)
        self.R = np.diag([0.05]*13).astype(float)

        # Measurement: 13 (p,q,v,w)

        # Delay estimation
        self.delay_states = 1
        qos1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub_ctrl_applied = self.create_publisher(ControlApplied, '/control_applied', qos1)
        self.create_subscription(Int32, '/estimated_delay',
                                 lambda m: setattr(self, 'delay_states', int(m.data)),
                                 qos1)

        # Logging
        log_headers = [
            'step', 'timestamp',
            'u0','u1','u2','u3','u0_rate', 'u1_rate', 'u2_rate', 'u3_rate',

            'orb_pose_x', 'orb_pose_y', 'orb_pose_z', 'orb_pose_qw', 'orb_pose_qx', 'orb_pose_qy', 'orb_pose_qz',
            'orb_pose_vx', 'orb_pose_vy', 'orb_pose_vz','orb_pose_avx', 'orb_pose_avy', 'orb_pose_avz',

            'mot_pose_x', 'mot_pose_y', 'mot_pose_z', 'mot_pose_qw', 'mot_pose_qx', 'mot_pose_qy', 'mot_pose_qz',
            'mot_pose_vx', 'mot_pose_vy', 'mot_pose_vz','mot_pose_avx', 'mot_pose_avy', 'mot_pose_avz',

            'est_pose_x', 'est_pose_y', 'est_pose_z', 'est_pose_qw', 'est_pose_qx', 'est_pose_qy', 'est_pose_qz',
            'est_pose_vx', 'est_pose_vy', 'est_pose_vz','est_pose_avx', 'est_pose_avy', 'est_pose_avz',

            'traj_x_ref', 'traj_y_ref', 'traj_z_ref', 'traj_qw_ref', 'traj_qx_ref', 'traj_qy_ref', 'traj_qz_ref',
            'est_param_thrust_ratio', 'est_param_drag_coeff_z', 'est_param_tau_rate',
            'est_param_centre_rate_deg', 'est_param_max_rate_deg', 'est_param_rate_expo',
            'est_param_angle_max_deg', 'est_param_tau_angle',
            'MPC_setup_time', 'MPC_solve_time', 'Visualisation_time', 'UKF_update_time',
        ]

        self.data_logger = DataLogger(LOGGING_NAME, trajectory_name, log_headers)

        self.observed_state_history = []
        self.control_history = []
        self.estimated_state_history = []
        self.UKF_state_estimation_history = []

    def orb_slam_state_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.orb_slam_pose = np.round(np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        ]), 3)
        if not USE_MOTION_CAPTURE:
            self.current_pose = self.orb_slam_pose
            self.last_pose_update_time = time.time()

    def control_loop(self):
        if self.shutdown_requested:
            self.cb.request_shutdown()
            return

        if self.armed and (time.time() - self.last_pose_update_time) > POSE_TIMEOUT_THRESHOLD:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.disarm(msg)
            return

        if self.armed and self.current_pose is not None:
            # End of trajectory
            if self.step_counter + self.N * self.skip_steps > self.steps:
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cb.disarm(msg)
                self.cb.request_shutdown()
                return

            start_time = time.time()

            # Update OCP parameters with current estimates (8 params)
            update_ocp_parameters(self.ocp, self.est_params, self.N)
            set_trajectory_reference_aligned(
                self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params
            )

            # ---- Delay-compensated state roll-forward using sim_integrator ----
            estimated_state = copy.deepcopy(self.current_pose[:13])

            if len(self.control_history) <= 0:
                delayed_control_history = [estimated_state]
            else:
                delayed_control_history = self.control_history[-self.delay_states:-1]

            for i, val in enumerate(delayed_control_history):
                # integrator state = [p(3), q(4), v(3), w(3), u(4)]
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(val[0:4]).flatten())))
                self.sim_integrator.set("u", np.array(val[4:8]))
                # params = 8 dyn + 4 qref (qref unused in dynamics)
                sim_p = np.concatenate([self.est_params, np.array([1.0, 0.0, 0.0, 0.0])])
                self.sim_integrator.set("p", sim_p)
                status_sim = self.sim_integrator.solve()
                if status_sim != 0:
                    raise Exception(f"Simulation integrator failed with status {status_sim}.")
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]



            # Tight initial-state box (softened)
            if len(self.control_history) == 0:
                self.get_logger().warn("Control history is empty - cannot set state bounds accurately")
                estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0])))
            else:
                estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4])))

            # Set tight bounds for the initial state, but relax parts of it
            lbx = estimated_state_with_control.copy()
            ubx = estimated_state_with_control.copy()

            relaxation_factor = 0.00000001 # 0.25 for orb slam

            # Indices for pose (p), linear velocity (v), and angular velocity (w)
            pos_indices = [0, 1, 2]
            vel_indices = [7, 8, 9]
            ang_vel_indices = [10, 11, 12]
            
            indices_to_relax = pos_indices + vel_indices + ang_vel_indices

            for i in indices_to_relax:
                val = estimated_state_with_control[i]
                # Avoid issues with near-zero values creating a zero-width bound
                delta = abs(val * relaxation_factor) if abs(val) > 1e-6 else relaxation_factor
                lbx[i] = val - delta
                ubx[i] = val + delta

            self.ocp.set(0, "lbx", lbx)
            self.ocp.set(0, "ubx", ubx)

            # Warm start
            if self.first_solve:
                set_initial_guess(self.ocp, self.N)
                self.first_solve = False
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            mpc_setup_time = time.time()

            # Solve OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]
            u_rate = self.ocp.get(0, "u")

            print(f"Step {self.step_counter}: Control u = {u}")

            mpc_solve_time = time.time()

            # Send commands
            if self.takeoff_requested:
                msg = ELRSCommand(
                    armed=True,
                    channel_0=round(u[0], 3),
                    channel_1=round(u[1], 3),
                    channel_2=round((u[2]*2)-1, 3),  # throttle [0..1] → [-1..1]
                    channel_3=round(u[3], 3)
                )
            else:
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)

            self.cb.cmd_publisher_.publish(msg)

            # MPC plan visualization
            mpc_trajectory = np.zeros((13, self.N))
            for i in range(self.N):
                x_i = self.ocp.get(i, "x")
                mpc_trajectory[:, i] = x_i[:13]
            mpc_trajectory[:, 0] = self.current_pose[:13]
            self.trajectory_visualizer.publish_mpc_plan(mpc_trajectory)
            self.trajectory_visualizer.publish_transform_frame(self.orb_slam_pose, "drone_orbslam")
            self.trajectory_visualizer.publish_transform_frame(self.cb.motion_capture_pose, "drone_mocap")
            self.trajectory_visualizer.publish_actual_path(self.current_pose)

            send_command_and_visualisation = time.time()

            # -------- UKF predict & update (only when flying) --------
            if self.takeoff_requested:
                old_u = np.array(self.control_history[-self.delay_states][0:4])
                old_u_rate = np.array(self.control_history[-self.delay_states][4:8])

                sigma_pts, wm, wc = self.generate_sigma_points(self.x_est, self.P, self.alpha, self.beta, self.kappa)
                sigma_pts_pred = np.array([self.fx(pt, old_u, old_u_rate) for pt in sigma_pts])
                x_pred, P_pred = self.unscented_transform(sigma_pts_pred, wm, wc, self.Q)

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
                self.P = 0.5 * (self.P + self.P.T)  # symmetry

                # Regularize if needed
                eigenvals = np.linalg.eigvals(self.P)
                if np.min(eigenvals) < 1e-8:
                    print("Warning: Covariance matrix becoming singular, adding regularization")
                    self.P += np.eye(self.P.shape[0]) * 1e-5

                # Normalize quaternion
                quat_norm = np.linalg.norm(self.x_est[3:7])
                if quat_norm > 0:
                    self.x_est[3:7] = self.x_est[3:7] / quat_norm

                # -------- Parameter clamping (UPDATED) --------
                # kT
                self.x_est[13] = np.clip(self.x_est[13], 18.0, 60.0)
                # dragZ
                self.x_est[14] = np.clip(self.x_est[14], 0.01, 0.5)
                # tau_rate
                self.x_est[15] = np.clip(self.x_est[15], 0.07, 0.3)
                # centre_rate_deg (TIGHTENED: was 0..1000)
                self.x_est[16] = np.clip(self.x_est[16], 50.0, 150.0)
                # max_rate_deg (TIGHTENED: was 0..1000)
                self.x_est[17] = np.clip(self.x_est[17], 100.0, 400.0)
                # rate_expo (keep within reasonable range)
                self.x_est[18] = np.clip(self.x_est[18], 0.1, 1.0)

                # ensure max >= centre
                if self.x_est[17] <= self.x_est[16]:
                    self.x_est[17] = self.x_est[16] + 10.0

                # angle_max_deg (TIGHTENED: was 20..85)
                self.x_est[19] = np.clip(self.x_est[19], 40.0, 70.0)
                # tau_angle
                self.x_est[20] = np.clip(self.x_est[20], 0.07, 0.3)

                # Update estimated parameters vector (8)
                self.est_params = np.array([
                    self.x_est[13], self.x_est[14], self.x_est[15],
                    self.x_est[16], self.x_est[17], self.x_est[18],
                    self.x_est[19], self.x_est[20]
                ], dtype=float)


            print("Estimated params:", self.est_params)
            end_ukf_time = time.time()

            # Logging
            log_row = [
                self.step_counter,
                time.time(),
                float(u[0]), float(u[1]), float(u[2]), float(u[3]),
                float(u_rate[0]), float(u_rate[1]), float(u_rate[2]), float(u_rate[3]),
                float(self.orb_slam_pose[0]), float(self.orb_slam_pose[1]), float(self.orb_slam_pose[2]),
                float(self.orb_slam_pose[3]), float(self.orb_slam_pose[4]), float(self.orb_slam_pose[5]), float(self.orb_slam_pose[6]),
                float(self.orb_slam_pose[7]), float(self.orb_slam_pose[8]), float(self.orb_slam_pose[9]),
                float(self.orb_slam_pose[10]), float(self.orb_slam_pose[11]), float(self.orb_slam_pose[12]),
                
                float(self.cb.motion_capture_pose[0]), float(self.cb.motion_capture_pose[1]), float(self.cb.motion_capture_pose[2]),
                float(self.cb.motion_capture_pose[3]), float(self.cb.motion_capture_pose[4]), float(self.cb.motion_capture_pose[5]), float(self.cb.motion_capture_pose[6]),
                float(self.cb.motion_capture_pose[7]), float(self.cb.motion_capture_pose[8]), float(self.cb.motion_capture_pose[9]),
                float(self.cb.motion_capture_pose[10]), float(self.cb.motion_capture_pose[11]), float(self.cb.motion_capture_pose[12]),

                float(estimated_state[0]), float(estimated_state[1]), float(estimated_state[2]),
                float(estimated_state[3]), float(estimated_state[4]), float(estimated_state[5]), float(estimated_state[6]),
                float(estimated_state[7]), float(estimated_state[8]), float(estimated_state[9]),
                float(estimated_state[10]), float(estimated_state[11]), float(estimated_state[12]),

                float(self.traj[0, self.step_counter]), float(self.traj[1, self.step_counter]), float(self.traj[2, self.step_counter]),
                float(self.traj[3, self.step_counter]), float(self.traj[4, self.step_counter]), float(self.traj[5, self.step_counter]), float(self.traj[6, self.step_counter]),
                # params (8)
                float(self.est_params[0]), float(self.est_params[1]), float(self.est_params[2]),
                float(self.est_params[3]), float(self.est_params[4]), float(self.est_params[5]),
                float(self.est_params[6]), float(self.est_params[7]),
                round(mpc_setup_time - start_time, 4),
                round(mpc_solve_time - mpc_setup_time, 4),
                round(send_command_and_visualisation - mpc_solve_time, 4),
                round(end_ukf_time - send_command_and_visualisation, 4),
            ]


            self.data_logger.append_row(log_row)

            # Step
            if self.takeoff_requested:
                self.step_counter += 1

            self.control_history.append(np.concatenate((u, u_rate)).tolist())
            self.observed_state_history.append(self.current_pose[:13].tolist())
            self.estimated_state_history.append(estimated_state[:13].tolist())
            self.UKF_state_estimation_history.append(self.x_est.tolist())

            saved_data = time.time()
            print(f"time taken to log {round(saved_data - end_ukf_time, 4)} seconds")

        else:
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cb.cmd_publisher_.publish(msg)
            self.step_counter = 0

    # --- UKF Functions ---
    def fx(self, x, u, u_rate):
        """
        Sigma-point propagation via the same CasADi integrator:
        - State vector: [p(3), q(4), v(3), w(3), params(8)]
        - Simulator state: [p, q, v, w, u]  (no params)
        - Simulator parameters: [dyn(8), q_ref(4)]
        """
        # unpack
        pos, quat, vel, ang_vel = x[:3], x[3:7], x[7:10], x[10:13]
        # params (8)
        thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo, angle_max_deg, tau_angle = x[13:21]
        state = np.concatenate((pos, quat, vel, ang_vel, u))
        param = np.array([thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo, angle_max_deg, tau_angle], dtype=float)

        # set into integrator
        self.sim_integrator.set("x", state)
        self.sim_integrator.set("u", u_rate)
        sim_p = np.concatenate([param, np.array([1.0, 0.0, 0.0, 0.0])])  # q_ref placeholder
        self.sim_integrator.set("p", sim_p)
        self.sim_integrator.solve()

        x_next = self.sim_integrator.get("x")
        # return next [p,q,v,w] plus unchanged params
        return np.concatenate((x_next[:13], param))

    def hx(self, x):
        return x[0:13]

    def generate_sigma_points(self, x, P, alpha, beta, kappa):
        n = len(x)
        lambda_ = alpha**2 * (n + kappa) - n
        sigma_points = [x]

        P_stable = P + np.eye(n) * 1e-9
        try:
            sqrt_P = cholesky((n + lambda_) * P_stable, lower=True)
        except np.linalg.LinAlgError:
            print("Warning: Covariance matrix not positive definite, using eigenvalue decomposition")
            eigenvals, eigenvecs = np.linalg.eigh((n + lambda_) * P_stable)
            eigenvals = np.maximum(eigenvals, 1e-9)
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
