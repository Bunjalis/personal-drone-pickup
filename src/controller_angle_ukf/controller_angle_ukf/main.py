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
        # UKF settings (UPDATED - Option 2: Partial measurement with yaw only)
        # ---------------------------
        # Fixed constants
        self.angle_max_deg = 55.0
        self.centre_rate_deg = 100.0   # Fixed BF rates curve params (for yaw)
        self.max_rate_deg = 100.0
        self.rate_expo = 0.5
        self.tau_angle = 0.08 # Fixed angle loop time constant
        self.tau_rate = 0.08   # Fixed yaw rate loop time constant
        
        # est_params order (4):
        # [kT, dragZ, fc_roll_offset_deg, fc_pitch_offset_deg]
        self.est_params = np.array([42.0, 0.1, 0.0, 0.0], dtype=float)

        self.alpha, self.beta, self.kappa = 0.1, 2, 0

        # State x_est: [p(3), q(4), v(3), w(3), params(4)] = 17
        self.x_est = np.array([
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            *self.est_params  # 4 params
        ], dtype=float)


        # Covariances sized to 17x17
        self.P = np.diag([
            0.1, 0.1, 0.1,              # position (observable)
            0.1, 0.1, 0.1, 0.1,         # quaternion (pitch/roll less observable, yaw more observable)
            0.1, 0.1, 0.1,              # velocity (observable)
            0.1, 0.1, 0.1,              # angular velocity (observable)
            0.1, 0.001,                 # kT, dragZ
            0.1, 0.1                    # fc_roll_offset_deg, fc_pitch_offset_deg (observable through position drift)
        ]).astype(float)   

        self.Q = np.diag([
            # process noise for states
            1e-4, 1e-4, 1e-4,           # p
            1e-4, 1e-4, 1e-4, 1e-5,     # q (increased for pitch/roll since less observable)
            1e-3, 1e-3, 1e-3,           # v
            1e-3, 1e-3, 1e-3,           # w
            # params (slower drift)
            1e-3, 1e-5,                 # kT, dragZ
            1e-5, 1e-5                  # fc_roll_offset_deg, fc_pitch_offset_deg (can change slowly)
        ]).astype(float)

        # Measurement: 10 (p(3), yaw(1), v(3), w(3))
        self.R = np.diag([0.05]*10).astype(float)

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


            'ukf_pose_x', 'ukf_pose_y', 'ukf_pose_z', 'ukf_pose_qw', 'ukf_pose_qx', 'ukf_pose_qy', 'ukf_pose_qz',
            'ukf_pose_vx', 'ukf_pose_vy', 'ukf_pose_vz', 'ukf_pose_avx', 'ukf_pose_avy', 'ukf_pose_avz',


            'traj_x_ref', 'traj_y_ref', 'traj_z_ref', 'traj_qw_ref', 'traj_qx_ref', 'traj_qy_ref', 'traj_qz_ref',
            'est_param_thrust_ratio', 'est_param_drag_coeff_z', 'est_param_tau_rate',
            'fixed_centre_rate_deg', 'fixed_max_rate_deg', 'fixed_rate_expo',  # Now fixed, not estimated
            'fixed_angle_max_deg', 'est_param_tau_angle',
            'est_param_fc_roll_offset_deg', 'est_param_fc_pitch_offset_deg',  # NEW: FC mounting offsets
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
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, 0.0, 0.0, av.z
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

            # Update OCP parameters with current estimates (4 params + 6 fixed)
            # Combine estimated params with fixed values
            # Full params: [kT, dragZ, tau_rate, centre, max, expo, angle_max, tau_angle, fc_roll, fc_pitch]
            full_params = np.concatenate([
                self.est_params[:2],  # kT, dragZ
                [self.tau_rate],       # fixed tau_rate
                [self.centre_rate_deg, self.max_rate_deg, self.rate_expo],  # fixed rates
                [self.angle_max_deg],  # fixed angle_max
                [self.tau_angle],      # fixed tau_angle
                self.est_params[2:4]   # fc_roll_offset, fc_pitch_offset
            ])
            update_ocp_parameters(self.ocp, full_params, self.N)
            set_trajectory_reference_aligned(
                self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, full_params
            )

            # ---- Delay-compensated state roll-forward using sim_integrator ----
            # Hybrid quaternion: Use UKF's pitch/roll + measured yaw
            estimated_state = copy.deepcopy(self.current_pose[:13])
            estimated_state[3:7] = self.x_est[3:7]  # start with UKF quaternion
            
            

            if len(self.control_history) <= 0 or self.delay_states == 0:
                delayed_control_history = []
            else:
                delayed_control_history = self.control_history[-self.delay_states:]

            for i, val in enumerate(delayed_control_history):
                # integrator state = [p(3), q(4), v(3), w(3), u(4)]
                self.sim_integrator.set("x", np.concatenate((estimated_state, np.array(val[0:4]).flatten())))
                self.sim_integrator.set("u", np.array(val[4:8]))
                # params = 10 dyn + 4 qref (qref unused in dynamics)
                # Combine estimated params (4) with fixed values (6)
                full_params = np.concatenate([
                    self.est_params[:2],  # kT, dragZ
                    [self.tau_rate],       # fixed tau_rate
                    [self.centre_rate_deg, self.max_rate_deg, self.rate_expo],  # fixed rates
                    [self.angle_max_deg],  # fixed angle_max
                    [self.tau_angle],      # fixed tau_angle
                    self.est_params[2:4]   # fc_roll_offset, fc_pitch_offset
                ])
                sim_p = np.concatenate([full_params, np.array([1.0, 0.0, 0.0, 0.0])])
                self.sim_integrator.set("p", sim_p)
                status_sim = self.sim_integrator.solve()
                if status_sim != 0:
                    raise Exception(f"Simulation integrator failed with status {status_sim}.")
                x_next = self.sim_integrator.get("x")
                estimated_state = x_next[:13]



            # Extract yaw from current_pose (measured)
            qw_meas, qx_meas, qy_meas, qz_meas = estimated_state[3:7]
            yaw_measured = np.arctan2(2*(qw_meas*qz_meas + qx_meas*qy_meas), 1 - 2*(qy_meas**2 + qz_meas**2))
            
            # Extract pitch/roll from UKF estimate (model-based)
            qw_ukf, qx_ukf, qy_ukf, qz_ukf = self.x_est[3:7]
            # Convert UKF quaternion to Euler
            roll_ukf = np.arctan2(2*(qw_ukf*qx_ukf + qy_ukf*qz_ukf), 1 - 2*(qx_ukf**2 + qy_ukf**2))
            pitch_ukf = np.arcsin(np.clip(2*(qw_ukf*qy_ukf - qz_ukf*qx_ukf), -1.0, 1.0))
            
            # Reconstruct quaternion from UKF's roll/pitch + measured yaw
            cy = np.cos(yaw_measured * 0.5)
            sy = np.sin(yaw_measured * 0.5)
            cp = np.cos(pitch_ukf * 0.5)
            sp = np.sin(pitch_ukf * 0.5)
            cr = np.cos(roll_ukf * 0.5)
            sr = np.sin(roll_ukf * 0.5)
            
            hybrid_qw = cr * cp * cy + sr * sp * sy
            hybrid_qx = sr * cp * cy - cr * sp * sy
            hybrid_qy = cr * sp * cy + sr * cp * sy
            hybrid_qz = cr * cp * sy - sr * sp * cy
            
            # Normalize
            quat_norm = np.sqrt(hybrid_qw**2 + hybrid_qx**2 + hybrid_qy**2 + hybrid_qz**2)
            estimated_state[3:7] = np.array([hybrid_qw, hybrid_qx, hybrid_qy, hybrid_qz]) / quat_norm


            # Tight initial-state box (softened)
            if len(self.control_history) == 0:
                self.get_logger().warn("Control history is empty - cannot set state bounds accurately")
                estimated_state_with_control = np.concatenate((estimated_state, np.array([0.0, 0.0, 0.0, 0.0])))
            else:
                estimated_state_with_control = np.concatenate((estimated_state, np.array(self.control_history[-1][0:4])))


            init_mpc_state = estimated_state_with_control.copy()
            relaxation_factor = 0.00001
            lbx = init_mpc_state - relaxation_factor * init_mpc_state
            ubx = init_mpc_state + relaxation_factor * init_mpc_state
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

                # Extract measurement: [p(3), yaw(1), v(3), w(3)] from ORB-SLAM
                # Extract yaw from ORB-SLAM quaternion
                qw, qx, qy, qz = self.current_pose[3:7]
                yaw_measured = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
                z_measured = np.concatenate([
                    self.current_pose[0:3],    # position
                    [yaw_measured],             # yaw only
                    self.current_pose[7:13]    # velocity + angular velocity
                ])

                K = P_xz @ np.linalg.inv(P_zz)
                self.x_est = x_pred + K @ (z_measured - z_pred)
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

                # -------- Parameter clamping (4 params) --------
                # kT
                self.x_est[13] = np.clip(self.x_est[13], 18.0, 60.0)
                # dragZ
                self.x_est[14] = np.clip(self.x_est[14], 0.01, 0.5)
                # fc_roll_offset_deg (index 15)
                self.x_est[15] = np.clip(self.x_est[15], -5.0, 5.0)
                # fc_pitch_offset_deg (index 16)
                self.x_est[16] = np.clip(self.x_est[16], -5.0, 5.0)

                # Update estimated parameters vector (4)
                self.est_params = np.array([
                    self.x_est[13], self.x_est[14],
                    self.x_est[15], self.x_est[16]
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



                float(self.x_est[0]), float(self.x_est[1]), float(self.x_est[2]),
                float(self.x_est[3]), float(self.x_est[4]), float(self.x_est[5]), float(self.x_est[6]),
                float(self.x_est[7]), float(self.x_est[8]), float(self.x_est[9]),
                float(self.x_est[10]), float(self.x_est[11]), float(self.x_est[12]),


                float(self.traj[0, self.step_counter]), float(self.traj[1, self.step_counter]), float(self.traj[2, self.step_counter]),
                float(self.traj[3, self.step_counter]), float(self.traj[4, self.step_counter]), float(self.traj[5, self.step_counter]), float(self.traj[6, self.step_counter]),
                # params (4 estimated + 6 fixed)
                float(self.est_params[0]), float(self.est_params[1]),  # kT, dragZ
                float(self.tau_rate),  # fixed tau_rate
                float(self.centre_rate_deg), float(self.max_rate_deg), float(self.rate_expo),    # fixed rates
                float(self.angle_max_deg), float(self.tau_angle),  # fixed angle_max, fixed tau_angle
                float(self.est_params[2]), float(self.est_params[3]),  # fc_roll_offset, fc_pitch_offset
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
        - State vector: [p(3), q(4), v(3), w(3), params(4)]
        - Simulator state: [p, q, v, w, u]  (no params)
        - Simulator parameters: [dyn(10), q_ref(4)]
        """
        # unpack
        pos, quat, vel, ang_vel = x[:3], x[3:7], x[7:10], x[10:13]
        # params (4 estimated)
        thrust_ratio, drag_coeff_z, fc_roll_offset_deg, fc_pitch_offset_deg = x[13:17]
        state = np.concatenate((pos, quat, vel, ang_vel, u))
        # Reconstruct full 10-param vector with fixed values
        param = np.array([
            thrust_ratio, drag_coeff_z,
            self.tau_rate,       # fixed
            self.centre_rate_deg, self.max_rate_deg, self.rate_expo,  # fixed
            self.angle_max_deg,  # fixed
            self.tau_angle,      # fixed
            fc_roll_offset_deg, fc_pitch_offset_deg
        ], dtype=float)

        # set into integrator
        self.sim_integrator.set("x", state)
        self.sim_integrator.set("u", u_rate)
        sim_p = np.concatenate([param, np.array([1.0, 0.0, 0.0, 0.0])])  # q_ref placeholder
        self.sim_integrator.set("p", sim_p)
        self.sim_integrator.solve()

        x_next = self.sim_integrator.get("x")
        # return next [p,q,v,w] plus unchanged params (4)
        return np.concatenate((x_next[:13], x[13:17]))

    def hx(self, x):
        """
        Measurement model: Extract [p(3), yaw(1), v(3), w(3)] = 10 measurements
        Ignores pitch and roll from quaternion.
        """
        # Extract yaw from quaternion
        qw, qx, qy, qz = x[3], x[4], x[5], x[6]
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        
        return np.concatenate([
            x[0:3],      # position
            [yaw],       # yaw only
            x[7:10],     # velocity
            x[10:13]     # angular velocity
        ])

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
