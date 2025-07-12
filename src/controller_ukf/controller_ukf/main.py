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
from .trajectories import hover_trajectory
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray
from scipy.linalg import cholesky


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

        self.traj = hover_trajectory(self.dt)   

        self.steps = self.traj.shape[1] - 1  # Number of steps in the trajectory

        self.gui = GUI(self)
        self.armed = False
        
        self.pre_start_duration = 2.0
        self.pre_start_counter = 0 
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 20
        self.skip_steps = 3

            


        self.logger_csv_file = open('logger.csv', mode='w', newline='')
        self.logger_csv_writer = csv.writer(self.logger_csv_file)

        self.logger_csv_writer.writerow([
                'm_px', 'm_py', 'm_pz', 'm_rw', 'm_rx', 'm_ry', 'm_rz',
                'm_vx', 'm_vy', 'm_vz', 'm_wx', 'm_wy', 'm_wz',
                'o_px', 'o_py', 'o_pz', 'o_rw', 'o_rx', 'o_ry', 'o_rz',
                'o_vx', 'o_vy', 'o_vz', 'o_wx', 'o_wy', 'o_wz',
        ])



        self.control_history = [0.0,0.0,0.0,0.0]

        self.est_params = np.array([20.0])  # Initialize thrust ratio parameter to a reasonable value

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
                          1.0])  # Thrust ratio parameter uncertainty
        self.Q = np.diag([1e-4, 1e-4, 1e-4,  # Position process noise
                          1e-5, 1e-5, 1e-5, 1e-5,  # Quaternion process noise
                          1e-3, 1e-3, 1e-3,  # Velocity process noise
                          1e-3, 1e-3, 1e-3,  # Angular rates process noise
                          1e-2])  # Thrust ratio process noise
        self.R = np.diag([0.1]*13)  # Measurement noise for all 14 state elements

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.motion_capture_pose = np.round(np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
        ]), 3)


    def orb_slam_state_callback(self, msg: MotionCaptureState):
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
            estimated_state_with_control = np.concatenate((self.current_pose, self.control_history))
            self.ocp.set(0, "lbx", estimated_state_with_control)
            self.ocp.set(0, "ubx", estimated_state_with_control)


            ### SOLVE OCP
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')
            x = self.ocp.get(1, "x")
            u = x[-4:]  # Extract the last 4 elements as control inputs
            self.control_history = u
            u_rate = self.ocp.get(0, "u")





            # UKF predict
            sigma_pts, wm, wc = self.generate_sigma_points(self.x_est, self.P, self.alpha, self.beta, self.kappa)
            sigma_pts_pred = np.array([
                self.fx(pt, u, u_rate) for pt in sigma_pts
            ])
            x_pred, P_pred = self.unscented_transform(sigma_pts_pred, wm, wc, self.Q)

            # UKF update
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


            print(f"x_est_position: {self.x_est[:3]}")
            print(f"actual_position: {self.current_pose[:3]}")



            # Update estimated parameters
            self.est_params = np.array([self.x_est[13]])

            print(f"Estimated thrust ratio: {self.est_params}")


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
