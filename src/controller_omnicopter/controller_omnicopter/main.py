import rclpy
import signal
import sys
import numpy as np
import math
import csv
import os
from rclpy.node import Node
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import time
# NOTE: import the new helper we added to acados.py
from .acados import (
    generate_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_trajectory_reference_aligned,   # <-- NEW
    set_adaptive_parameters,            # <-- NEW for adaptive parameters
)
from .ukf_estimator import UKFEstimator  # <-- UKF for adaptive parameter estimation
from .gui import GUI
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory, hover_and_rotate, sine_wave_trajectory, hover_and_yaw, four_roll_rotations_trajectory, zsine_trajectory
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


def _norm_quat_np(q):
    n = float(np.linalg.norm(q))
    return q if n == 0.0 else (q / n)


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

        # self.timer_test_angular = self.create_timer(self.dt, self.angular_velocity_test)

        self.traj = circle_trajectory(self.dt)
        #self.traj = hover_and_yaw(self.dt)
        #self.traj = zsine_trajectory(self.dt)

        self.steps = self.traj.shape[1] - 1

        self.gui = GUI(self)
        self.armed = False

        self.pre_start_duration = 2.0
        self.pre_start_counter = 0
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 30
        self.skip_steps = 3
        # Parameters: [theta_roll, theta_pitch, theta_yaw, thrust_base, motor_time_constant, ixx, iyy, izz]
        # thrust_base = 7.42678162 (will be multiplied by 1e-7 in dynamics)
        # mass and motor_distance are now known constants in the dynamics model
        self.params = np.array([0.0, 0.0, 0.0, 7.42678162, 0.12])
        
        # Initialize UKF estimator for adaptive parameter estimation (always enabled)
        self.ukf = UKFEstimator(self.sim_integrator, self.dt, initial_params=self.params)
    
        # Set initial adaptive parameters for both solvers (only once)
        set_adaptive_parameters(self.ocp, self.sim_integrator, self.params, self.N)
        self.predicted_next_state = None
        self.last_pose = None
        self.last_control = None
        self.last_predicted_state = None  # Store predicted state for error calculation

        self.sent_command = False
        self.initial_guess_set = False
        self.last_actual_actuators = None  # Track last actual actuator states
        self.last_desired_actuators = None  # Track last desired actuator states
        self.sd = 0.2
        self.IG = 0.04

        # CSV init
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Create descriptive headers for x0 (29D current state)
            x0_headers = [
                'x0_px', 'x0_py', 'x0_pz',  # position (0-2)
                'x0_qw', 'x0_qx', 'x0_qy', 'x0_qz',  # quaternion (3-6)
                'x0_vx', 'x0_vy', 'x0_vz',  # linear velocity (7-9)
                'x0_wx', 'x0_wy', 'x0_wz',  # angular velocity (10-12)
                'x0_u0_act', 'x0_u1_act', 'x0_u2_act', 'x0_u3_act',  # actual actuators (13-16)
                'x0_u4_act', 'x0_u5_act', 'x0_u6_act', 'x0_u7_act',  # actual actuators (17-20)
                'x0_u0_des', 'x0_u1_des', 'x0_u2_des', 'x0_u3_des',  # desired actuators (21-24)
                'x0_u4_des', 'x0_u5_des', 'x0_u6_des', 'x0_u7_des'   # desired actuators (25-28)
            ]
            
            # Create descriptive headers for x_next (29D next state)  
            x_next_headers = [
                'x_next_px', 'x_next_py', 'x_next_pz',  # position (0-2)
                'x_next_qw', 'x_next_qx', 'x_next_qy', 'x_next_qz',  # quaternion (3-6)
                'x_next_vx', 'x_next_vy', 'x_next_vz',  # linear velocity (7-9)
                'x_next_wx', 'x_next_wy', 'x_next_wz',  # angular velocity (10-12)
                'x_next_u0_act', 'x_next_u1_act', 'x_next_u2_act', 'x_next_u3_act',  # actual actuators (13-16)
                'x_next_u4_act', 'x_next_u5_act', 'x_next_u6_act', 'x_next_u7_act',  # actual actuators (17-20)
                'x_next_u0_des', 'x_next_u1_des', 'x_next_u2_des', 'x_next_u3_des',  # desired actuators (21-24)
                'x_next_u4_des', 'x_next_u5_des', 'x_next_u6_des', 'x_next_u7_des'   # desired actuators (25-28)
            ]
            
            # Create headers for u_dot_rates (control rates)
            u_dot_headers = [
                'u_dot_0', 'u_dot_1', 'u_dot_2', 'u_dot_3',
                'u_dot_4', 'u_dot_5', 'u_dot_6', 'u_dot_7'
            ]
            
            # Create headers for prediction errors
            error_headers = [
                'pos_error', 'quat_error', 'vel_error', 'angvel_error', 
                'act_error', 'des_act_error'
            ]
            
            self.csv_writer.writerow([
                'Step'
            ] + x0_headers + u_dot_headers + x_next_headers + error_headers)

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)
        q = _norm_quat_np(q)  # keep unit quaternion
        self.current_pose = np.array([
            p.x, p.y, p.z, q[0], q[1], q[2], q[3], lv.x, lv.y, lv.z, av.x, av.y, av.z
        ])

    def expand_state_to_29d(self, state_13d, actual_actuators=None, desired_actuators=None):
        if actual_actuators is None:
            actual_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        if desired_actuators is None:
            desired_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        return np.concatenate([state_13d, actual_actuators, desired_actuators])

    def get_current_state_29d(self):
        if hasattr(self, 'last_actual_actuators') and self.last_actual_actuators is not None:
            actual_actuators = self.last_actual_actuators
        else:
            actual_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
            
        if hasattr(self, 'last_desired_actuators') and self.last_desired_actuators is not None:
            desired_actuators = self.last_desired_actuators
        else:
            desired_actuators = np.array([-self.IG, self.IG, -self.IG, self.IG, self.IG, -self.IG, self.IG, -self.IG])
        
        return self.expand_state_to_29d(self.current_pose, actual_actuators, desired_actuators)

    def control_loop(self):

        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")

            msg = ELRSCommand(armed=True, channel_0=-self.sd, channel_1=self.sd, channel_2=-self.sd, channel_3=self.sd, channel_4=self.sd, channel_5=-self.sd, channel_6=self.sd, channel_7=-self.sd)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:

            if self.step_counter + self.N * self.skip_steps > self.steps:
                self.step_counter = 0
                self.armed = False
                msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=0.0, channel_3=0.0, channel_4=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0)
                self.cmd_publisher_.publish(msg)
                self.on_close()

            # ---------- Build time-varying state references for the horizon ----------
            # X_ref shape: (N+1, 29). Row j is the state ref at stage j. Row N is terminal.
            X_ref = np.zeros((self.N + 1, 29), dtype=float)
            for j in range(self.N):
                sc = self.step_counter + j * self.skip_steps
                traj_13d = self.traj[:, sc]  # Get 13D trajectory point
                # Expand to 29D with hover actuator values as reference
                X_ref[j, :] = self.expand_state_to_29d(traj_13d)
            sn = self.step_counter + self.N * self.skip_steps
            traj_13d_terminal = self.traj[:, sn]
            X_ref[self.N, :] = self.expand_state_to_29d(traj_13d_terminal)


            set_trajectory_reference_aligned(self.ocp, X_ref)

            x0 = self.get_current_state_29d()  # Get 29D current state
            x0[3:7] = _norm_quat_np(x0[3:7])  # ensure unit quaternion
            
            # UKF Adaptive Parameter Estimation (always enabled)
            if self.step_counter > 0:  # Skip first step for initialization
                # Use the control from the previous step for UKF prediction
                prev_u_dot = self.last_control if self.last_control is not None else np.zeros(8)
                
                # Update actuator states in UKF before prediction
                #if hasattr(self, 'last_actual_actuators') and self.last_actual_actuators is not None:
                #    self.ukf.update_actuator_states(self.last_actual_actuators, self.last_desired_actuators)
                
                # Update UKF with current measurement and previous control
                #estimated_params = self.ukf.predict_and_update(self.current_pose, prev_u_dot)
                
                # Update parameters used by MPC solver
                #self.params = estimated_params.copy()
                #set_adaptive_parameters(self.ocp, self.sim_integrator, self.params, self.N)
                
                #print(f"UKF estimated params: {[round(val, 4) for val in estimated_params]}")
            elif self.step_counter == 0:
                # Initialize UKF with current state on first step
                # Provide actuator states if available, otherwise use defaults
                if hasattr(self, 'last_actual_actuators') and self.last_actual_actuators is not None:
                    actuator_states = np.concatenate([self.last_actual_actuators, self.last_desired_actuators])
                    self.ukf.set_initial_state(self.current_pose, actuator_states)
                else:
                    self.ukf.set_initial_state(self.current_pose)
            
            # The key fix: Set both lower and upper bounds to the current state
            # This constrains the first shooting node to the current measured/estimated state
            self.ocp.set(0, "lbx", x0 - 0.025*x0)
            self.ocp.set(0, "ubx", x0 + 0.025*x0)

            if not self.initial_guess_set:
                set_initial_guess(self.ocp, self.N)
                self.initial_guess_set = True
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status} after retry.')

            u_dot_rates = self.ocp.get(0, "u")  # These are now rates of desired actuators (d(u_desired)/dt)
            
            # Store control for next UKF iteration
            self.last_control = u_dot_rates.copy()
            
            # Get the states from the optimized solution
            x_next = self.ocp.get(1, "x")  # Next optimized state 
            actual_actuators = x_next[13:21].copy()  # Extract actual actuator states (force values)
            desired_actuators = x_next[21:29].copy()  # Extract desired actuator states (force values)
            
            # Store for next iteration
            self.last_actual_actuators = actual_actuators
            self.last_desired_actuators = desired_actuators


            motor_speeds = np.sqrt(np.abs(desired_actuators))
            # Preserve sign of original actuator values
            motor_speeds = np.sign(desired_actuators) * motor_speeds
            
            # Clamp motor speeds to [-1, 1] range for safety
            motor_speeds = np.clip(motor_speeds, -1.0, 1.0)

            # Send the converted motor speeds to the motors
            msg = ELRSCommand(
                armed=True,
                channel_0=round(motor_speeds[0], 3),
                channel_1=round(motor_speeds[1], 3),
                channel_2=round(motor_speeds[2], 3),
                channel_3=round(motor_speeds[3], 3),
                channel_4=round(motor_speeds[4], 3),
                channel_5=round(motor_speeds[5], 3),
                channel_6=round(motor_speeds[6], 3),
                channel_7=round(motor_speeds[7], 3)
            )
            self.cmd_publisher_.publish(msg)
            self.step_counter += 1

            print(f"Step {self.step_counter}, Control (des_act): {[round(val, 4) for val in desired_actuators]}")
            print(f"Step {self.step_counter}, Control (mot_spd): {[round(val, 4) for val in motor_speeds]}")

            # Calculate prediction error from previous timestep (if available)
            prediction_error = np.zeros(29)  # Initialize with zeros
            position_error = quaternion_error = velocity_error = 0.0
            angular_vel_error = actuator_error = desired_actuator_error = 0.0
            
            if self.last_predicted_state is not None:
                # Compare last predicted state with current actual observed state
                prediction_error = x0 - self.last_predicted_state
                
                # Calculate norms for different state components
                position_error = np.linalg.norm(prediction_error[0:3])
                quaternion_error = np.linalg.norm(prediction_error[3:7])
                velocity_error = np.linalg.norm(prediction_error[7:10])
                angular_vel_error = np.linalg.norm(prediction_error[10:13])
                actuator_error = np.linalg.norm(prediction_error[13:21])
                desired_actuator_error = np.linalg.norm(prediction_error[21:29])


                print(x0[7:10])
                print(self.last_predicted_state[7:10])
                print(prediction_error[7:10])

            # Run simulation integrator to predict next state for comparison in next iteration
            self.sim_integrator.set("x", x0) 
            self.sim_integrator.set("u", u_dot_rates) 
            self.sim_integrator.set("p", self.params)
            status_sim = self.sim_integrator.solve()
            self.last_predicted_state = self.sim_integrator.get("x")  # Store for next iteration comparison

            # Save data: step_counter, x0 (29D), u_dot_rates (8D), x_next (29D), errors (6) as separate columns
            error_data = [position_error, quaternion_error, velocity_error, 
                         angular_vel_error, actuator_error, desired_actuator_error]
            row_data = [self.step_counter] + list(x0) + list(u_dot_rates) + list(x_next) + error_data
            self.csv_writer.writerow(row_data)





        else:
            print("Controller is not armed or current pose is not available.")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=0.0, channel_3=0.0, channel_4=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0











    def angular_velocity_test(self):

        if self.armed and self.sent_command == False:
            print("Sending initial command to arm the controller.")
            print(f"Current pose: {self.current_pose[10:13]}")
            u = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            # Use 29D state for simulation
            x_29d = self.get_current_state_29d()
            self.sim_integrator.set("x", x_29d)
            self.sim_integrator.set("u", u)
            self.sim_integrator.solve()
            predicted_state = self.sim_integrator.get("x")
            print("Predicted state:", predicted_state[10:13])

            msg = ELRSCommand(armed=True, channel_0=u[0], channel_1=u[1], channel_2=u[2], channel_3=u[3], channel_4=u[4], channel_5=u[5], channel_6=u[6], channel_7=u[7])
            self.cmd_publisher_.publish(msg)
            self.sent_command = True

        elif self.armed and self.sent_command == True:
            print(f"Current pose: {self.current_pose[10:13]}")
        else:
            u = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            msg = ELRSCommand(armed=True, channel_0=u[0], channel_1=u[1], channel_2=u[2], channel_3=u[3], channel_4=u[4], channel_5=u[5], channel_6=u[6], channel_7=u[7])
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
