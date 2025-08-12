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
# NOTE: import the new helper we added to acados.py
from .acados import (
    generate_ocp_controller,
    set_initial_guess,
    warm_start_from_previous_solution,
    set_trajectory_reference_aligned,   # <-- NEW
)
from .gui import GUI
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory, hover_and_rotate, sine_wave_trajectory, hover_and_yaw, four_roll_rotations_trajectory
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

        #self.traj = circle_trajectory(self.dt)
        #self.traj = hover_and_yaw(self.dt)
        #self.traj = hover_and_rotate(self.dt)
        self.traj = hover_trajectory(self.dt)
        #self.traj = sine_wave_trajectory(self.dt)
        #self.traj = four_roll_rotations_trajectory(self.dt)

        self.steps = self.traj.shape[1] - 1

        self.gui = GUI(self)
        self.armed = False

        self.pre_start_duration = 2.0
        self.pre_start_counter = 0
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 30
        self.skip_steps = 3
        self.predicted_next_state = None
        self.last_pose = None
        self.last_control = None

        self.sent_command = False
        self.initial_guess_set = False
        self.last_actual_actuators = None  # Track last actual actuator states
        self.last_desired_actuators = None  # Track last desired actuator states
        self.sd = 0.25

        # CSV init
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'Step',
                'px', 'py', 'pz', 'rw', 'rx', 'ry', 'rz',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz',
                'u0_act', 'u1_act', 'u2_act', 'u3_act', 'u4_act', 'u5_act', 'u6_act', 'u7_act', 
                'u0_des', 'u1_des', 'u2_des', 'u3_des', 'u4_des', 'u5_des', 'u6_des', 'u7_des', 
                'u0_dot', 'u1_dot', 'u2_dot', 'u3_dot', 'u4_dot', 'u5_dot', 'u6_dot', 'u7_dot', 
            ])

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)
        q = _norm_quat_np(q)  # keep unit quaternion
        self.current_pose = np.array([
            p.x, p.y, p.z, q[0], q[1], q[2], q[3], lv.x, lv.y, lv.z, av.x, av.y, av.z
        ])

    def expand_state_to_29d(self, state_13d, actual_actuators=None, desired_actuators=None):
        """
        Expand a 13-dimensional state to 29 dimensions by adding actual and desired actuator states.
        
        Args:
            state_13d: (13,) array with [p, q, v, r]
            actual_actuators: (8,) array with actual actuator states. If None, uses hover values.
            desired_actuators: (8,) array with desired actuator states. If None, uses hover values.
        
        Returns:
            state_29d: (29,) array with [p, q, v, r, actual_actuators, desired_actuators]
        """
        if actual_actuators is None:
            # Start with hover actuator values
            actual_actuators = np.array([-self.sd, self.sd, -self.sd, self.sd, self.sd, -self.sd, self.sd, -self.sd])
        if desired_actuators is None:
            # Start with hover actuator values
            desired_actuators = np.array([-self.sd, self.sd, -self.sd, self.sd, self.sd, -self.sd, self.sd, -self.sd])
        return np.concatenate([state_13d, actual_actuators, desired_actuators])

    def get_current_state_29d(self):
        """
        Get current state expanded to 29 dimensions. 
        For the actuator states, we'll try to get them from the solver if available,
        otherwise use the last known values or hover values.
        """
        if hasattr(self, 'last_actual_actuators') and self.last_actual_actuators is not None:
            actual_actuators = self.last_actual_actuators
        else:
            # Start with hover actual actuator values
            actual_actuators = np.array([-self.sd, self.sd, -self.sd, self.sd, self.sd, -self.sd, self.sd, -self.sd])
            
        if hasattr(self, 'last_desired_actuators') and self.last_desired_actuators is not None:
            desired_actuators = self.last_desired_actuators
        else:
            # Start with hover desired actuator values
            desired_actuators = np.array([-self.sd, self.sd, -self.sd, self.sd, self.sd, -self.sd, self.sd, -self.sd])
        
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
            
            # The key fix: Set both lower and upper bounds to the current state
            # This constrains the first shooting node to the current measured/estimated state
            self.ocp.set(0, "lbx", x0)
            self.ocp.set(0, "ubx", x0)

            if not self.initial_guess_set:
                set_initial_guess(self.ocp, self.N)
                self.initial_guess_set = True
            else:
                warm_start_from_previous_solution(self.ocp, self.N)

            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status} after retry.')

            u_dot_rates = self.ocp.get(0, "u")  # These are now rates of desired actuators (d(u_desired)/dt)
            
            # Get the states from the optimized solution
            x_current = self.ocp.get(1, "x")  # Next optimized state 
            actual_actuators = x_current[13:21].copy()  # Extract actual actuator states
            desired_actuators = x_current[21:29].copy()  # Extract desired actuator states
            
            # Store for next iteration
            self.last_actual_actuators = actual_actuators
            self.last_desired_actuators = desired_actuators

            print(f"Control rates (u_dot): {u_dot_rates}")
            print(f"Desired actuators: {desired_actuators}")
            print(f"Actual actuators: {actual_actuators}")

            # Send the ACTUAL actuator values (not desired) to the motors
            # The actual actuators will lag behind the desired ones due to first-order dynamics
            msg = ELRSCommand(
                armed=True,
                channel_0=round(desired_actuators[0], 8),
                channel_1=round(desired_actuators[1], 8),
                channel_2=round(desired_actuators[2], 8),
                channel_3=round(desired_actuators[3], 8),
                channel_4=round(desired_actuators[4], 8),
                channel_5=round(desired_actuators[5], 8),
                channel_6=round(desired_actuators[6], 8),
                channel_7=round(desired_actuators[7], 8)
            )
            self.cmd_publisher_.publish(msg)
            self.step_counter += 1


            self.csv_writer.writerow([
                self.step_counter,
                x_current[0], x_current[1], x_current[2], x_current[3], x_current[4], x_current[5], x_current[6],
                x_current[7], x_current[8], x_current[9], x_current[10], x_current[11], x_current[12],
                actual_actuators[0], actual_actuators[1], actual_actuators[2], actual_actuators[3],
                actual_actuators[4], actual_actuators[5], actual_actuators[6], actual_actuators[7],
                desired_actuators[0], desired_actuators[1], desired_actuators[2], desired_actuators[3],
                desired_actuators[4], desired_actuators[5], desired_actuators[6], desired_actuators[7],
                u_dot_rates[0], u_dot_rates[1], u_dot_rates[2], u_dot_rates[3],
                u_dot_rates[4], u_dot_rates[5], u_dot_rates[6], u_dot_rates[7]
            ])





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
