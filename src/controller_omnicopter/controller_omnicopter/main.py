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
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory, hover_and_rotate, sine_wave_trajectory, hover_and_yaw
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

        # self.traj = circle_trajectory(self.dt)
        self.traj = hover_and_yaw(self.dt)
        # self.traj = hover_and_rotate(self.dt)

        self.steps = self.traj.shape[1] - 1

        self.gui = GUI(self)
        self.armed = False

        self.pre_start_duration = 2.0
        self.pre_start_counter = 0
        self.pre_start_steps = int(self.pre_start_duration / self.dt)

        self.N = 20
        self.skip_steps = 3
        self.predicted_next_state = None
        self.last_pose = None
        self.last_control = None

        self.sent_command = False
        self.initial_guess_set = False

        # CSV init
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'Step', 'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7',   # <- fixed header label
                'px', 'py', 'pz', 'rw', 'rx', 'ry', 'rz',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz',
                'sp_px', 'sp_py', 'sp_pz', 'sp_rw', 'sp_rx', 'sp_ry', 'sp_rz',
                'sp_vx', 'sp_vy', 'sp_vz', 'sp_wx', 'sp_wy', 'sp_wz',
            ])

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        q = np.array([o.w, o.x, o.y, o.z], dtype=float)
        q = _norm_quat_np(q)  # keep unit quaternion
        self.current_pose = np.array([
            p.x, p.y, p.z, q[0], q[1], q[2], q[3], lv.x, lv.y, lv.z, av.x, av.y, av.z
        ])

    def control_loop(self):

        if self.armed and self.pre_start_counter < self.pre_start_steps:
            print(f"Pre-start phase: {self.pre_start_counter + 1}/{self.pre_start_steps}")
            sd = 0.2
            msg = ELRSCommand(armed=True, channel_0=-sd, channel_1=sd, channel_2=-sd, channel_3=sd, channel_4=sd, channel_5=-sd, channel_6=sd, channel_7=-sd)
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
            # X_ref shape: (N+1, 13). Row j is the state ref at stage j. Row N is terminal.
            X_ref = np.zeros((self.N + 1, 13), dtype=float)
            for j in range(self.N):
                sc = self.step_counter + j * self.skip_steps
                X_ref[j, :] = self.traj[:, sc]
            sn = self.step_counter + self.N * self.skip_steps
            X_ref[self.N, :] = self.traj[:, sn]


            set_trajectory_reference_aligned(self.ocp, X_ref)

            x0 = self.current_pose.copy()
            x0[3:7] = _norm_quat_np(x0[3:7])  # ensure unit quaternion
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

            u = self.ocp.get(0, "u")

            print(u)



            msg = ELRSCommand(
                armed=True,
                channel_0=round(u[0], 3),
                channel_1=round(u[1], 3),
                channel_2=round(u[2], 3),
                channel_3=round(u[3], 3),
                channel_4=round(u[4], 3),
                channel_5=round(u[5], 3),
                channel_6=round(u[6], 3),
                channel_7=round(u[7], 3)
            )
            self.cmd_publisher_.publish(msg)
            self.step_counter += 1

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

            self.sim_integrator.set("x", self.current_pose)
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
