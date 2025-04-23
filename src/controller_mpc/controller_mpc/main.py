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

        self.ocp = generate_ocp_controller()

        time_space = np.linspace(0, self.steps * self.dt, self.steps)
        
        # Alternate between [0, 0, 2] and [1, 1, 2] every 10 seconds
        self.x_traj = np.where((time_space // 10) % 2 == 0, 0.0, 1.0)
        self.y_traj = np.where((time_space // 10) % 2 == 0, 0.0, 1.0)
        self.z_traj = 2.0 * np.ones_like(time_space)

        # Define yaw trajectory (90 degrees to the left, which is -π/2 radians)
        yaw_traj = np.pi / 2 * np.zeros_like(time_space)  # Yaw remains constant at -π/2 radians
        roll_traj = np.zeros_like(time_space)  # Roll remains 0
        pitch_traj = np.zeros_like(time_space)  # Pitch remains 0

        # Convert RPY to quaternions
        rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
        quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Converts to [q_x, q_y, q_z, q_w]

        self.qx_traj = quaternions[:, 0]
        self.qy_traj = quaternions[:, 1]
        self.qz_traj = quaternions[:, 2]
        self.qw_traj = quaternions[:, 3]

        print(f"w {self.qw_traj[0]}, x {self.qx_traj[0]}, y {self.qy_traj[0]}, z {self.qz_traj[0]}")

        self.gui = GUI(self)
        self.armed = False
        self.executing_actions = False
        self.executed_steps = 0
        self.saved_states = []
        self.saved_controls = []
        self.recorded_states = []  # To store the recorded states
        self.initial_solve_state = None
        self.initial_solve_controls = None

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

        if self.executing_actions:
            # Execute the saved actions for the next 60 timesteps
            if self.executed_steps < 30:
                u_command = self.saved_controls[self.executed_steps]
                msg.armed = True
                msg.channel_0 = u_command[0]
                msg.channel_1 = u_command[1]
                msg.channel_2 = u_command[2]
                msg.channel_3 = u_command[3]

                # Record the current state while taking the action
                self.recorded_states.append(self.current_pose)

                self.executed_steps += 1
            else:
                # Disarm after 60 timesteps
                self.executing_actions = False
                self.armed = False
                self.executed_steps = 0
                print("Disarmed after executing actions.")
                self.save_to_csv()
        elif self.armed and self.current_pose is not None:
            # Solve the OCP and save the trajectory
            for j in range(60):
                if self.step_counter + j < self.steps:
                    yref = np.array([self.x_traj[self.step_counter + j], self.y_traj[self.step_counter + j],
                                     self.z_traj[self.step_counter + j], self.qw_traj[self.step_counter + j],
                                     self.qx_traj[self.step_counter + j], self.qy_traj[self.step_counter + j],
                                     self.qz_traj[self.step_counter + j], 0, 0, 0, 0, 0, 0, 0.6, 0.6, 0.6, 0.6])
                else:
                    yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0,0, 0, 0,0, 0, 0.6, 0.6, 0.6, 0.6])
                self.ocp.set(j, "yref", yref)
            self.ocp.set(0, "lbx", self.current_pose)
            self.ocp.set(0, "ubx", self.current_pose)

            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')

            # Save the solved trajectory
            self.saved_states = [self.ocp.get(i, "x") for i in range(self.ocp.N + 1)]
            self.saved_controls = [self.ocp.get(i, "u") for i in range(self.ocp.N)]
            # Start executing the saved actions
            self.step_counter += 1


            if self.step_counter >= 60:
                self.executing_actions = True
        else:
            self.step_counter = 0

        self.cmd_publisher_.publish(msg)

    def save_to_csv(self):
        # Save the initial solve state, actions, and incoming state information to a CSV file
        with open("trajectory_data.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Step", "State", "Recorded State", "Control"])
            for i, (state, recorded_state, control) in enumerate(zip(self.saved_states[5:], self.recorded_states[5:], self.saved_controls[5:])):  # Start from the 5th step
                writer.writerow([i] + list(state) + list(recorded_state) + list(control))
        print("Trajectory data saved to trajectory_data.csv.")


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
