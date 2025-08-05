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
from .acados import generate_ocp_controller, set_initial_guess
from .gui import GUI
from .trajectories import hover_trajectory, circle_trajectory, power_loop_trajectory, hover_and_rotate
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


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


        #self.timer_test_angular = self.create_timer(self.dt, self.angular_velocity_test)  # Adjust the timer frequency as needed

        

        self.traj = circle_trajectory(self.dt)
        #self.traj = hover_trajectory(self.dt)   


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
        self.initial_guess_set = False  # Flag to track if initial guess has been set


        # Initialize CSV file at the start of the program
        if not hasattr(self, 'csv_initialized'):
            self.csv_initialized = True
            self.csv_file = open('control_results.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header row
            self.csv_writer.writerow([
                'Step', 'u0', 'u1', 'create_timeru2', 'u3','u4', 'u5', 'u6', 'u7',
                'px', 'py', 'pz', 'rw', 'rx', 'ry', 'rz',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz',
                'sp_px', 'sp_py', 'sp_pz', 'sp_rw', 'sp_rx', 'sp_ry', 'sp_rz',
                'sp_vx', 'sp_vy', 'sp_vz', 'sp_wx', 'sp_wy', 'sp_wz',

        ])

    def pose_callback(self, msg: MotionCaptureState):
        p, o, lv, av = msg.pose.position, msg.pose.orientation, msg.twist.linear, msg.twist.angular
        self.current_pose = np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z, lv.x, lv.y, lv.z, av.x, av.y, av.z
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

            # Set reference trajectory for the horizon
            for j in range(self.N):
                sc = self.step_counter + j * self.skip_steps
                yref = self.traj[:, sc]
                self.ocp.set(j, "yref", yref)

            sn = self.step_counter + self.N * self.skip_steps
            yref_N = self.traj[:, sn]
            self.ocp.set(self.N, "yref", yref_N)

            # Set current state constraint
            self.ocp.set(0, "lbx", self.current_pose )
            self.ocp.set(0, "ubx", self.current_pose )

            # Set initial guess based on hover solution
            # Only set on first solve or after failure for better performance

            set_initial_guess(self.ocp, 1)

            # Solve the MPC problem
            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status} after retry.')

            u = self.ocp.get(0, "u")


            


            #u_sqrt = np.sign(u) * np.sqrt(np.abs(u))
            print(u)

            msg = ELRSCommand(armed=True, channel_0=u[0], channel_1=u[1], channel_2=u[2], channel_3=u[3], channel_4=u[4], channel_5=u[5], channel_6=u[6], channel_7=u[7])

            #sd = 0.28 # -0.32890574
            #sd = u[1]
            #msg = ELRSCommand(armed=True, channel_0=-sd, channel_1=sd, channel_2=-sd, channel_3=sd, channel_4=sd, channel_5=-sd, channel_6=sd, channel_7=-sd)
            self.cmd_publisher_.publish(msg)





        else:
            print("Controller is not armed or current pose is not available.")
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=0.0, channel_3=0.0, channel_4=0.0, channel_5=0.0, channel_6=0.0, channel_7=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0








    def angular_velocity_test(self):

        if self.armed and self.sent_command == False:
            print("Sending initial command to arm the controller.")
            print(f"Current pose: {self.current_pose[10:13]}")
            # Convert the list 'u' to a NumPy array before passing it to self.sim_integrator.set
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
