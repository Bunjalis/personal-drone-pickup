import rclpy
import signal
import sys
import numpy as np
import copy
import math
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
        #self.x_traj = 0.0 * np.ones_like(time_space)
        #self.y_traj = 1.0 * np.ones_like(time_space)
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



        if self.current_pose is not None:
            # Extract current orientation quaternion from the current pose
            quaternion = self.current_pose[3:7]  # [qw, qx, qy, qz]
            rpy = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]]).as_euler('xyz', degrees=False)

            # Extract current yaw
            current_yaw = rpy[2]

            # Extract desired yaw from the trajectory
            desired_quaternion = [self.qx_traj[self.step_counter], self.qy_traj[self.step_counter], self.qz_traj[self.step_counter], self.qw_traj[self.step_counter]] if self.step_counter < self.steps else [self.qx_traj[-1], self.qy_traj[-1], self.qz_traj[-1], self.qw_traj[-1]]
            desired_rpy = R.from_quat(desired_quaternion).as_euler('xyz', degrees=False)
            desired_yaw = desired_rpy[2]

            # Print current and desired yaw in the order qw, qx, qy, qz
            current_quaternion_rounded = [f"{value:+.3f}" for value in [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]]
            desired_quaternion_rounded = [f"{value:+.3f}" for value in [desired_quaternion[3], desired_quaternion[0], desired_quaternion[1], desired_quaternion[2]]]
            print(f"Current Quaternion: {current_quaternion_rounded}, Desired Quaternion: {desired_quaternion_rounded}")


        if self.armed == True:

            scale = 2
            
            for j in range(60):
                if self.step_counter + j*scale < self.steps:
                    yref = np.array([self.x_traj[self.step_counter + j*scale], self.y_traj[self.step_counter + j*scale], self.z_traj[self.step_counter + j*scale], self.qw_traj[self.step_counter + j*scale], self.qx_traj[self.step_counter + j*scale], self.qy_traj[self.step_counter + j*scale], self.qz_traj[self.step_counter + j*scale], 0,0,0, 0, 0, 0, 0.6, 0.6, 0.6, 0.6])
                else:

                    print(f"GOT THE THE END THIS IS THE FINAL YREF")
                    yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0, 0, 0, 0, 0, 0.6, 0.6, 0.6, 0.6])
                
                self.ocp.set(j, "yref", yref)
            self.ocp.set(0, "lbx", self.current_pose)
            self.ocp.set(0, "ubx", self.current_pose)


            status = self.ocp.solve()
            if status != 0:
                raise Exception(f'acados returned status {status}.')

            u = self.ocp.get(0, "u")

            #print(f"u: {u}")

            u_command = u.tolist()


            msg.armed = True
            msg.channel_0 = u_command[0]
            msg.channel_1 = u_command[1]
            msg.channel_2 = u_command[2]
            msg.channel_3 = u_command[3]

            self.step_counter += 1
        
        else:
            step_counter = 0
            u_command = [0, 0, 0, 0]

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
