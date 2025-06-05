import rclpy
import signal
import sys
import numpy as np
import math
from math import cos, sin, sqrt, hypot, pi
from rclpy.node import Node
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)

        self.current_pose = None
        self.setpoint = np.array([0.0, 0.0, 1.0,0.0])

        # Set up control loop
        self.control_frequency = 30.0
        self.dt = 1.0 / self.control_frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.pre_start_counter = 0
        self.pre_start_steps = self.control_frequency
        self.armed = False

        self.gui = GUI(self)

        self.xError_prev = 0
        self.yError_prev = 0
        self.zError_prev = 0
        self.rError_prev = 0
        self.pError_prev = 0
        self.yawError_prev = 0
        self.g = 9.81
        self.M = 0.65
        self.Ixx = 0.001744744189
        self.Iyy = 0.001400539551
        self.Izz = 0.002782410904


    # Recieve motion capture data
    def pose_callback(self, msg: MotionCaptureState):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        self.current_pose = np.concatenate((position, orientation, linear_velocity, angular_velocity))

    def control_loop(self):

        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:
            
            
            xd, yd, zd, yawd = self.setpoint
            state = self.current_pose
            x, y, z = state[0:3]
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            


            kpz, kpr, kpp, kpyaw = 75, 2.1, 2.4, 0.804 #185, 0.88,0.62, 0.99# #27, 0.5, 0.5, 1.9
            kiz, kir, kip, kiyaw = 42.8571, 0.84, 0.48, 0.29236 #105, 0.9, 0.8, 0.5#42.8571, 0.84, 0.48, 0.29236 #17,0.1, 0.1, 1 
            kdz, kdr, kdp, kdyaw = 32.8125, 1.3125, 3.09, 0.55275 #22.5, 0.3, 0.05, 0.56 #32.8125, 1.3125, 3.09, 0.55275 #15,6,5, 6.95 

            kpx, kpy = 0.00414, 0.00414 #0.28, 0.36# #0.02, 0.025 #0.09, 0.07 #0.1, 0.1 #0.03,0.04 #0.1,0.1 #0.00414, 0.00414
            kix, kiy = 0.0000345, 0.0000345 #2.73e-6, 1.56e-5#0.0000345, 0.0000345 #0.00004, 0.00004 #0.00001, 0.00001 #0.7,0.01 #0.1,0.1 #0.0000345, 0.0000345
            kdx, kdy = 0.1242, 0.1242 #0.63, 0.88#0.1242, 0.1242 # 0.03,0.03#0.16, 0.16  # 0.1, 0.1 #0.11,0.11 #0.1,0.1 #0.1242, 0.1242 
            
            xd_dotdot = xd/(self.dt*self.dt)
            yd_dotdot = yd/(self.dt*self.dt)
            Ux = kpx*(xd-x) + kix*(xd-x)*self.dt + kdx*((xd-x) - self.xError_prev)/self.dt #+ xd_dotdot
            Uy = kpy*(yd-y) + kiy*(yd-y)*self.dt + kdy*((yd-y) - self.yError_prev)/self.dt #+ yd_dotdot

            #rd = Ux*sin(yaw) - Uy*cos(yaw)
            #pd = Ux*cos(yaw) + Uy*sin(yaw)

            force = (self.g + kpz*(zd-z) + kiz*(zd-z)*self.dt + kdz*((zd-z) - self.zError_prev)/self.dt)*self.M/(cos(r)*cos(p)) #depends of roll and pitch
            print(f"force: {force}")


            #rd = (-Ux*sin(yaw) + Uy*cos(yaw))*(self.M/force)
            #pd = (-Ux*cos(yaw) - Uy*sin(yaw))*(self.M/force)
            rd = (Ux*sin(yaw) - Uy*cos(yaw))*(self.M/force)
            pd = (Ux*cos(yaw) + Uy*sin(yaw))*(self.M/force)
            rTau = (kpr*(rd-r) + kir*(rd-r)*self.dt + kdr*((rd-r) - self.rError_prev)/self.dt)*self.Ixx
            pTau = (kpp*(pd-p) + kip*(pd-p)*self.dt + kdp*((pd-p)-self.pError_prev)/self.dt)*self.Iyy
            yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*self.dt + kdyaw*((yawd-yaw)-self.yawError_prev)/self.dt)*self.Izz

            self.xError_prev = xd-x
            self.yError_prev = yd-y
            self.zError_prev = zd-z
            self.rError_prev = rd-r
            self.pError_prev = pd-p
            self.yawError_prev = yawd-yaw

            #force = math.tanh(force) # 1/(1+np.exp(-force))
            if force > 1:
                force = 1.0
            elif force < 0:
                force = 0
            rTau = math.tanh(rTau)
            pTau = math.tanh(pTau)
            yawTau = math.tanh(yawTau)
            

            u = [rTau,pTau,force,yawTau]  # [aetr] [w_x (-1.0,1.0), w_y, throttle (0,1), w_z]

            print(f"control output {u[0]}, {u[1]}, {u[2]}, {u[3]}")

            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)

        else:         
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            self.cmd_publisher_.publish(msg)
            self.step_counter = 0
            print(f"control output {msg.channel_0}, {msg.channel_1}, {msg.channel_2}, {msg.channel_3}")

    # Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)
    # Returns angles in radians.
    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    # GUI functions
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
