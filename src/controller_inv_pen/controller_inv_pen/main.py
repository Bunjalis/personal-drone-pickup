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
        #print(self.current_pose)
        #print(self.setpoint)
        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0
   
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            #msg = ELRSCommand(armed=True, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            
            msg.armed = True
            msg.channel_0 = 0.05
            msg.channel_1 = 0.05
            msg.channel_2 = 0.05
            msg.channel_3 = 0.05
            self.pre_start_counter += 1
            self.cmd_publisher_.publish(msg)
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:
            
            
            xd, yd, zd, yawd = self.setpoint

            state = self.current_pose
            dt = self.dt
            x, y, z = state[0:3]
            yawd = 0
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            vx, vy, vz = state[7:10]
            vr, vp, vyaw = state[10:13]
            
            self.xError_prev = xd-x
            self.yError_prev = yd-y
            self.zError_prev = zd-z
 
            self.yawError_prev = yawd-yaw
            #print(f"f:{force}, rTau: {rTau}, pTau:{pTau}, yawTau{yawTau}")
            wy = -1*np.array([21.4067, 18.0000, 10.9072])@np.array([[x-xd],[p],[vx]]) 
            wx = -1*np.array([-0.6116,6.0000,-1.1213])@np.array([[y-yd],[r],[vy]])

            #force = K3@np.array([[z], [vz]])
            kpz, kiz, kdz = 15.0, 10.0, 10.0 
            force =  (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M /(cos(r)*cos(p))
            #force = (self.g+ 15*(zd-z) + 10*(0-vz))*self.M
            kpyaw = 0.804 
            kiyaw = 0.29236 
            kdyaw = 0.55275
    
            #wz =  (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt + kdyaw*((-vyaw)))*self.Izz #(kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt + kdyaw*((yawd-yaw)-self.yawError_prev)/dt)*self.Izz
            kpyaw, kiyaw, kdyaw = 80.0, 10.0, 50.0#6.0, 1.5, 1.75
            wz = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt + kdyaw*(-vyaw))*self.Izz
            #print(f"f:{force}, wx: {angularV[0][0]}, wy: {angularV[1][0]}, wz: { angularV[2][0]}")
            Cf = 1.42e-6
            Ct = 2.84e-7
            max_motor_speed = 4631.0
            maxForce = Cf*(4*max_motor_speed**2)
            throttle = force/maxForce
            u = [wx[0],wy[0],throttle,wz]  # [aetr] [w_x (-1.0,1.0), w_y, throttle (0,1), w_z]


            print(f"control output {u[0]}, {u[1]}, {u[2]}, {u[3]}")

            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)

        else:         
            #msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)      
            self.pre_start_counter = 0

        self.cmd_publisher_.publish(msg)
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
