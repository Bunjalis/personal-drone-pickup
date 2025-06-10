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
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            vx, vy, vz = state[7:10]
            vr, vp, vyaw = state[10:13]
            kpz, kiz, kdz = 75.0, 42.857, 32.8125
            kpx, kix, kdx = 0.00414, 0.0000345, 0.1242
            kpy, kiy, kdy = 0.00414, 0.0000345, 0.1242

            kpp, kip, kdp =2.4, 0.48, 3.09
            kpr, kir, kdr = 2.1, 0.84, 1.3125
            kpyaw, kiyaw, kdyaw = 0.804, 0.29236, 0.55275

        
            
            #kpyaw, kdyaw, kiyaw = 1.9, 1.0, 6.95
        
            #chromosomes= [np.float64(0.0), np.float64(0.06274605548265022), np.float64(0.00012207403790398877), 0.003173924985503708, np.float64(0.31025116733298747), np.float64(0.0020142216254158147), np.float64(0.0037384318422839535), np.float64(0.8886786550800712), np.float64(0.010223466670735709), 0.40703130364458956, np.float64(0.7386073196969583), np.float64(6.613819990692068), np.float64(34.52121212121212), 47.895934959349596, np.float64(32.540390530005006)]
            #[kpx, kix, kdx, kpy, kiy, kdy, kpr, kir, kdr, kpp, kip, kdp, kpz, kiz, kdz] = chromosomes
            xd_dotdot = xd/(dt*dt)
            yd_dotdot = yd/(dt*dt)
            Ux = kpx*(xd-x) + kix*(xd-x)*dt + kdx*((xd-x) - self.xError_prev)/dt #+ xd_dotdot
            Uy = kpy*(yd-y) + kiy*(yd-y)*dt + kdy*((yd-y) - self.yError_prev)/dt #+ yd_dotdot

            #rd = Ux*sin(yaw) - Uy*cos(yaw)
            #pd = Ux*cos(yaw) + Uy*sin(yaw)
            
            #force = (self.g + kpz*(zd-z) + kiz*(zd-z)*dt + kdz*((zd-z) - self.zError_prev)/dt)*self.M/(cos(r)*cos(p)) #depends of roll and pitch
            #force = (self.g -kpz*(z-zd) - kdz*(vz-0))*self.M
            force = (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M*(cos(r)*cos(p))
            Ux =  kpx*(xd-x) + kix*(xd-x)*dt - kdx*vx#+ xd_dotdot
            Uy = kpy*(yd-y) + kiy*(yd-y)*dt - kdy*vy #+ yd_dotdot
            rd = 0 # (Ux*sin(yaw) - Uy*cos(yaw))*self.M/force#(self.M/force)
            pd = 0 #(Ux*cos(yaw) + Uy*sin(yaw))*self.M/force#(self.M/force)
            rTau = (kpr*(rd-r) + kir*(rd-r)*dt - kdr*vr)*self.Ixx
            pTau= (kpp*(pd-p) + kip*(pd-p)*dt - kdp*vp)*self.Iyy
            yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt - kdyaw*vyaw)*self.Izz


            self.xError_prev = xd-x
            self.yError_prev = yd-y
            self.zError_prev = zd-z
            self.rError_prev = rd-r
            self.pError_prev = pd-p
            self.yawError_prev = yawd-yaw
            #print(f"f:{force}, rTau: {rTau}, pTau:{pTau}, yawTau{yawTau}")
            
            #force = 1/(1+np.exp(-force)) #math.tanh(force) #
            
            self.xError_prev = xd-x
            self.yError_prev = yd-y
            self.zError_prev = zd-z
            self.rError_prev = rd-r
            self.pError_prev = pd-p
            self.yawError_prev = yawd-yaw
            '''if force > 1:
                force = 1.0
            elif force < 0:
                force = 0.0

            if rTau> 1:
                rTau = 1.0
            elif rTau < -1:
                rTau = -1.0
            if pTau> 1:
                pTau = 1.0
            elif pTau< -1:
                pTau = -1.0
            if yawTau > 1:
                yawTau = 1.0
            elif yawTau < -1:
                yawTau = -1.0'''
                
            Cf = 1.42e-6
            Ct = 2.84e-7
            l = 0.11
            max_motor_speed = 1755*25.2
            u1 = sqrt(abs(force/(4*Cf) - rTau/(2*Cf*l) + yawTau/(4*Ct)))/max_motor_speed
            u2 = sqrt(abs(force/(4*Cf) + pTau/(2*Cf*l) - yawTau/(4*Ct)))/max_motor_speed
            u3 = sqrt(abs(force/(4*Cf) + rTau/(2*Cf*l) + yawTau/(4*Ct)))/max_motor_speed
            u4 = sqrt(abs(force/(4*Cf) - pTau/(2*Cf*l) - yawTau/(4*Ct)))/max_motor_speed
            #u = [u1, u2, u3, u4]
            u = [rTau,pTau,force,yawTau]  # [aetr] [w_x (-1.0,1.0), w_y, throttle (0,1), w_z]

            print(f"control output {u[0]}, {u[1]}, {u[2]}, {u[3]}")
        
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]*2)-1, 3), channel_3=round(u[3], 3))
            #msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round((u[2]), 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)

        else:         
            #msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
            msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=0.0, channel_3=0.0)
 
            msg.channel_0 = 0.05
            msg.channel_1 = 0.05
            msg.channel_2 = 0.05
            msg.channel_3 = 0.05
            self.pre_start_counter += 1
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
