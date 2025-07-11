import rclpy
import signal
import sys
import numpy as np
from rclpy.node import Node
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray
from math import cos, sin, sqrt, hypot, pi
import math
import matplotlib.pyplot as plt
import csv 


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)

        self.current_pose = None
        self.setpoint = np.array([1.0, 1.0, 1.0])

        # Set up control loop
        self.control_frequency = 120.0
        self.dt = 1.0 / self.control_frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.pre_start_counter = 0
        self.pre_start_steps = self.control_frequency
        self.armed = False

        self.g = 9.81
        self.M = 0.65
        self.Ixx = 0.001744744189
        self.Iyy = 0.001400539551
        self.Izz = 0.002782410904

        self.g = 9.81
        self.M = 0.65
        self.Ixx = 0.001744744189
        self.Iyy = 0.001400539551
        self.Izz = 0.002782410904

        self.gui = GUI(self)

        self.xError = []
        self.yError = []
        self.zError = []
        self.rollError = []
        self.pitchError = []
        self.yawError = []
        self.timePoints = []
        self.t = 0
        self.xError_prev = 0.0
        self.yError_prev = 0.0
        self.zError_prev = 0.0
        self.rError_prev = 0.0
        self.pError_prev = 0.0
        self.yawError_prev = 0.0
        self.exitGUI = False
        
        self.dataFileName = "dataFile.csv"
        self.dataFile =  open(self.dataFileName,'w', newline="")
        self.dataWriter = csv.DictWriter(self.dataFile, fieldnames=['xError', 'yError', 'zError', 'rollError', 'pitchError', 'yawError'])
        self.dataWriter.writeheader()

        self.usingBetaFLight =False

    # Recieve motion capture data
    def pose_callback(self, msg: MotionCaptureState):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        self.current_pose = np.concatenate((position, orientation, linear_velocity, angular_velocity))

    def control_loop(self):

        # For saftey generate a message with all channels set to 0.0
        msg = ELRSCommand()
        msg.armed = False
        if self.usingBetaFLight:
            msg.channel_0 = 0.0 # roll (-1,1)
            msg.channel_1 = 0.0 # pitch
            msg.channel_2 = -1.0 # throttle (-1 = 0)
            msg.channel_3 = 0.0 # yaw
        else:
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = 0.0
            msg.channel_3 = 0.0



        self.t += self.dt

        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        if self.armed and self.pre_start_counter < self.pre_start_steps and not self.usingBetaFLight:
            msg.armed = True
            msg.channel_0 = 0.05
            msg.channel_1 = 0.05
            msg.channel_2 = 0.05
            msg.channel_3 = 0.05
            self.pre_start_counter += 1
        if self.armed and self.pre_start_counter < self.pre_start_steps and self.usingBetaFLight:
            msg.armed = True
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = -0.999
            msg.channel_3 = 0.0
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:
            state = self.current_pose
            goal = self.setpoint


            # CONTROL CODE GOES HERE
            xd, yd, zd = self.setpoint
            yawd = 0.0
            dt = self.dt
            x, y, z = state[0:3]
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            vx, vy, vz = state[7:10]
            vr, vp, vyaw = state[10:13]

            kpz, kiz, kdz = 15.0, 10.0, 10.0 
            kpx, kix, kdx = 0.06938, 0.0, 0.14488#0.006, 0.0, 0.0 #6.0, 0, 12.0 
            kpy, kiy, kdy = 0.07, 0.0, 0.1456 #0.06, 0.0, 0.001#0.006, 0.0, 0.0 #6.0, 0, 12.0 

            kpp, kip, kdp = 28.8235, 0.0, 8.235 #90.0, 10.0, 20.0 #30.0 # 80.0, 10.0, 50.0 note derivative term is very sensitive to noise (reduce as much as possible)
            kpr, kir, kdr = 43.64, 0.0, 12.43 #60.0, 10.0, 40.0 # 80.0, 10.0, 50.0 
            kpyaw, kiyaw, kdyaw =60.0, 10.0, 30.0 

            '''kpx, kix, kdx = 0.00414, 0.0000345, 0.1242
            kpy, kiy, kdy = 0.00414, 0.0000345, 0.1242

            kpp, kip, kdp = 2.4, 0.48, 3.09
            kpr, kir, kdr = 2.1, 0.84, 1.3125'''
 
            force = (self.g +kpz*(zd-z) + kdz*(0-vz))*self.M*(cos(r)*cos(p))
            Ux =  kpx*(xd-x) + kix*(xd-x)*dt - kdx*vx
            Uy = kpy*(yd-y) + kiy*(yd-y)*dt - kdy*vy 
            rd = (Ux*sin(yaw) - Uy*cos(yaw)) #*self.M/force
            pd = (Ux*cos(yaw) + Uy*sin(yaw)) #*self.M/force
            rTau = (kpr*(rd-r) + kir*(rd-r)*dt - kdr*vr)*self.Ixx
            pTau= (kpp*(pd-p) + kip*(pd-p)*dt - kdp*vp)*self.Iyy
            yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt - kdyaw*vyaw)*self.Izz

              
            
            #print(f"Ux: {Ux}, Uy: {Uy}")
            self.xError_prev = xd-x
            self.yError_prev = yd-y
            self.zError_prev = zd-z
            self.rError_prev = rd-r
            self.pError_prev = pd-p
            self.yawError_prev = yawd-yaw
            self.xError.append(xd-x)
            self.yError.append(yd-y)
            self.zError.append(zd-z)
            self.rollError.append(rd-r)
            self.pitchError.append(pd-p)
            self.yawError.append(yawd-yaw)
            self.timePoints.append(self.t)
            
            self.dataWriter.writerow({'xError': xd-x, 'yError': yd-y, 'zError': zd-z, 'rollError': rd-r, 'pitchError':pd-p, 'yawError': yawd-yaw})
            
            Cf = 1.42e-6 # motor constant 
            Ct = 2.84e-7

            l_x = 0.0865
            l_y = 0.073

            max_motor_speed = 4631.0# 1755*25.2
            
            
            if force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct)< 0:
                u1 = 0.0
            else:
                u1 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed

            if (force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct)) < 0:
                u2 = 0.0
            else:
                u2 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed

            if force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct) < 0:
                u3 = 0.0
            else:
                u3 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed


            if force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct)< 0:
                u4 = 0.0
            else:
                 u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
                 
            u = [u1,u2,u3,u4]


            '''u1 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed
            u2 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
            u3 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
            u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed'''
            
            

            '''maxForce = (Cf*max_motor_speed**2) 
            maxTorque = (Ct*max_motor_speed**2) 
        
            wy = -1*np.array([12.6320, 125.3600,   25.0785])@np.array([[x-xd], [p], [vx]])
            wy = (( wy[0]))/100.0
            wx = -1*np.array([-12.6320,   125.3600,   -25.0785])@np.array([[y-yd], [r], [vy]]) # roll control
            wx = (( wx[0]))/100.0
           
            kpz, kiz, kdz = 15.0, 10.0, 10.0 
            force = (self.g + kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M
            throttle = 2*(force)/(maxForce) - 1
            if throttle < -1:
                throttle = -1.0
            if throttle > 1:
                throttle = 1.0
            sumYawError = sum(self.yawError)
            wz =  -0.5*(yawd-yaw) #-0.001*sumYawError*dt# (0.2500*u1 - 0.2500*u2 - 0.2500*u3 + 0.2500*u4)/100.0 # -0.002*yaw#kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt - kdyaw*vyaw #-yaw #-1.25*yaw -0.5*vyaw#2.0*(0.2500*u1 - 0.2500*u2 - 0.2500*u3 + 0.2500*u4)/100.0
      
            print(f"force: {force}, r: {wx}, p: {wy}, yaw: {wz}")
            u = [wx, wy, throttle, wz]'''
    

            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3))
            print(f"x: {x}, y: {y}, z: {z}, r: {r}, p: {p}, yaw: {yaw}, vx: {vx}, vy: {vy}")
            self.cmd_publisher_.publish(msg)
            
        else:         
            self.pre_start_counter = 0

        self.cmd_publisher_.publish(msg)
        print(f"control output {msg.channel_0}, {msg.channel_1}, {msg.channel_2}, {msg.channel_3}")

    def plotSystemResponse(self):
        figure2, ax2 = plt.subplots(4,1)
        time = self.timePoints
        ax2[0].plot(time, self.xError)
        ax2[0].set_title('x position error over time')
        ax2[0].set_ylabel('x position error')
        ax2[0].set_xlabel('time (s)')

        ax2[1].plot(time, self.yError)
        ax2[1].set_title('y position error over time')
        ax2[1].set_ylabel('y position error')
        ax2[1].set_xlabel('time (s)')

        ax2[2].plot(time, self.zError)
        ax2[2].set_title('z position error over time')
        ax2[2].set_ylabel('z position error')
        ax2[2].set_xlabel('time (s)')

        ax2[3].plot(time, self.yawError)
        ax2[3].set_title('yaw position error over time')
        ax2[3].set_ylabel('yaw position error')
        ax2[3].set_xlabel('time (s)')
        plt.tight_layout()

        figure, ax3 = plt.subplots(2,1)
        ax3[0].plot(time, self.rollError)
        ax3[0].set_title('roll position error over time')
        ax3[0].set_ylabel('roll position error')
        ax3[0].set_xlabel('time (s)')

        ax3[1].plot(time, self.pitchError)
        ax3[1].set_title('pitch position error over time')
        ax3[1].set_ylabel('pitch position error')
        ax3[1].set_xlabel('time (s)')
        plt.tight_layout()
        plt.show()


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
        self.plotSystemResponse()
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
