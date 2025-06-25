import rclpy
import signal
import sys
import numpy as np
import math
from math import cos, sin, sqrt, hypot, pi
from rclpy.node import Node
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand, InvertedPendulumStates
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
import matplotlib.pyplot as plt

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.IP_state_subscription_ = self.create_subscription(InvertedPendulumStates, '/pendulum_state_publisher', self.IP_state_callback, 10)
        self.current_pose = None
        self.setpoint = np.array([0.5, 0.0, 1.0])
        self.currentPenPose = None
        #self.pendulumVelocity = None
        # Set up control loop
        self.control_frequency = 30.0
        self.dt = 1.0 / self.control_frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.pre_start_counter = 0
        self.pre_start_steps = self.control_frequency
        self.armed = False

        self.gui = GUI(self)

        self.g = 9.81
        self.M = 0.65
        self.Ixx = 0.001744744189
        self.Iyy = 0.001400539551
        self.Izz = 0.002782410904
        self.aError = []
        self.bError = []
        self.xError = []
        self.yError = []
        self.zError = []
        self.rollError = []
        self.pitchError = []
        self.yawError = []
        self.timePoints = []
        self.t = 0

        self.testInvPen = False
        self.pen_length = 0.3
        self.pen_mass =  0.0
        self.a = 0.0
        self.b = 0.0
        self.a_dot = 0.0
        self.b_dot = 0.0
        self.a_ddot = 0.0
        self.b_ddot = 0.0
        self.eta = math.sqrt(self.pen_length**2 - self.a**2 -self.b**2)

        self.vx_prev = 0.0
        self.vy_prev = 0.0
        self.vz_prev = 0.0
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.z_prev = 0.0
        self.prevForce = 0.0

        self.vx_approx = 0.0
        self.vy_approx = 0.0
        self.vz_approx = 0.0


    # Recieve motion capture data
    def pose_callback(self, msg: MotionCaptureState):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        self.current_pose = np.concatenate((position, orientation, linear_velocity, angular_velocity))
    
    def IP_state_callback(self, msg:InvertedPendulumStates):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        self.currentPenPose = np.concatenate((position, orientation, linear_velocity, angular_velocity))
        
        
        #print(f"penAngle: {self.currentPenPose}")

    

    def navController(self):
        state = self.current_pose
        xd, yd, zd = self.setpoint
        yawd = 0.0
        dt = self.dt
        x, y, z = state[0:3]
        r, p, yaw = self.quaternion_to_euler(*state[3:7])
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]
        
        pTau = -1*np.array([0.0034,0.0490,0.0071,0.0140])@np.array([[x-xd],[p],[vx],[vp]]) 
        rTau = -1*np.array([-0.0043, 0.0611, -0.0089, 0.0174])@np.array([[y-yd],[r],[vy],[vr]])
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        force =  (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M /(cos(r)*cos(p))
        self.prevForce = force
        #force = force/self.M
        kpyaw, kiyaw, kdyaw = 80.0, 10.0, 50.0#6.0, 1.5, 1.75
        yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt + kdyaw*(-vyaw))*self.Izz
        return force, rTau[0], pTau[0], yawTau

    def FIPController(self):
        state = self.current_pose
        xd, yd, zd = self.setpoint
        yawd = 0.0
        dt = self.dt
        x, y, z = state[0:3]
        r, p, yaw = self.quaternion_to_euler(*state[3:7])
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]
        
        a,b,a_dot,b_dot = self.computePenPosition()
        
        self.aError.append(a)
        self.bError.append(b)
        
        
        pTau = -1*np.array([0.0034,0.0490,0.02,0.0071,0.0140, 0.0])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([-0.0043,0.0611,0.0, -0.0089, 0.0174,0.0])@np.array([[y-yd],[r],[b],[vy],[vr], [b_dot]])
        #pTau = -1*np.array([ -0.0032,    0.2349,   -0.9595,   -0.0064,    0.0252,   -0.1675])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        #rTau = -1*np.array([0.1762,1.0184,6.6954,0.2046,0.0646,1.1709])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])
        
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        force =  (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M /(cos(r)*cos(p))
        self.prevForce = force
        #force = force/self.M
        kpyaw, kiyaw, kdyaw = 80.0, 10.0, 50.0#6.0, 1.5, 1.75
        yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt + kdyaw*(-vyaw))*self.Izz
        return force, rTau[0], pTau[0], yawTau


    def control_loop(self):
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0
        self.t += self.dt

        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg.armed = True
            msg.channel_0 = 0.05
            msg.channel_1 = 0.05
            msg.channel_2 = 0.05
            msg.channel_3 = 0.05
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:
            state = self.current_pose
            goal = self.setpoint
            xd, yd, zd = self.setpoint
            yawd = 0.0
            x, y, z = state[0:3]
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            vx, vy, vz = state[7:10]
            vr, vp, vyaw = state[10:13]
            self.xError.append(xd-x)
            self.yError.append(yd-y)
            self.zError.append(zd-z)
            self.rollError.append(r)
            self.pitchError.append(p)
            self.yawError.append(yaw)
            self.timePoints.append(self.t)

            # CONTROL CODE GOES HERE
            if self.testInvPen:
                force, rTau, pTau, yawTau = self.FIPController()
            else:
                force, rTau, pTau, yawTau = self.navController()
            
            
            
            Cf = 1.42e-6
            Ct = 2.84e-7

            l_x = 0.0865
            l_y = 0.073

            max_motor_speed = 4631.0# 1755*25.2
            
            '''if force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct)< 0:
                u1 = 0.0
            else:
                u1 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed

            if (force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct)) < 0:
                u2 = 0.0
            else:
                u2 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed

            if force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct) < 0:
                u3 = 0.0
            else:
                u3 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed


            if force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct)< 0:
                u4 = 0.0
            else:
                 u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed'''


            ########################################################
            print(f"force: {force}, rTau: {rTau}, pTau: {pTau}, yawTau: {yawTau}")
            u1 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed
            u2 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
            u3 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
            u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed
            
            u = [u1,u2,u3,u4]
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 3), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)
            
        else:         
            self.pre_start_counter = 0

        self.cmd_publisher_.publish(msg)
        print(f"control output {msg.channel_0}, {msg.channel_1}, {msg.channel_2}, {msg.channel_3}")

    def computePenPosition(self):
        dt = self.dt
        L= self.pen_length
        state = self.current_pose
        
        roll, pitch, yaw = self.quaternion_to_euler(*state[3:7])
        x, y, z = state[0:3]
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]

        eta = self.eta
        r = self.a
        s=self.b
        r_dot = self.a_dot
        s_dot = self.b_dot
        s_ddot = self.b_ddot
        r_ddot = self.a_ddot
        fp1 = 3*r*eta*self.g/(4*(L**2-s**2)) + ((r**3)*(s_dot**2 + s*s_ddot) - 2*r**2*s*r_dot*s_dot)/((L**2 -s**2)*eta**2) + (r*(-(L**2)*s*s_ddot + s_ddot*s**3 + (s**2)*(r_dot**2) -(L**2)*(r_dot**2)-(L**2)*(s_dot**2))/((L**2-s**2)*eta**2))
        fp2 = 3*s*eta*self.g/(4*(L**2-r**2)) + ((s**3)*(r_dot**2 + r*r_ddot) - 2*s**2*r*s_dot*r_dot)/((L**2 -r**2)*eta**2) + (s*(-(L**2)*r*r_ddot + r_ddot*r**3 + (r**2)*(s_dot**2) -(L**2)*(s_dot**2)-(L**2)*(r_dot**2))/((L**2-r**2)*eta**2))


        #zd_ddot = 4*(L**2-r**2)*(bd_ddot - fp2)*(1/(3*(s+1e-3)*eta))
        
        x_ddot, y_ddot, z_ddot = (vx-self.vx_prev)/dt, (vy-self.vy_prev)/dt, (vz-self.vz_prev)/dt
        utz = self.prevForce

        accel_x = (utz / (self.M + self.pen_mass)) * (cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw))
        accel_y = (utz / (self.M + self.pen_mass)) * (cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw))
        accel_z = (utz / (self.M + self.pen_mass)) * (cos(roll) * cos(pitch)) - self.g
        #print(f"ax: {accel_x}, ay: {accel_y}, az: {accel_z}")
        #print(f"xd: {x_ddot}, yd: {y_ddot}, zd: {z_ddot}")
        self.vx_approx += accel_x*dt
        self.vy_approx += accel_y*dt
        self.vz_approx += accel_z*dt
        #print(f"ax: {vx}, ay: {vy}, az: {vz}")
        #print(f"xd: {(x-self.x_prev)/dt}, yd: {(y-self.y_prev)/dt}, zd: {(z-self.z_prev)/dt}")
        #print(f"x: {self.vx_approx}, y: {self.vy_approx}, z: {self.vz_approx}")
        #x_ddot, y_ddot, z_ddot = accel_x, accel_y, accel_z
        r_ddot = x_ddot/(-4*(L**2-s**2)/(3*eta**2)) + (3*r*eta*z_ddot/(4*(L**2-s**2))) + fp1 
        s_ddot  = y_ddot/(-4*(L**2-r**2)/(3*eta**2)) + (3*s*eta*z_ddot/(4*(L**2-r**2)))  +fp2 



        self.a_ddot = r_ddot
        self.b_ddot = s_ddot 
        self.a_dot += self.a_ddot*self.dt 
        self.b_dot += self.b_ddot*self.dt
        self.a += self.a_dot*self.dt
        self.b += self.b_dot*self.dt
        if L**2 - self.a**2 - self.b**2 <= 0:
            self.eta = 1e-02
            self.b = 0.0
            '''if self.a < 0:
                self.a = -L
            else:
                self.a = L'''
        else:
            self.eta = (L**2 - self.a**2 - self.b**2)**(1/2)
        self.vx_prev, self.vy_prev, self.vz_prev = vx, vy, vz
        self.x_prev, self.y_prev, self.z_prev = x, y, z
        return self.a,self.b,self.a_dot,self.b_dot 

    def plotSystemResponse(self):
        time = self.timePoints
        if self.testInvPen:
            figure1, ax1 = plt.subplots(2,1)
            ax1[0].plot(time, self.aError)
            ax1[0].set_title('a position error over time')
            ax1[0].set_ylabel('a position error')
            ax1[0].set_xlabel('time (s)')

            ax1[1].plot(time, self.bError)
            ax1[1].set_title('b position error over time')
            ax1[1].set_ylabel('b position error')
            ax1[1].set_xlabel('time (s)')
            plt.tight_layout()

        figure2, ax2 = plt.subplots(4,1)
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

        figure3, ax3 = plt.subplots(2,1)
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
