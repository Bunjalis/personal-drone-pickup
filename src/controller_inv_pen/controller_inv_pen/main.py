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
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_matrix
import time
from .acados import generate_ocp_controller


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)
        self.IP_state_subscription_ = self.create_subscription(InvertedPendulumStates, '/pendulum_state_publisher', self.IP_state_callback, 10)
        self.current_pose = None
        self.setpoint = np.array([1.0,1.0, 1.0])
        self.currentPenPose = None
        #self.pendulumVelocity = None
        # Set up control loop
        self.control_frequency = 120.0 # 120.0
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
        self.b_dotError = []
        self.a_dotError = []
        self.y_dotError = []
        self.wxOutput = []
        self.timePoints = []
        self.t = 0
        self.modelOutputA = []
        self.modelOutputB = []
        self.modelOutputAdot = []
        self.modelOutputBdot = []

        self.testInvPen = True
        self.testMPC = False
        self.usingBetaFlight = True
        self.pen_length = 0.6
        self.pen_mass =  0.01
        self.a = 0.0
        self.b = 0.0
        self.a_dot = 0.0
        self.b_dot = 0.0
        self.a_ddot = 0.0
        self.b_ddot = 0.0
        self.eta = math.sqrt((self.pen_length/2)**2 - self.a**2 -self.b**2)
        self.bErrorSum = 0
        self.prev_bError = 0
        self.prev_rollError = 0

        # controller 
        self.last_a = 0.0
        self.last_b = 0.0
        self.last_eta = 0.3
        #model 
        self.a_prev = 0 #0.01
        self.b_prev = -0.01
        self.a_dotprev = 0.0
        self.b_dotprev = 0.0
        self.a_ddotprev = 0.0
        self.b_ddotprev = 0.0


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

        self.rd = 0
        self.pd = 0

        ######################## MPC variables #######################
        #self.steps = 90 * 30
        self.dt = 1.0 / 120.0
        self.step_counter = 0
        #self.timer = self.create_timer(self.dt, self.control_loop)
        self.traj = hover_trajectory(self.dt)  
        self.steps = self.traj.shape[1] - 1   
        # Get both the OCP solver and the integrator
        self.ocp, self.sim_integrator = generate_ocp_controller()

        time_space = np.linspace(0, self.steps * self.dt, self.steps)
        # Original trajectories
        self.x_traj = np.zeros_like(time_space)
        self.y_traj = np.zeros_like(time_space)
        self.z_traj = 1.0 * np.ones_like(time_space)

        # New oscillating trajectories
        #self.x_traj = 2.0 * np.sin(2 * np.pi * 1.0 * time_space)  # Sine wave with frequency 0.1 Hz
        #self.y_traj = 2.0 * np.sin(2 * np.pi * 0.5 * time_space)  # Sine wave with frequency 0.2 Hz
        #self.z_traj = 1.5 + 0.5 * np.sin(2 * np.pi * 0.5 * time_space)  # Sine wave with frequency 0.05 Hz

        roll_traj = np.zeros_like(time_space)  # Roll remains 0
        pitch_traj = np.zeros_like(time_space)  # Pitch remains 0
        yaw_traj = np.zeros_like(time_space)  # Pitch remains 0

        rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
        quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Converts to [q_x, q_y, q_z, q_w]

        self.qx_traj = quaternions[:, 0]
        self.qy_traj = quaternions[:, 1]
        self.qz_traj = quaternions[:, 2]
        self.qw_traj = quaternions[:, 3]

        # Calculate world frame velocities for the trajectory
        self.vx_traj = np.gradient(self.x_traj, self.dt)
        self.vy_traj = np.gradient(self.y_traj, self.dt)
        self.vz_traj = np.gradient(self.z_traj, self.dt)

        # Calculate desired angular velocities for the orientation trajectory
        self.ax_traj = np.zeros_like(time_space)  # Roll rate remains 0
        self.ay_traj = np.zeros_like(time_space)  # Pitch rate remains 0
        self.az_traj = np.zeros_like(time_space)  # Yaw rate trajectory

        self.executing_actions = False
        self.executed_steps = 0
        self.saved_states = []
        self.saved_controls = []
        self.recorded_states = []  # To store the recorded states
        self.initial_solve_state = None
        self.initial_solve_controls = None

        self.omega_est = np.array([0.1,0.1,0.1,0.1])  # Initialize omega_est if not already present

        self.pre_start_duration = 2.0  # Duration for the pre-start state in seconds
        self.pre_start_counter = 0  # Counter to track pre-start steps
        self.pre_start_steps = int(self.pre_start_duration / self.dt)  # Steps for pre-start state

        self.N = 60
        self.skip_steps = 3
        self.augmented_u = 0.0  # Initialize the augmented control input

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
        penState = self.currentPenPose
        self.currentPenPose = np.concatenate((position, orientation, linear_velocity, angular_velocity))
    

    def navController(self):

        state = self.current_pose
        xd, yd, zd = self.setpoint
        penState = self.currentPenPose
        a, b, eta = penState[0:3]
        a_dot, b_dot, eta_dot = penState[7:10]
        
        yawd = 0.0
        dt = self.dt
        x, y, z = state[0:3]
        r, p, yaw = self.quaternion_to_euler(*state[3:7])
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]
        print(f"a:{a}, b:{b}, eta:{eta}, a_dot:{a_dot}, b_dot:{b_dot}, eta_dot:{eta_dot}")
        print(f"x:{x}, y:{y}, z:{z}, x_dot:{vx}, y_dot:{vy}, z_dot:{vz}")
        


        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        kpx, kix, kdx = 0.06938, 0.0, 0.14488#0.006, 0.0, 0.0 #6.0, 0, 12.0 
        kpy, kiy, kdy = 0.07, 0.0, 0.1456 #0.06, 0.0, 0.001#0.006, 0.0, 0.0 #6.0, 0, 12.0 

        kpp, kip, kdp = 28.8235, 0.0, 8.235 #90.0, 10.0, 20.0 #30.0 # 80.0, 10.0, 50.0 note derivative term is very sensitive to noise (reduce as much as possible)
        kpr, kir, kdr = 43.64, 0.0, 12.43 #60.0, 10.0, 40.0 # 80.0, 10.0, 50.0 
        kpyaw, kiyaw, kdyaw =60.0, 10.0, 30.0 


        force = (self.g +kpz*(zd-z) + kdz*(0-vz))*(self.M)*(cos(r)*cos(p))
        Ux =  kpx*(xd-x) + kix*(xd-x)*dt - kdx*vx
        Uy = kpy*(yd-y) + kiy*(yd-y)*dt - kdy*vy 
        rd = (Ux*sin(yaw) - Uy*cos(yaw)) #*self.M/force
        pd = (Ux*cos(yaw) + Uy*sin(yaw)) #*self.M/force
        rTau = (kpr*(rd-r) + kir*(rd-r)*dt - kdr*vr)*self.Ixx
        pTau= (kpp*(pd-p) + kip*(pd-p)*dt - kdp*vp)*self.Iyy
        yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt - kdyaw*vyaw)*self.Izz


        return force, rTau, pTau, yawTau


    def FIPControllerBeta(self):
        state = self.current_pose
        xd, yd, zd = self.setpoint
        yawd = 0.0
        dt = self.dt
        x, y, z = state[0:3]
        r, p, yaw = self.quaternion_to_euler(*state[3:7])
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]
        aModel, bModel, a_dotModel, b_dotModel = self.computePenPosition()
        self.modelOutputA.append(aModel)
        self.modelOutputB.append(bModel)
        self.modelOutputAdot.append(a_dotModel)
        self.modelOutputBdot.append(b_dotModel)
        
        penState = self.currentPenPose
        a, b, eta = penState[0:3]
        a_dot, b_dot, eta_dot = penState[7:10]
       
        print(f"a:{a}, b:{b}, eta:{eta}, a_dot:{a_dot}, b_dot:{b_dot}, eta_dot:{eta_dot}")
        self.aError.append(a)
        self.bError.append(b)
        self.b_dotError.append(b_dot)
        self.a_dotError.append(a_dot)
        self.y_dotError.append(vy)
        
        wy = -1*np.array([ -78.1342,   -0.0648,   13.2000,  -11.4676,   -0.2087])@np.array([[a], [x-xd], [p], [a_dot], [vx]])
        wy = (( wy[0]))/100.0
        
        wx = -1*np.array([ 269.4669,   12.5000,   24.1688,   38.4820,    8.7607])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = (( wx[0]))/100.0
      
        wy = -1*np.array([12.6320, 125.3600,   25.0785])@np.array([[x-xd], [p], [vx]])
        wy = (( wy[0]))/100.0

        wx = -1*np.array([-12.6320, 125.3600,   -25.0785])@np.array([[y-yd], [r], [vy]])
        wx = (( wx[0]))/100.0

        
        wy = -33*np.array([-48.4589, -0.1215, 11.6000, -8.6544, -0.2966])@np.array([[a],[x-xd],[p],[a_dot], [vx]]) 
        wy = (( wy[0]))/100.0
        wx = -33*np.array([48.4589,0.1215, 11.6000,8.6544, 0.2966])@np.array([[b],[y-yd],[r],[b_dot],[vy]]) 
        wx = wx[0]/100.0

        wy = -1*np.array([-1599.1437, -4.0095, 382.8, -285.5952, -9.7878])@np.array([[a],[x-xd],[p],[a_dot], [vx]]) 
        wy = (( wy[0]))/100.0
        wx = -1*np.array([1599.1437, 4.0095, 382.8, 285.5952, 9.7878])@np.array([[b],[y-yd],[r],[b_dot],[vy]]) 
        wx = wx[0]/100.0
        
        Cf = 1.42e-6
        Ct = 2.84e-7
        l_x = 0.0865
        l_y = 0.073

        max_motor_speed = 4631.0
        maxForce = (Cf*max_motor_speed**2) 
        maxTorque = (Ct*max_motor_speed**2)
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        dt = self.dt
        force = (self.g + kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*(self.M+0.055) #*self.M
        
        throttle = 2*(force)/(maxForce) - 1
        if throttle < -1:
            throttle = -1.0
        if throttle > 1:
            throttle = 1.0
       
        wz =  -0.5*(yawd-yaw)
        print(f"force: {force}, r: {wx}, p: {wy}, yaw: {wz}")
        self.wxOutput.append(wx)
        
        u = [wx, wy, throttle, wz]
        return u

    def MPC(self):
        for j in range(self.N):
            sc = self.step_counter + j * self.skip_steps
            #yref = np.concatenate((self.traj[:, sc], [0.0, 0.0]))
            yref = np.array([0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
            self.ocp.set(j, "yref", yref)

        sn = self.step_counter + self.N * self.skip_steps
        yref_N = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #self.traj[:, sn]
        self.ocp.set(self.N, "yref", yref_N)

        
        currentState = self.current_pose
        r, p, yaw = self.quaternion_to_euler(*currentState[3:7])
        vx, vy, vz = currentState[7:10]
        rotate = R.from_euler('zyx', [yaw, p, r], degrees=False)
        rotationMatrix = rotate.as_matrix()
        [[vx], [vy], [vz]] = rotationMatrix@np.array([[vx],[vy],[vz]])
        

        state = np.concatenate((self.current_pose[0:2], [r], [p], [self.currentPenPose[1]], self.current_pose[7:9], [vx], [vy], [self.currentPenPose[8]]))
        self.ocp.set(0, "lbx", state)
        self.ocp.set(0, "ubx", state)
    
        print(f" b: {self.currentPenPose[1]}, b_dot: {self.currentPenPose[8]}")
        status = self.ocp.solve()
        if status != 0:
            raise Exception(f'acados returned status {status}.')

        u = self.ocp.get(0, "u")
        cost = self.ocp.get_cost()
        print("Optimal cost:", cost)
        #if self.enable_L1_augmentation:
        #        u[2] += self.augmented_u
        #        self.augmented_u = self.l1_controller.update(self.current_pose, u)
        #u[2] = u[2]*2 - 1

        Cf = 1.42e-6
        Ct = 2.84e-7

        max_motor_speed = 4631.0
        maxForce = (Cf*max_motor_speed**2) 
        maxTorque = (Ct*max_motor_speed**2)
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        zd, yawd  = 1.0, 0.0
        z = currentState[2]
        force = (self.g + kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*self.dt)*(self.M+0.055) #*self.M 
        throttle = 2*(force)/(maxForce) - 1
        if throttle < -1:
            throttle = -1.0
        if throttle > 1:
            throttle = 1.0
       
        wz =  -0.5*(yawd-yaw)
        u = [u[0], u[1], throttle, wz] 
        print(f"Step {self.step_counter}, Control Output: {u}")
        self.step_counter += 1
        
        return u
         
    def control_loop(self):
        msg = ELRSCommand()
        msg.armed = False
        if self.usingBetaFlight:
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = -1.0
            msg.channel_3 = 0.0
        else:
            msg.channel_0 = 0.0
            msg.channel_1 = 0.0
            msg.channel_2 = 0.0
            msg.channel_3 = 0.0
        
        self.t += self.dt

        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg.armed = True

            if self.usingBetaFlight:
                msg.channel_0 = 0.0
                msg.channel_1 = 0.0
                msg.channel_2 = -0.999
                msg.channel_3 = 0.0
            else:
                msg.channel_0 = 0.05
                msg.channel_1 = 0.05
                msg.channel_2 = 0.05
                msg.channel_3 = 0.05

            self.pre_start_counter += 1

        if self.armed and self.current_pose is not None:
            if self.step_counter + self.N * self.skip_steps > self.steps:
                self.step_counter = 0
                self.armed = False
                msg = ELRSCommand(armed=False, channel_0=0.0, channel_1=0.0, channel_2=-1.0, channel_3=0.0)
                self.cmd_publisher_.publish(msg)
                self.on_close()

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
            
            if self.testInvPen and self.usingBetaFlight:
                u1, u2, u3, u4 = self.FIPControllerBeta()

            elif not self.testMPC and not self.testInvPen:
                force, rTau, pTau, yawTau = self.navController()
            if self.testMPC:
                u1,u2,u3,u4 = self.MPC()
            
            
            
            if not self.testMPC and not self.usingBetaFlight:
                Cf = 1.42e-6
                Ct = 2.84e-7

                l_x = 0.0865
                l_y = 0.073

                max_motor_speed = 4631.0# 1755*25.2
                
                
                if force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct)< 0:
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
                    u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed


                ########################################################
                print(f"force: {force}, rTau: {rTau}, pTau: {pTau}, yawTau: {yawTau}")
                '''u1 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed
                u2 = sqrt(force/(4*Cf) - rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
                u3 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct))/max_motor_speed
                u4 = sqrt(force/(4*Cf) + rTau/(4*Cf*l_x)  - pTau/(4*Cf*l_y) + yawTau/(4*Ct))/max_motor_speed'''
            print(f"x: {x}, y: {y}, z: {z}, r: {r}, p: {p}, yaw: {yaw}, vx: {vx}, vy: {vy}")
            
            u = [u1,u2,u3,u4]
            #u = [0.0,0.0,0.0,0.0]
            '''Cf = 1.42e-6
            Ct = 2.84e-7
            l_x = 0.0865
            l_y = 0.073

            max_motor_speed = 4631.0
            maxForce = (Cf*max_motor_speed**2) 
            maxTorque = (Ct*max_motor_speed**2) 
        
            #wy = -1*np.array([12.6320, 125.3600,   25.0785])@np.array([[x-xd], [p], [vx]])
            #wy = (( wy[0]))/100.0
            #wx = -1*np.array([-12.6320,   125.3600,   -25.0785])@np.array([[y-yd], [r], [vy]]) # roll control
            #wx = (( wx[0]))/100.0

            state = self.current_pose
            xd, yd, zd = self.setpoint
            yawd = 0.0
            dt = self.dt
            x, y, z = state[0:3]
            r, p, yaw = self.quaternion_to_euler(*state[3:7])
            vx, vy, vz = state[7:10]
            vr, vp, vyaw = state[10:13]
            
            penState = self.currentPenPose
            a, b, eta = penState[0:3]
            a_dot, b_dot, eta_dot = penState[7:10]
            '''
            
            msg = ELRSCommand(armed=True, channel_0=round(u[0], 8), channel_1=round(u[1], 3), channel_2=round(u[2], 3), channel_3=round(u[3], 3))
            self.cmd_publisher_.publish(msg)
            
            
        else:         
            self.pre_start_counter = 0
            self.step_counter = 0

        self.cmd_publisher_.publish(msg)
        print(f"control output {msg.channel_0}, {msg.channel_1}, {msg.channel_2}, {msg.channel_3}")

    def computePenPosition(self):
        '''dt = self.dt
        L= self.pen_length/2
        state = self.current_pose
        
        roll, pitch, yaw = self.quaternion_to_euler(*state[3:7])
        x, y, z = state[0:3]
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]

        penState = self.currentPenPose
        a, b, eta = penState[0:3]
        a_dot, b_dot, eta_dot = penState[7:10]
        rotate = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        rotationMatrix = rotate.as_matrix()
        [[a], [b], [eta]] = rotationMatrix@np.array([[a],[b],[eta]])
        [[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])

      
        r = a
        s= b
        r_dot = a_dot
        s_dot = b_dot
        s_ddot = (b_dot - self.b_dotprev)/dt
        r_ddot = (a_dot - self.a_dotprev)/dt
        fp1 = 3*r*eta*self.g/(4*(L**2-s**2)) + ((r**3)*(s_dot**2 + s*s_ddot) - 2*r**2*s*r_dot*s_dot)/((L**2 -s**2)*eta**2) + (r*(-(L**2)*s*s_ddot + s_ddot*s**3 + (s**2)*(r_dot**2) -(L**2)*(r_dot**2)-(L**2)*(s_dot**2))/((L**2-s**2)*eta**2))
        fp2 = 3*s*eta*self.g/(4*(L**2-r**2)) + ((s**3)*(r_dot**2 + r*r_ddot) - 2*s**2*r*s_dot*r_dot)/((L**2 -r**2)*eta**2) + (s*(-(L**2)*r*r_ddot + r_ddot*r**3 + (r**2)*(s_dot**2) -(L**2)*(s_dot**2)-(L**2)*(r_dot**2))/((L**2-r**2)*eta**2))


        #zd_ddot = 4*(L**2-r**2)*(bd_ddot - fp2)*(1/(3*(s+1e-3)*eta))
        
        x_ddot, y_ddot, z_ddot = (vx-self.vx_prev)/dt, (vy-self.vy_prev)/dt, (vz-self.vz_prev)/dt
        r_ddot = x_ddot/(-4*(L**2-s**2)/(3*eta**2)) + (3*r*eta*z_ddot/(4*(L**2-s**2))) + fp1 
        s_ddot  = y_ddot/(-4*(L**2-r**2)/(3*eta**2)) + (3*s*eta*z_ddot/(4*(L**2-r**2)))  +fp2 

        r_ddot = r*self.g/L- pitch*self.g
        s_ddot = s*self.g/L+ roll*self.g
        a_dot += r_ddot*dt 
        b_dot += s_ddot*dt
        a += a_dot*dt
        b += b_dot*dt 

        self.b_dotprev = b_dot 
        self.a_dotprev = a_dot 
        self.vx_prev, self.vy_prev, self.vz_prev = vx, vy, vz
        self.x_prev, self.y_prev, self.z_prev = x, y, z
        print(f"model states | a: {a}, b: {b}, a_dot: {a_dot}, b_dot: {b_dot}")'''


        dt = self.dt
        L= self.pen_length/2
        state = self.current_pose
        
        roll, pitch, yaw = self.quaternion_to_euler(*state[3:7])
        x, y, z = state[0:3]
        vx, vy, vz = state[7:10]
        vr, vp, vyaw = state[10:13]

        '''penState = self.currentPenPose
        a, b, eta = penState[0:3]
        a_dot, b_dot, eta_dot = penState[7:10]
        rotate = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        rotationMatrix = rotate.as_matrix()
        [[a], [b], [eta]] = rotationMatrix@np.array([[a],[b],[eta]])
        [[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])'''

      
        r =self.a_prev
        s= self.b_prev
        eta = (L**2 - r**2 - s**2)**(1/2)
        r_dot = self.a_dotprev
        s_dot = self.b_dotprev
        s_ddot = self.a_ddotprev #(b_dot - self.b_dotprev)/dt
        r_ddot =self.b_ddotprev #(a_dot - self.a_dotprev)/dt
        
        g = 9.81
        '''fp1 = 3*r*eta*g/(4*(L**2-s**2)) + ((r**3)*(s_dot**2 + s*s_ddot) - 2*r**2*s*r_dot*s_dot)/((L**2 -s**2)*eta**2) + (r*(-(L**2)*s*s_ddot + s_ddot*s**3 + (s**2)*(r_dot**2) -(L**2)*(r_dot**2)-(L**2)*(s_dot**2))/((L**2-s**2)*eta**2))
        fp2 = 3*s*eta*g/(4*(L**2-r**2)) + ((s**3)*(r_dot**2 + r*r_ddot) - 2*s**2*r*s_dot*r_dot)/((L**2 -r**2)*eta**2) + (s*(-(L**2)*r*r_ddot + r_ddot*r**3 + (r**2)*(s_dot**2) -(L**2)*(s_dot**2)-(L**2)*(r_dot**2))/((L**2-r**2)*eta**2))

        
        x_ddot, y_ddot, z_ddot = (vx-self.vx_prev)/dt, (vy-self.vy_prev)/dt, (vz-self.vz_prev)/dt
        r_ddot = x_ddot/(-4*(L**2-s**2)/(3*eta**2)) + (3*r*eta*z_ddot/(4*(L**2-s**2))) + fp1 
        s_ddot  = y_ddot/(-4*(L**2-r**2)/(3*eta**2)) + (3*s*eta*z_ddot/(4*(L**2-r**2)))  +fp2'''

        g = 9.81
        x_ddot, y_ddot, z_ddot = (vx-self.vx_prev)/dt, (vy-self.vy_prev)/dt, (vz-self.vz_prev)/dt
        self.a_ddotprev = 1/((L**2 -s**2)*eta**2)*(-(r**4)*x_ddot - ((L**2 - s**2)**2)*x_ddot - (2*r**2)*(s*r_dot*s_dot + (-L**2 + s**2)*x_ddot) + (r**3)*(s_dot**2 + s*s_ddot-eta*(g+z_ddot))+r*((-L**2)*s*s_ddot + s_ddot*s**3 + (s**2)*(r_dot**2-eta*(g+z_ddot))+(L**2)*(-r_dot**2 -s_dot**2 + eta*(g + z_ddot))))
        self.b_ddotprev = 1/((L**2 -r**2)*eta**2)*(-(s**4)*y_ddot - ((L**2 - r**2)**2)*y_ddot - (2*s**2)*(r*s_dot*r_dot + (-L**2 + r**2)*y_ddot) + (s**3)*(r_dot**2 + r*r_ddot-eta*(g+z_ddot))+s*((-L**2)*r*r_ddot + r_ddot*r**3 + (r**2)*(s_dot**2-eta*(g+z_ddot))+(L**2)*(-s_dot**2 -r_dot**2 + eta*(g + z_ddot))))
        
        
       
        #r_ddot =(r*g/L - pitch*g)
        #s_ddot = (s*g/L + roll*g)
        #self.a_ddotprev = r_ddot
        #self.b_ddotprev = s_ddot
        self.a_dotprev += self.a_ddotprev*dt #r_ddot*dt 
        self.b_dotprev += self.b_ddotprev*dt #s_ddot*dt
        self.a_prev += self.a_dotprev*dt
        self.b_prev += self.b_dotprev*dt 
        self.vx_prev, self.vy_prev, self.vz_prev = vx, vy, vz
        self.x_prev, self.y_prev, self.z_prev = x, y, z

        print(f"model states | a: {self.a_prev}, b: {self.b_prev}, a_dot: {self.a_dotprev}, b_dot: {self.b_dotprev}")
        return self.a_prev, self.b_prev, self.a_dotprev, self.b_dotprev

    def plotSystemResponse(self):
        time = self.timePoints
        if self.testInvPen:
            figure1, ax1 = plt.subplots(3,1)
  
            #ax1[0].plot(time, self.aError, label="a")
            ax1[0].plot(time, self.bError, label="b")
            ax1[0].set_title('b  over time')
            ax1[0].set_ylabel('b')
            ax1[0].set_xlabel('time (s)')
            ax1[0].legend('lower right')

            ax1[1].plot(time, self.y_dotError, label="y_dot")
            ax1[1].plot(time, self.b_dotError, label="b_dot")
            ax1[1].set_title('y_dot and b_dot over time')
            ax1[1].set_ylabel('y_dot and b_dot')
            ax1[1].set_xlabel('time (s)')
            ax1[1].legend('lower right')
    
            ax1[2].plot(time, self.wxOutput)
            ax1[2].set_title('wx output over time')
            ax1[2].set_ylabel('wx output')
            ax1[2].set_xlabel('time (s)')
            plt.tight_layout()

            figure4, ax4 = plt.subplots(2,1)
            #ax4[0].plot(time, self.bError, label="actual data")
            
            #ax4[0].plot(time, self.modelOutputA, label="model data A")
            ax4[0].plot(time, self.modelOutputB, label="model data B")
            ax4[0].set_title('b model over time')
            ax4[0].set_ylabel('b')
            ax4[0].set_xlabel('time (s)')
            #ax4[0].legend('lower right')

            #ax4[1].plot(time, self.bError, label="actual data")
            #ax4[1].plot(time, self.modelOutputAdot, label="model dataA")
            ax4[1].plot(time, self.modelOutputBdot, label="model dataB")
            
            ax4[1].set_title('b_dot model over time')
            ax4[1].set_ylabel('b_dot')
            ax4[1].set_xlabel('time (s)')
            ax4[1].legend('lower right')

            '''figure6, ax6 = plt.subplots(2,1)
            ax6[0].plot(time, self.a_dotError, label="actual data")
            ax6[0].plot(time, self.modelOutputAdot, label="model data")
            ax6[0].set_title('a_dot over time')
            ax6[0].set_ylabel('a_dot')
            ax6[0].set_xlabel('time (s)')
            ax6[0].legend('lower right')

            ax6[1].plot(time, self.b_dotError, label="actual data")
            ax6[1].plot(time, self.modelOutputBdot, label="model data")
            ax6[1].set_title('b_dot over time')
            ax6[1].set_ylabel('b_dot')
            ax6[1].set_xlabel('time (s)')
            ax6[1].legend('lower right')'''
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

def takeoff_trajectory(dt):
    steps = 1 * 30  # 5 seconds of takeoff at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space)

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)

    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
   
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    vr_traj = np.zeros_like(time_space)
    vp_traj = np.zeros_like(time_space) 

    a_traj = np.zeros_like(time_space)
    b_traj = np.zeros_like(time_space)
    a_dot_traj = np.zeros_like(time_space)
    b_dot_traj = np.zeros_like(time_space)

    return np.array([x_traj, y_traj, roll_traj, pitch_traj, a_traj, b_traj,
                     vx_traj, vy_traj, vr_traj, vp_traj, a_dot_traj, b_dot_traj])
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 30  # 5 seconds of takeoff at 30 Hz
    steps_descend = 3 * 30  # 5 seconds of takeoff at 30 Hz
    time_space_move_back = np.linspace(0, steps_move_back * dt, steps_move_back)
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    time_space_total = np.concatenate((time_space_move_back, time_space_descend))

    x_traj_move_back = np.linspace(init_pose[0], 0, steps_move_back)  # Move back 1 meter
    x_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    x_traj = np.concatenate((x_traj_move_back, x_traj_descend))

    y_traj_move_back = np.linspace(init_pose[1], 0, steps_move_back)  # Move back 1 meter
    y_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    y_traj = np.concatenate((y_traj_move_back, y_traj_descend))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
   
    vr_traj = np.zeros_like(time_space_total)
    vp_traj = np.zeros_like(time_space_total)
    

    a_traj = np.zeros_like(time_space_total)
    b_traj = np.zeros_like(time_space_total)
    a_dot_traj = np.zeros_like(time_space_total)
    b_dot_traj = np.zeros_like(time_space_total)
 
    return np.array([x_traj, y_traj, roll_traj, pitch_traj,  a_traj, b_traj,
                     vx_traj, vy_traj, vr_traj, vp_traj, a_dot_traj, b_dot_traj])


def move_to_start_of_main_trajectory(dt, init_pose, final_pose):
    steps = 3 * 30  # 3 seconds at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)

    x_traj = np.linspace(init_pose[0], final_pose[0], steps)
    y_traj = np.linspace(init_pose[1], final_pose[1], steps)
    
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
   
    vr_traj = np.zeros_like(time_space)
    vp_traj = np.zeros_like(time_space)
 
    a_traj = np.zeros_like(time_space)
    b_traj = np.zeros_like(time_space)
    a_dot_traj = np.zeros_like(time_space)
    b_dot_traj = np.zeros_like(time_space)

    return np.array([x_traj, y_traj, roll_traj, pitch_traj, a_traj, b_traj,
                     vx_traj, vy_traj, vr_traj, vp_traj, a_dot_traj, b_dot_traj])





def hover_trajectory(dt):

        take_off_traj = takeoff_trajectory(dt)


        steps = 10 * 30  # 10 seconds of hover at 30 Hz
        time_space = np.linspace(0, steps * dt, steps)
        x_traj = np.zeros_like(time_space)
        y_traj = np.zeros_like(time_space)
        

        roll_traj = np.zeros_like(time_space)
        pitch_traj = np.zeros_like(time_space)
        
        vx_traj = np.gradient(x_traj, dt)
        vy_traj = np.gradient(y_traj, dt)

        vr_traj = np.zeros_like(time_space)
        vp_traj = np.zeros_like(time_space)

        a_traj = np.zeros_like(time_space)
        b_traj = np.zeros_like(time_space)
        a_dot_traj = np.zeros_like(time_space)
        b_dot_traj = np.zeros_like(time_space)

        hover_traj =  np.array([x_traj, y_traj, roll_traj, pitch_traj, a_traj, b_traj,
                     vx_traj, vy_traj, vr_traj, vp_traj, a_dot_traj, b_dot_traj])


        land_traj = land_trajectory(dt, take_off_traj[:, -1])
        zeros = np.zeros((take_off_traj.shape[0], 1 * 30))


        return np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)


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
