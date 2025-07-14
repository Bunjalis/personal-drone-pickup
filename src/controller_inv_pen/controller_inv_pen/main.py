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
        self.setpoint = np.array([0.0, 0.0, 0.4])
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
        self.b_dotError = []
        self.y_dotError = []
        self.wxOutput = []
        self.timePoints = []
        self.t = 0

        self.testInvPen = False 
        self.testMPC = True
        self.usingBetaFlight = False
        self.pen_length = 0.6
        self.pen_mass =  0.000001
        self.a = 0.0
        self.b = 0.0
        self.a_dot = 0.0
        self.b_dot = 0.0
        self.a_ddot = 0.0
        self.b_ddot = 0.0
        self.eta = math.sqrt(self.pen_length**2 - self.a**2 -self.b**2)
        self.bErrorSum = 0
        self.prev_bError = 0
        self.prev_rollError = 0

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
        self.steps = 90 * 30
        #self.dt = 1.0 / 30.0
        self.step_counter = 0
        self.timer = self.create_timer(self.dt, self.control_loop)

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

        self.pre_start_duration = 1.0  # Duration for the pre-start state in seconds
        self.pre_start_counter = 0  # Counter to track pre-start steps
        self.pre_start_steps = int(self.pre_start_duration / self.dt)  # Steps for pre-start state

        self.N = 20

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
        #print(f"a:{a}, b:{b}, eta:{eta}, a_dot:{a_dot}, b_dot:{b_dot}, eta_dot:{eta_dot}")
       
        
        #print(f"penAngle: {self.currentPenPose}")

    

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


        return force, rTau, pTau, yawTau

    def FIPController(self):
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
        rotate = R.from_euler('zyx', [yaw, p, r], degrees=False)
        rotationMatrix = rotate.as_matrix()
        print(rotationMatrix)
        [[a], [b], [eta]] = rotationMatrix@np.array([[a],[b],[eta]])
        [[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])
        #print(position)
        print(f"a:{a}, b:{b}, eta:{eta}, a_dot:{a_dot}, b_dot:{b_dot}, eta_dot:{eta_dot}")
        print(f"x:{x}, y:{y}, z:{z}, x_dot:{vx}, y_dot:{vy}, z_dot:{vz}")
        self.aError.append(a)
        self.bError.append(b)
        self.b_dotError.append(b_dot)
        self.y_dotError.append(vy)
        
         
        #pTau = -1*np.array([-0.1320,0.9669,-10.0866,-0.1314,0.0546,-1.2473])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        #rTau = -1*np.array([0.1645, 1.2046, 12.5655 , 0.1638, 0.0680, 1.5538])@np.array([[y-yd],[r],[b],[vy],[vr], [b_dot]])
        
        
       
       
        '''pTau = -1*K1@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*K2@np.array([[y-yd],[r],[b],[vy],[vr], [b_dot]])
        pTau = -1*np.array([ -0.0032,0.2349,-0.9595,-0.0064,0.0252,-0.1675])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([ 0.0040,0.2926,1.1953,0.0079, 0.0314,0.2086])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])'''
        #a = 0
        #a_dot = 0
        
        
        

        #rTau = -1*np.array([0.0064,    0.3333,   -1.4141,   0.0118,    0.0340,   -0.2471])@np.array([[y-yd],[r],[b],[vy],[vr], [b_dot]])
        '''kpr, kir, kdr = 80.0, 10.0, 50.0
        rTau = (kpr*(rd-r) + kir*(rd-r)*dt - kdr*vr)*self.Ixx
        pTau = -1*np.array([0.0034,0.0490,0.0071,0.0140])@np.array([[x-xd],[p],[vx],[vp]]) '''
        
        '''vr, vp, vyaw = state[10:13]
        kpz, kiz, kdz = 15.0, 10.0, 10.0 #75.0, 42.857, 32.8125
        kpx, kix, kdx = 6.0, 0, 12.0 #0.6, 0, 1.2#0.5, 0,0.4
        kpy, kiy, kdy = 6.0, 0, 12.0 #0.6, 0, 1.2#0.36, 0, 0.45

        kpp, kip, kdp = 80.0, 10.0, 50.0 #6.0, 1.5, 1.75 #5.0, 3.0, 3.0 # 2.4, 0.48, 3.09 #
        kpr, kir, kdr = 80.0, 10.0, 50.0 #6.0, 1.5, 1.75 # 2.1, 0.84, 1.3125 #
        kpyaw, kiyaw, kdyaw = 80.0, 10.0, 50.0 #6.0, 1.5, 1.75 # #0.804, 0.29236, 0.55275
        force = (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt )*(self.M + self.pen_mass)*(cos(r)*cos(p))
        K1= np.array([-4.73384185e+01, -2.23606798e-02,  1.26778474e+01, -8.27830464e+00, -8.62906201e-02])
        K2= np.array([4.73384185e+01, 2.23606798e-02, 1.26778474e+01, 8.27830464e+00, 8.62906201e-02])
        vpd = -K1@np.array([[a],[x-xd],[p],[a_dot],[vx]]) #-1*np.array([-0.0341,0.6168, -5.4842,-0.0409,0.0420,-0.6779])@np.array([[x-xd],[pitch],[a],[vx],[vp],[a_dot]]) 
        vrd = -K2@np.array([[b],[y-yd],[r],[b_dot],[vy]])#-1*np.array([0.0020,0.4194,3.0870,0.0048,0.0366,0.3798])@np.array([[y-yd],[roll],[b],[vy],[vr],[b_dot]])
        vpd = math.atan(vpd[0])
        vrd = math.atan(vrd[0])
        self.rd += vrd*dt
        self.pd += vpd*dt
        rd = self.rd
        pd = self.pd'''

        '''pd = -np.array([-24.3912,   -2.0574,   -4.9252,   -1.7571])@np.array([[a],[x-xd],[a_dot],[vx]]) # vrd*dt
        pd = math.atan(pd[0])
        rd = -np.array([24.3912,   2.0574,   4.9252,   1.7571])@np.array([[a],[x-xd],[a_dot],[vx]]) #vpd*dt
        rd = math.atan(rd[0])'''

        '''ad_ddot = 3/4*self.g*(1/0.3*a-p)*20
        Ux =  kpx*(xd-x) + kix*(xd-x)*dt - kdx*vx #+ ad_ddot#+ xd_dotdot
        Uy = kpy*(yd-y) + kiy*(yd-y)*dt - kdy*vy #+ yd_dotdot
        
        rd =8*((-5*b) + (-b_dot)) # (Ux*sin(yaw) - Uy*cos(yaw))*(self.M+self.pen_mass)/force # =2*((-5*b) + (-b_dot))
        pd = (Ux*cos(yaw) + Uy*sin(yaw))*(self.M+self.pen_mass)/force'''
        
        #rTau = (kpr*(rd-r) + kir*(rd-r)*dt + kdr*(vrd-vr))*self.Ixx
        #pTau= (kpp*(pd-p) + kip*(pd-p)*dt + kdp*(vpd-vp))*self.Iyy
        #pTau = -1*np.array([ -0.0032,0.2349,-0.9595,-0.0064,0.0252,-0.1675])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        #rTau = -1*np.array([0.0085,    0.3191,    1.4704,    0.0157,    0.0340,    0.2969])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])
        
        kpyaw, kiyaw, kdyaw = 60.0, 10.0, 30.0 #6.0, 1.5, 1.75 # #0.804, 0.29236, 0.55275
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        force = (self.g +kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt )*(self.M + self.pen_mass)*(cos(r)*cos(p))
        yawTau = (kpyaw*(yawd-yaw) + kiyaw*(yawd-yaw)*dt - kdyaw*vyaw)*self.Izz
        pTau = -1*np.array([-0.0034, 1.9617,   -7.2211,   -0.0134,    0.2896,   -1.4960])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([0.0043,    2.4439,    8.9958,    0.0167,    0.3607,    1.8637])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])
       
        pTau = -1*np.array([ -0.0032,0.2349,-0.9595,-0.0064,0.0252,-0.1675])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([ 0.0040,0.2926,1.1953,0.0079, 0.0314,0.2086])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])

        pTau = -1*np.array([-0.0004,   0.1719,   -0.6198,   -0.0011,    0.0238,   -0.1134])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([0.0137,2.1804,9.1509,0.0460, 0.0387,1.2022])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])
        pTau = pTau[0]
        rTau = rTau[0]
    

        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        kpx, kix, kdx = 0.06938, 0.0, 0.14488#0.006, 0.0, 0.0 #6.0, 0, 12.0 
        kpy, kiy, kdy = 0.07, 0.0, 0.1456
        kpp, kip, kdp = 28.8235, 0.0, 8.235 #90.0, 10.0, 20.0 #30.0 # 80.0, 10.0, 50.0 note derivative term is very sensitive to noise (reduce as much as possible)
        kpr, kir, kdr = 43.64, 0.0, 12.43 #60.0, 10.0, 40.0 # 80.0, 10.0, 50.0 
        #kpr, kir, kdr = 23.64, 0.0, 5.43
        kpyaw, kiyaw, kdyaw =60.0, 10.0, 30.0 

        Ux =  kpx*(xd-x) + kix*(xd-x)*dt - kdx*vx
        Uy = kpy*(yd-y) + kiy*(yd-y)*dt - kdy*vy 

        rd = (Ux*sin(yaw) - Uy*cos(yaw)) #*self.M/force
        rd = -1*np.array([10.8967,    0.0608,    1.8794,    0.1465])@np.array([[b], [y-yd], [b_dot], [vy]])
        rd = rd[0]
        pd = (Ux*cos(yaw) + Uy*sin(yaw)) #*self.M/force
        #pd = -1*np.array([-37.4179,    -0.2181,   -11.6594,    -0.6523])@np.array([[a], [x-xd], [a_dot], [vx]])
        #pd = pd[0]
        self.rollError.append(self.prev_rollError-r)
        self.prev_rollError = rd
        #pd = 0
        #rd = 0
        rTau = (kpr*(rd-r) + kir*(rd-r)*dt - kdr*vr)*self.Ixx
        pTau= (kpp*(pd-p) + kip*(pd-p)*dt - kdp*vp)*self.Iyy
        #self.prevForce = force
        #wy = -1*np.array([-210.9648, -7.8557, 25.0000, -36.8930, -8.5851])@np.array([[a],[x-xd],[pitch],[a_dot], [vx]])  
        #wx = -1*np.array([48.4589,0.1215, 11.6000,8.6544, 0.2966])@np.array([[b],[y-yd],[roll],[b_dot],[vy]])
        #pTau = -1*np.array([ -0.0010,    0.2708,   -1.0214,   -0.0030,    0.0332,   -0.1856])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        rTau = -1*np.array([ 0.0180,    0.3548,    1.6279,    0.0222,    0.0352,    0.2842])@np.array([[y-yd],[r],[b],[vy],[vr],[b_dot]])
        pTau = -1*np.array([   -0.0180,    0.5027,   -2.9400,   -0.0373,    0.0375,   -0.4895])@np.array([[x-xd],[p],[a],[vx],[vp],[a_dot]]) 
        pTau = pTau[0]
        rTau=rTau[0]
        self.wxOutput.append(rTau)
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
        
        penState = self.currentPenPose
        a, b, eta = penState[0:3]
        a_dot, b_dot, eta_dot = penState[7:10]
        print(f"a:{a}, b:{b}, eta:{eta}, a_dot:{a_dot}, b_dot:{b_dot}, eta_dot:{eta_dot}")
        #print(f"x:{x}, y:{y}, z:{z}, x_dot:{vx}, y_dot:{vy}, z_dot:{vz}")
        self.aError.append(a)
        self.bError.append(b)
        self.b_dotError.append(b_dot)
        self.y_dotError.append(vy)
        #wy = -1*np.array([-664.5726,   -0.8313,  106.0000,  -69.9895,   -2.5022])@np.array([[a], [x-xd], [p], [a_dot], [vx]])
        #wy = (( wy[0]))/100.0
        #wx = -1000*np.array([ 3.050,    0.014,    0.40,    0.441,    0.0912])@np.array([[b], [y-yd], [r], [b_dot], [vy]]) # roll control
        #wx = -1000*np.array([3.250,    0.014,    0.0,    0.05,    0.0912])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        #wx = -1000*np.array([7.0,    0.0,    0.125,    0.5,    0.0])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = -1000*np.array([7.624,    0.001,    0.1,    0.672,    0.023])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = -1000*np.array([1.824,    0.001,    0.1,    0.772,    0.023])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = -1000*np.array([2.724,    0.001,    0.3,    1.372,    0.023])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        #wx = -1*np.array([1871.4617,    0.0125,   125.5217,  512.2909,    0.2571])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = -1000*np.array([4.2515,    0.0400,    0.3023,    1.5277,    0.1076])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        wx = (( wx[0]))/100.0

        rd = -1*np.array([33.0, 0.10104,    6.5,    0.200628])@np.array([[b], [y-yd], [b_dot], [vy]])
        wx = -125.36*(r-rd[0])/100.0
        #wx = -1*np.array([269.1772,   12.5000,   104.1573,   38.4470,    8.7540])@np.array([[b], [y-yd], [r], [b_dot], [vy]])
        #wx = (( wx[0]))/100.0
        #wx = -1*np.array([48.4589,0.1215, 11.6000,8.6544, 0.2966])@np.array([[b],[y-yd],[r],[b_dot],[vy]])
        #wx = (( wx[0]))/100.0
        wy = -1*np.array([12.6320, 125.3600,   25.0785])@np.array([[x-xd], [p], [vx]])
        wy = (( wy[0]))/100.0
        #wx = -1*np.array([-12.6320,   125.3600,   -25.0785])@np.array([[y-yd], [r], [vy]]) # roll control
        #wx = (( wx[0]))/100.0



        
        Cf = 1.42e-6
        Ct = 2.84e-7
        l_x = 0.0865
        l_y = 0.073

        max_motor_speed = 4631.0
        maxForce = (Cf*max_motor_speed**2) 
        maxTorque = (Ct*max_motor_speed**2)
        kpz, kiz, kdz = 15.0, 10.0, 10.0 
        dt = self.dt
        force = (self.g + kpz*(zd-z) + kdz*(0-vz) +kiz*(zd-z)*dt)*self.M
        
        throttle = 2*(force)/(maxForce) - 1
        if throttle < -1:
            throttle = -1.0
        if throttle > 1:
            throttle = 1.0
        #sumYawError = sum(self.yawError)
        wz =  -0.5*(yawd-yaw)
        print(f"force: {force}, r: {wx}, p: {wy}, yaw: {wz}")
        self.wxOutput.append(wx)
        u = [wx, wy, throttle, wz]
        return u

    def MPC(self):
        skip_steps = 3
        
        for j in range(self.N):
            if self.step_counter + j*skip_steps < self.steps:
                yref = np.array([self.x_traj[self.step_counter + j*skip_steps], self.y_traj[self.step_counter + j*skip_steps],
                                    self.z_traj[self.step_counter + j*skip_steps], self.qw_traj[self.step_counter + j*skip_steps],
                                    self.qx_traj[self.step_counter + j*skip_steps], self.qy_traj[self.step_counter + j*skip_steps],
                                    self.qz_traj[self.step_counter + j*skip_steps], 
                                    self.vx_traj[self.step_counter + j*skip_steps], self.vy_traj[self.step_counter + j*skip_steps], self.vz_traj[self.step_counter + j*skip_steps], 
                                    self.ax_traj[self.step_counter + j*skip_steps], self.ay_traj[self.step_counter + j*skip_steps], self.az_traj[self.step_counter + j*skip_steps], 
                                    0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0, 0.2, 0.2, 0.2, 0.2])
            else:
                yref = np.array([self.x_traj[-1], self.y_traj[-1], self.z_traj[-1], 1, 0, 0, 0, 0,0, 0, 0,0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0,0.0, 0.2, 0.2, 0.2, 0.2])
            self.ocp.set(j, "yref", yref)


        yref_N = np.array([self.x_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.y_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.z_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],
                                self.qw_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.qx_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.qy_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.qz_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],
                                self.vx_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.vy_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.vz_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], 
                                self.ax_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)], self.ay_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],  self.az_traj[min(self.step_counter + self.N*skip_steps, self.steps - 1)],
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ])
        self.ocp.set(self.N, "yref", yref_N)

        # merge current_pose with currentPenPosepenState = self.currentPenPose
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
        rotate = R.from_euler('zyx', [yaw, p, r], degrees=False)
        rotationMatrix = rotate.as_matrix()
        #print(rotationMatrix)
        [[a], [b], [eta]] = rotationMatrix@np.array([[a],[b], [eta]])
        [[a_dot], [b_dot], [eta_dot]] = rotationMatrix@np.array([[a_dot], [b_dot], [eta_dot]])
        penPose = np.array([a,a_dot,b, b_dot])
        current_state_with_omega = np.concatenate((self.current_pose,penPose, self.omega_est)) #add pen a,b,a_dot and b_dot  to current pose
        

        self.ocp.set(0, "lbx", current_state_with_omega)
        self.ocp.set(0, "ubx", current_state_with_omega)

        # Solve the OCP
        status = self.ocp.solve()
        print(f"STATUS: {status}")
        if status != 0:
            raise Exception(f'acados returned status {status}.')

        # Retrieve the control inputs
        u = self.ocp.get(0, "u")
        '''msg.armed = True
        msg.channel_0 = round(u[0], 3)
        msg.channel_1 = round(u[1], 3)
        msg.channel_2 = round(u[2], 3)
        msg.channel_3 = round(u[3], 3)'''
        
        #self.cmd_publisher_.publish(msg)
        self.omega_est = self.ocp.get(1,"x")[-4:]
        print(f"step: {self.step_counter}")
        self.step_counter += 1
        
        print(f"pendulum: {penPose}")
        print(f"U = {np.round(u, 3)} omega_est = {np.round(self.omega_est, 3)}")
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
            if self.testInvPen and not self.usingBetaFlight:
                force, rTau, pTau, yawTau = self.FIPController()

            if self.testInvPen and self.usingBetaFlight:
                u1, u2, u3, u4 = self.FIPControllerBeta()

            elif not self.testMPC and not self.testInvPen:
                force, rTau, pTau, yawTau = self.navController()
            else:
                u1,u2,u3,u4 = self.MPC()
            
            
            
            if not self.testMPC and not self.usingBetaFlight:
                Cf = 1.42e-6
                Ct = 2.84e-7

                l_x = 0.0865
                l_y = 0.073

                max_motor_speed = 4631.0# 1755*25.2
                
                
                if force/(4*Cf) - rTau/(4*Cf*l_x)  + pTau/(4*Cf*l_y) - yawTau/(4*Ct)< 0:
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
            figure1, ax1 = plt.subplots(3,1)
            ax1[0].plot(time, self.aError)
            ax1[0].set_title('a position error over time')
            ax1[0].set_ylabel('a position error')
            ax1[0].set_xlabel('time (s)')

            ax1[1].plot(time, self.bError)
            ax1[1].set_title('b position error over time')
            ax1[1].set_ylabel('b position error')
            ax1[1].set_xlabel('time (s)')

            ax1[2].plot(time, self.wxOutput)
            ax1[2].set_title('wx output over time')
            ax1[2].set_ylabel('wx output')
            ax1[2].set_xlabel('time (s)')
            plt.tight_layout()

            figure3, ax3 = plt.subplots(2,1)
            ax3[0].plot(time, self.b_dotError)
            ax3[0].set_title('b_dot error over time')
            ax3[0].set_ylabel('b_dot error')
            ax3[0].set_xlabel('time (s)')

            ax3[1].plot(time, self.y_dotError)
            ax3[1].set_title('y_dot error over time')
            ax3[1].set_ylabel('y_dot error')
            ax3[1].set_xlabel('time (s)')

            
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
