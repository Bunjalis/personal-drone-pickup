import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


class QuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 3)  # position
        self.q = cs.MX.sym('a', 4)  # angle quaternion (wxyz)
        self.v = cs.MX.sym('v', 3)  # velocity
        self.r = cs.MX.sym('r', 3)  # angular velocity
        
        # Actuator states (actual motor control values)
        self.actuators = cs.MX.sym('actuators', 8)  # 8 actual actuator states
        
        # Desired actuator states (what we want the actuators to be)
        self.u_desired = cs.MX.sym('u_desired', 8)  # 8 desired actuator states

        # State vector: position, quaternion, velocity, angular velocity, actual actuators, desired actuators
        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.actuators, self.u_desired)
        self.state_dim = 29  # 13 + 8 actual actuators + 8 desired actuators

        # Control input: rate of change of desired actuator values (d(u_desired)/dt)
        u_dot0 = cs.MX.sym('u_dot0')  # rate for desired actuator 0
        u_dot1 = cs.MX.sym('u_dot1')  # rate for desired actuator 1
        u_dot2 = cs.MX.sym('u_dot2')  # rate for desired actuator 2
        u_dot3 = cs.MX.sym('u_dot3')  # rate for desired actuator 3
        u_dot4 = cs.MX.sym('u_dot4')  # rate for desired actuator 4
        u_dot5 = cs.MX.sym('u_dot5')  # rate for desired actuator 5
        u_dot6 = cs.MX.sym('u_dot6')  # rate for desired actuator 6
        u_dot7 = cs.MX.sym('u_dot7')  # rate for desired actuator 7
        
        self.u_dot = cs.vertcat(u_dot0, u_dot1, u_dot2, u_dot3, u_dot4, u_dot5, u_dot6, u_dot7)
        
        # Actuator time constant for first-order dynamics
        self.actuator_time_constant = 0.083

        self.mass = 1.1

        self.J = np.array([0.015, 0.015, 0.015])
        self.max_rpm = 4631.0
        self.thrust_constant = 1.2e-06
        self.moment_constant = 0.1

        self.motor_moment_directions = np.array([1, -1, -1, 1, 1, -1, -1, 1])  # Direction of each motor's moment


        self.mot_pos_vec = 0.12 * np.array([[1, -1, 1],
                                    [-1, -1, 1], 
                                    [1, -1, -1],
                                    [-1, -1, -1],
                                    [1, 1, -1],
                                    [-1, 1, -1],
                                    [1, 1, 1],
                                    [-1, 1, 1]])

        self.mot_rot_vec = np.array([[-0.211325,-0.788675,  -0.57735],  
                                    [0.788675, -0.211325 ,  0.57735],       
                                    [0.211325, 0.788675, -0.57735],    
                                    [-0.788675,  0.211325,   0.57735],     
                                    [0.788675, -0.211325, 0.57735],        
                                    [-0.211325, -0.788675,  -0.57735],     
                                    [-0.788675,  0.211325,   0.57735],       
                                    [0.211325, 0.788675, -0.57735]])     



    
    def q_to_rot_mat(self, q):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        if isinstance(q, np.ndarray):
            rot_mat = np.array([
                [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]])

        else:
            rot_mat = cs.vertcat(
                cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
                cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
                cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

        return rot_mat

    def v_dot_q(self, v, q):
        rot_mat = self.q_to_rot_mat(q)
        if isinstance(q, np.ndarray):
            return rot_mat.dot(v)

        return cs.mtimes(rot_mat, v)

    def skew_symmetric(self, v):
        if isinstance(v, np.ndarray):
            return np.array([[0, -v[0], -v[1], -v[2]],
                             [v[0], 0, v[2], -v[1]],
                             [v[1], -v[2], 0, v[0]],
                             [v[2], v[1], -v[0], 0]])

        return cs.vertcat(
            cs.horzcat(0, -v[0], -v[1], -v[2]),
            cs.horzcat(v[0], 0, v[2], -v[1]),
            cs.horzcat(v[1], -v[2], 0, v[0]),
            cs.horzcat(v[2], v[1], -v[0], 0))

    def quad_dynamics(self):
        x_dot = cs.vertcat(
            self.p_dynamics(), 
            self.q_dynamics(), 
            self.v_dynamics(), 
            self.w_dynamics(),
            self.actuator_dynamics(),
            self.u_dynamics()
        )
        return cs.Function('x_dot', [self.x, self.u_dot], [x_dot], ['x', 'u_dot'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):

        # Use actuator states instead of direct control inputs
        motor_thrusts = cs.sign(self.actuators) * self.thrust_constant * cs.power(self.max_rpm * cs.fabs(self.actuators), 2)
        f_thrust = cs.mtimes(self.mot_rot_vec.T, motor_thrusts)
        
        g = cs.vertcat(0.0, 0.0, 9.81)
        v_dynamics = self.v_dot_q(f_thrust, self.q) / self.mass - g

        return v_dynamics

    def w_dynamics(self):

        # Calculate thrust forces for each motor using actuator states
        # T = sign(actuator) * k * (max_rpm * |actuator|)^2
        thrusts = cs.sign(self.actuators) * self.thrust_constant * cs.power(self.max_rpm * cs.fabs(self.actuators), 2)

        # Compute torques generated by each motor (thrust-induced torques)
        thrust_torque = cs.MX.zeros(3)
        for i in range(8):
            thrust_torque += thrusts[i] * cs.cross(self.mot_pos_vec[i], self.mot_rot_vec[i])

        # Compute motor reaction torques (drag/moment torques from spinning motors)
        # Following Gazebo's implementation: dragTorque = (0, 0, -turningDirection * thrust * momentConstant)
        # The drag torque acts in the local Z-axis of each motor, then gets transformed to body frame

        motor_moments = cs.MX.zeros(3)
        for i in range(8):
            # Calculate thrust for this motor
            thrust_i = cs.sign(self.actuators[i]) * self.thrust_constant * cs.power(self.max_rpm * cs.fabs(self.actuators[i]), 2)
            
            # Gazebo's drag torque in motor local frame: (0, 0, -turningDirection * thrust * momentConstant)
            # The local Z-axis is along the thrust direction (mot_rot_vec[i])
            drag_torque_magnitude = self.motor_moment_directions[i] * thrust_i * self.moment_constant
            
            # Apply the drag torque along the motor's thrust vector (rotation axis)
            # This represents the reaction torque from propeller drag opposing rotation
            motor_moments += drag_torque_magnitude * self.mot_rot_vec[i]


        # Total torque = thrust-induced torques + motor reaction torques
        total_torque = thrust_torque #+ motor_moments

        # Compute angular acceleration in the world frame
        J_inv = cs.diag(1 / self.J)  # Inverse of inertia matrix
        angular_acceleration = cs.mtimes(J_inv, total_torque - cs.cross(self.r, cs.mtimes(cs.diag(self.J), self.r)))

        return angular_acceleration
    
    def actuator_dynamics(self):
        """
        First-order actuator dynamics: d(actuator)/dt = (1/tau) * (u_desired - actuator_current)
        where tau is the time constant and u_desired is the desired actuator state
        """
        return (1.0 / self.actuator_time_constant) * (self.u_desired - self.actuators)
    
    def u_dynamics(self):
        """
        Desired actuator dynamics: d(u_desired)/dt = u_dot
        where u_dot is the control input (rate of change of desired actuator states)
        """
        return self.u_dot