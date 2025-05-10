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

        # Update full state vector to include motor speeds
        self.omega = cs.MX.sym('omega', 4)  # Motor speeds
        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.omega)
        self.state_dim = 17  # Updated state dimension to include motor speeds

        # Control input vector (throttle, roll, pitch, yaw)
        m1 = cs.MX.sym('m1') # back right, counter-clockwise
        m2 = cs.MX.sym('m2') # front right, clockwise
        m3 = cs.MX.sym('m3') # back left, clockwise
        m4 = cs.MX.sym('m4') # front left, counter-clockwise
        self.u = cs.vertcat(m1, m2, m3, m4)

        ''' simulated quadcopter parameters 
        self.mass = 1.04
        self.x_l = 0.15
        self.y_l = 0.15
        self.J = np.array([.03, .03, .06])
        self.motor_constant = 8.54858e-6
        self.moment_constant = 0.016 
        self.max_speed = 1000  # rad/s
        '''

        #''' Simulated tiny trainer parameters
        self.mass = 0.2
        self.x_l = 0.054
        self.y_l = 0.046
        #self.J = np.array([.03, .03, .06])
        self.J = np.array([0.0004124292645, 0.0003459416836, 0.0005984955024])
        self.motor_constant = 1.326e-07
        self.moment_constant = 0.3
        self.max_speed = 6000  # rad/s
        #'''
        

        self.x_f = np.array([-self.y_l, -self.y_l, self.y_l, self.y_l])
        self.y_f = np.array([self.x_l, -self.x_l, self.x_l, -self.x_l])
        self.z_l_tau = np.array([-1, 1, 1, -1])

        # Motor dynamics parameters
        self.tau_motor = 0.114  # Time constant for motor dynamics
        self.K_motor = 1.053    # Gain for motor dynamics

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

    def motor_dynamics(self):
        # Define the motor dynamics as a first-order system
        omega_dot = (1 / self.tau_motor) * (-self.omega + self.K_motor * self.u)
        return omega_dot

    def quad_dynamics(self):
        # Include motor dynamics in the overall dynamics
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics(), self.motor_dynamics())
        return cs.Function('x_dot', [self.x, self.u], [x_dot], ['x', 'u'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        # Update thrust model to use the new equation
        #f_thrust = 7.46e-08 * cs.power(self.omega * self.max_speed, 2) + 1.51e-04 * (self.omega * self.max_speed)
        f_thrust = self.motor_constant * cs.power(self.omega * self.max_speed, 2)

        # Gravity vector
        g = cs.vertcat(0.0, 0.0, 9.81)

        # Total thrust in the body z-direction
        a_thrust = cs.vertcat(0.0, 0.0, f_thrust[0] + f_thrust[1] + f_thrust[2] + f_thrust[3]) / self.mass

        # Rotate thrust to the world frame and subtract gravity
        v_dynamics = self.v_dot_q(a_thrust, self.q) - g

        return v_dynamics

    def w_dynamics(self):
        # Use motor speeds (omega) instead of control inputs (u) directly
        #f_thrust = 7.46e-08 * cs.power(self.omega * self.max_speed, 2) + 1.51e-04 * (self.omega * self.max_speed) # Thrust for each motor using omega
        f_thrust = self.motor_constant * cs.power(self.omega * self.max_speed, 2)
        tau_yaw = self.moment_constant * f_thrust  # Torque for each motor (yaw)

        # Convert parameters to CasADi symbolic variables
        x_f = cs.MX(self.x_f)  # x-offsets of motors for roll dynamics
        y_f = cs.MX(self.y_f)  # y-offsets of motors for pitch dynamics
        c_f = cs.MX(self.z_l_tau)  # yaw torque coefficients

        # Calculate angular velocity dynamics
        w_dynamics = cs.vertcat(
            (cs.mtimes(f_thrust.T, x_f) + (self.J[1] - self.J[2]) * self.r[1] * self.r[2]) / self.J[0],  # Roll dynamics
            (cs.mtimes(f_thrust.T, y_f) + (self.J[2] - self.J[0]) * self.r[2] * self.r[0]) / self.J[1],  # Pitch dynamics
            (cs.mtimes(tau_yaw.T, c_f) + (self.J[0] - self.J[1]) * self.r[0] * self.r[1]) / self.J[2]   # Yaw dynamics
        )

        return w_dynamics
