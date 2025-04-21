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

        # Full state vector (13-dimensional)
        self.x = cs.vertcat(self.p, self.q, self.v, self.r)
        self.state_dim = 13

        # Control input vector (throttle, roll, pitch, yaw)
        m1 = cs.MX.sym('m1') # back right, counter-clockwise
        m2 = cs.MX.sym('m2') # front right, clockwise
        m3 = cs.MX.sym('m3') # back left, clockwise
        m4 = cs.MX.sym('m4') # front left, counter-clockwise
        self.u = cs.vertcat(m1, m2, m3, m4)

        # Additional parameters
        self.max_thrust = 0.5  # N
        self.mass = 0.2

        self.c = 0.013 
        self.length = 0.1/2

        h = np.cos(np.pi / 4) * self.length
        self.x_f = np.array([-h, -h, h, h])
        self.y_f = np.array([h, -h, h, -h])
        self.z_l_tau = np.array([self.c, -self.c, -self.c, self.c])

        self.J = np.array([.03, .03, .06])


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
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics())
        return cs.Function('x_dot', [self.x, self.u], [x_dot], ['x', 'u'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        f_thrust = self.u * self.max_thrust
        g = cs.vertcat(0.0, 0.0, 9.81)
        a_thrust = cs.vertcat(0.0, 0.0, f_thrust[0] + f_thrust[1] + f_thrust[2] + f_thrust[3]) / self.mass
        v_dynamics = self.v_dot_q(a_thrust, self.q) - g

        return v_dynamics

    def w_dynamics(self):

        f_thrust = self.u * self.max_thrust

        y_f = cs.MX(self.y_f)
        x_f = cs.MX(self.x_f)
        c_f = cs.MX(self.z_l_tau)

        w_dynamics = cs.vertcat(
            (cs.mtimes(f_thrust.T, y_f) + (self.J[1] - self.J[2]) * self.r[1] * self.r[2]) / self.J[0],
            (-cs.mtimes(f_thrust.T, x_f) + (self.J[2] - self.J[0]) * self.r[2] * self.r[0]) / self.J[1],
            (cs.mtimes(f_thrust.T, c_f) + (self.J[0] - self.J[1]) * self.r[0] * self.r[1]) / self.J[2])

        return w_dynamics
