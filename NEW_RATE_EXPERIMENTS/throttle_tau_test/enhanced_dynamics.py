import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


class EnhancedQuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 3)
        self.q = cs.MX.sym('a', 4)
        self.v = cs.MX.sym('v', 3) 
        self.r = cs.MX.sym('r', 3) 
        self.u = cs.MX.sym('u', 4)

        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.u)
        self.state_dim = 17

        self.u_dot = cs.MX.sym('u_dot', 4)

        # Original parameters
        self.thrust_ratio = cs.MX.sym('kT', 1)
        self.tau_rate = cs.MX.sym('tau_rate', 1)
        self.tau_throttle = cs.MX.sym('tau_throttle', 1)
        self.centre_rate_deg = cs.MX.sym('centre_rate_deg', 1)
        self.max_rate_deg = cs.MX.sym('max_rate_deg', 1)
        self.rate_expo = cs.MX.sym('rate_expo', 1)
        
        # Enhanced parameters for better Z-axis modeling
        self.drag_coeff_z = cs.MX.sym('drag_z', 1)           # Vertical drag coefficient
        self.thrust_offset = cs.MX.sym('thrust_offset', 1)    # Thrust offset/bias
        self.tau_thrust = cs.MX.sym('tau_thrust', 1)          # Thrust actuator time constant
        
        # Combined parameter vector (10 parameters total)
        self.p_param = cs.vertcat(
            self.thrust_ratio, self.tau_rate, self.tau_throttle, 
            self.centre_rate_deg, self.max_rate_deg, self.rate_expo,
            self.drag_coeff_z, self.thrust_offset, self.tau_thrust
        )

        self.g = 9.81
    
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
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics(), self.u_dynamics())
        return cs.Function('x_dot', [self.x, self.u_dot, self.p_param], [x_dot], ['x', 'u', 'p'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        # Enhanced thrust model with offset and drag
        thrust_magnitude = self.thrust_ratio * self.u[2] + self.thrust_offset
        
        # Thrust in body frame Z direction only
        a_thrust = cs.vertcat(0.0, 0.0, thrust_magnitude)
        
        # Transform body-frame thrust to world frame
        a_thrust_world = self.v_dot_q(a_thrust, self.q)
        
        # Vertical drag force (proportional to velocity)
        # Using linear drag for stability, but could use quadratic: -drag * v * |v|
        drag_force = cs.vertcat(
            0.0,  # No horizontal drag for now
            0.0,
            -self.drag_coeff_z * self.v[2]  # Vertical drag
        )
        
        # Gravity vector
        g = cs.vertcat(0.0, 0.0, 9.81)
        
        v_dot = a_thrust_world - g + drag_force
        
        return v_dot

    def betaflight_rates(self, x):
        """
        Betaflight rates formula:
        h = x * (x^5 * g + x * (1-g))
        j = (d * x) + ((f-d) * h) for x in [-1, 1]
        """
        ax = cs.sqrt(x*x + 1e-6) 
        sgn = x / ax   
        
        h_abs = ax * (cs.power(ax, 5) * self.rate_expo + ax * (1.0 - self.rate_expo))
        j_abs = self.centre_rate_deg * ax + (self.max_rate_deg - self.centre_rate_deg) * h_abs
        j = sgn * j_abs
        
        return j

    def w_dynamics(self):
        """
        Angular rate dynamics using Betaflight rates formula
        """
        r_cmd = cs.vertcat(
            (self.betaflight_rates(self.u[0]) * np.pi) / 180.0,
            (self.betaflight_rates(self.u[1]) * np.pi) / 180.0,
            (self.betaflight_rates(-self.u[3]) * np.pi) / 180.0 
        )
        
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot
    
    def u_dynamics(self):
        # Enhanced actuator dynamics with first-order response for throttle
        u_dot_enhanced = cs.vertcat(
            self.u_dot[0],  # Roll - direct response
            self.u_dot[1],  # Pitch - direct response
            (1/self.tau_thrust) * (self.u_dot[2] - self.u[2]),  # Throttle - first order lag
            self.u_dot[3]   # Yaw - direct response
        )
        return u_dot_enhanced
