import os
import sys
import shutil
import casadi as cs
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

class QuadDynamics:
    def __init__(self):
        # ... (your existing declarations)
        self.p = cs.MX.sym('p', 3)
        self.q = cs.MX.sym('a', 4)
        self.v = cs.MX.sym('v', 3)
        self.r = cs.MX.sym('r', 3)
        self.u = cs.MX.sym('u', 4)
        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.u)
        self.state_dim = 17
        self.u_dot = cs.MX.sym('u_dot', 4)

        # --- parameters ---
        self.thrust_ratio   = cs.MX.sym('kT', 1)
        self.drag_coeff_z   = cs.MX.sym('drag_coeff_z', 1)
        self.tau_rate       = cs.MX.sym('tau_rate', 1)          # first-order body-rate time constant
        self.centre_rate_deg= cs.MX.sym('centre_rate_deg', 1)   # BF rates (kept for yaw)
        self.max_rate_deg   = cs.MX.sym('max_rate_deg', 1)
        self.rate_expo      = cs.MX.sym('rate_expo', 1)
        # NEW for angle mode:
        self.angle_max_deg  = cs.MX.sym('angle_max_deg', 1)     # stick→angle map
        self.tau_angle      = cs.MX.sym('tau_angle', 1)         # first-order angle loop time constant

        # Parameter vector (append new ones to keep prior order stable)
        self.p_param = cs.vertcat(
            self.thrust_ratio,
            self.drag_coeff_z,
            self.tau_rate,
            self.centre_rate_deg,
            self.max_rate_deg,
            self.rate_expo,
            self.angle_max_deg,   # NEW (index +6)
            self.tau_angle        # NEW (index +7)
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
    
    # ---------------- helpers ----------------
    def quat_to_euler(self, q):
        # q = [qw, qx, qy, qz]
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        # Tait-Bryan ZYX → roll(x), pitch(y), yaw(z)
        # roll
        sinr_cosp = 2*(qw*qx + qy*qz)
        cosr_cosp = 1 - 2*(qx*qx + qy*qy)
        roll = cs.atan2(sinr_cosp, cosr_cosp)
        # pitch (clamp for numeric safety)
        sinp = 2*(qw*qy - qz*qx)
        sinp = cs.fmax(-1.0, cs.fmin(1.0, sinp))
        pitch = cs.asin(sinp)
        # yaw
        siny_cosp = 2*(qw*qz + qx*qy)
        cosy_cosp = 1 - 2*(qy*qy + qz*qz)
        yaw = cs.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def betaflight_rates(self, x):
        # unchanged (we'll use it for yaw only)
        ax  = cs.sqrt(x*x + 1e-6)
        sgn = x / ax
        h_abs = ax * (cs.power(ax, 5)*self.rate_expo + ax*(1.0 - self.rate_expo))
        j_abs = self.centre_rate_deg*ax + (self.max_rate_deg - self.centre_rate_deg)*h_abs
        return sgn * j_abs

    # ---------------- dynamics ----------------
    def quad_dynamics(self):
        x_dot = cs.vertcat(
            self.p_dynamics(),
            self.q_dynamics(),
            self.v_dynamics(),
            self.w_dynamics(),     # (modified)
            self.u_dynamics()
        )
        return cs.Function('x_dot', [self.x, self.u_dot, self.p_param], [x_dot], ['x','u','p'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 0.5 * cs.mtimes(self.skew_symmetric(self.r), self.q)

    def v_dynamics(self):
        a_thrust = cs.vertcat(0.0, 0.0, self.thrust_ratio * self.u[2])
        drag_force = cs.vertcat(0.0, 0.0, -self.drag_coeff_z * self.v[2])
        g = cs.vertcat(0.0, 0.0, self.g)
        return self.v_dot_q(a_thrust, self.q) - g + drag_force

    def w_dynamics(self):
        """
        Angle mode (roll/pitch): sticks map to angle setpoints.
          φ_sp = angle_max * u0,  θ_sp = angle_max * u1    (rad)
        First-order outer angle loop:  φ̇_des = (φ_sp - φ)/tau_angle,  θ̇_des = (θ_sp - θ)/tau_angle
        Yaw stays rate-mode via Betaflight curve → ψ̇_des (rad/s).
        Desired Euler-rate vector → desired body rates via kinematic map.
        Then first-order actuator on body rates:  ṙ = (1/tau_rate) * (r_cmd - r)
        """
        # angles from quaternion
        roll, pitch, yaw = self.quat_to_euler(self.q)

        # stick→angle (rad)
        angle_max_rad = (self.angle_max_deg * cs.pi) / 180.0
        phi_sp   = angle_max_rad * self.u[0]
        theta_sp = angle_max_rad * self.u[1]

        # desired Euler angle rates (rad/s) – first-order towards the setpoint
        phi_dot_des   = (phi_sp   - roll)  / self.tau_angle
        theta_dot_des = (theta_sp - pitch) / self.tau_angle

        # yaw desired rate from Betaflight curve (deg/s → rad/s)
        yaw_rate_des = (self.betaflight_rates(-self.u[3]) * cs.pi) / 180.0

        euler_dot_des = cs.vertcat(phi_dot_des, theta_dot_des, yaw_rate_des)  # [φ̇, θ̇, ψ̇]

        # Kinematic map: euler_dot = T(φ,θ) * r   →   r_cmd = solve(T, euler_dot_des)
        sphi, cphi = cs.sin(roll), cs.cos(roll)
        ttheta = cs.tan(pitch)
        ctheta = cs.cos(pitch)
        eps = 1e-6  # avoid singularities at cos(θ)=0
        ctheta = ctheta + 0.0*eps  # symbolic no-op but keeps intent clear

        T = cs.vertcat(
            cs.horzcat(1,          sphi*ttheta,  cphi*ttheta),
            cs.horzcat(0,          cphi,         -sphi),
            cs.horzcat(0,          sphi/(ctheta+eps), cphi/(ctheta+eps))
        )
        r_cmd = cs.solve(T, euler_dot_des)   # [p,q,r] desired (rad/s)

        # first-order actuator on body rates
        r_dot = (1.0 / self.tau_rate) * (r_cmd - self.r)
        return r_dot

    def u_dynamics(self):
        return self.u_dot
