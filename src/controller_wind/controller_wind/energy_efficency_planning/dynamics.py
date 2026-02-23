import casadi as cs
import numpy as np

# ---------------------------------------------------------
# Helper: Generate Wind Tunnel Interpolant (The "Map")
# ---------------------------------------------------------
def create_wind_interpolant():
    """
    Creates a CasADi B-Spline representing a 3D wind field.
    Simulates a 'Wind Tunnel' blowing +5m/s along the X-axis at Z=1.5m.
    """
    # 1. Define the spatial domain (in meters)
    # Range should cover your entire flight area
    nx, ny, nz = 30, 20, 10
    x_grid = np.linspace(-4, 8, nx)
    y_grid = np.linspace(-3, 3, ny)
    z_grid = np.linspace(0, 3, nz)

    # 2. Generate Grid Data
    # Shape: (nx, ny, nz, 3) flattened to 1D for CasADi
    # We use 3 separate channels for u, v, w components
    u_data = np.zeros((nx, ny, nz))
    v_data = np.zeros((nx, ny, nz))
    w_data = np.zeros((nx, ny, nz))

    for i, x in enumerate(x_grid):
        for j, y in enumerate(y_grid):
            for k, z in enumerate(z_grid):
                # --- GAUSSIAN WIND TUNNEL MODEL ---
                # Center of tunnel: Y=0, Z=1.5
                # Radius: ~1.0m
                # Strength: 5.0 m/s blowing towards +X
                
                dist_sq = (y - 0.0)**2 + (z - 1.5)**2
                radius_sq = 1.0**2
                strength = 5.0

                # Gaussian falloff
                wind_speed = strength * np.exp(-dist_sq / (2 * radius_sq))
                
                # Check bounds (optional: limit tunnel length)
                if x > -2.0 and x < 6.0: 
                    u_data[i, j, k] = wind_speed # X-component
                else:
                    u_data[i, j, k] = 0.0
                
                v_data[i, j, k] = 0.0        # Y-component
                w_data[i, j, k] = 0.0        # Z-component

    # 3. Create CasADi Interpolants
    # We create one function that returns a 3x1 vector [u, v, w]
    data_flat = np.concatenate([u_data.ravel(order='F'), 
                                v_data.ravel(order='F'), 
                                w_data.ravel(order='F')])
    
    # Note: We stack the data; creating a vector-valued interpolant in CasADi 
    # usually requires defining the output dimension.
    # Simpler approach: Create one interpolant per channel.
    
    interp_u = cs.interpolant('wind_u', 'bspline', [x_grid, y_grid, z_grid], u_data.ravel(order='F'))
    interp_v = cs.interpolant('wind_v', 'bspline', [x_grid, y_grid, z_grid], v_data.ravel(order='F'))
    interp_w = cs.interpolant('wind_w', 'bspline', [x_grid, y_grid, z_grid], w_data.ravel(order='F'))

    return interp_u, interp_v, interp_w


# ---------------------------------------------------------
# Original Class (Base)
# ---------------------------------------------------------
class QuadDynamics:
    def __init__(self):
        # Declare model variables
        self.p = cs.MX.sym('p', 3)  # Position
        self.q = cs.MX.sym('a', 4)  # Quaternion
        self.v = cs.MX.sym('v', 3)  # Velocity
        self.r = cs.MX.sym('r', 3)  # Rates
        self.u = cs.MX.sym('u', 4)  # Control inputs

        self.x = cs.vertcat(self.p, self.q, self.v, self.r, self.u)
        self.state_dim = 17

        self.u_dot = cs.MX.sym('u_dot', 4)
        
        # External parameters (Explicit Wind)
        self.wind = cs.MX.sym('wind', 3)

        self.thrust_ratio = cs.MX.sym('kT', 1)
        self.drag_coeff_z = cs.MX.sym('drag_coeff_z', 1)
        self.tau_rate = cs.MX.sym('tau', 1)
        self.centre_rate_deg = cs.MX.sym('centre_rate_deg', 1)
        self.max_rate_deg = cs.MX.sym('max_rate_deg', 1)
        self.rate_expo = cs.MX.sym('rate_expo', 1)

        # Parameter vector used in OCP
        self.p_param = cs.vertcat(self.thrust_ratio,
                                  self.drag_coeff_z,
                                  self.tau_rate,
                                  self.centre_rate_deg,
                                  self.max_rate_deg,
                                  self.rate_expo)
        self.g = 9.81
    
    def q_to_rot_mat(self, q):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        return cs.vertcat(
            cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
            cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
            cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

    def v_dot_q(self, v, q):
        rot_mat = self.q_to_rot_mat(q)
        return cs.mtimes(rot_mat, v)

    def skew_symmetric(self, v):
        return cs.vertcat(
            cs.horzcat(0, -v[0], -v[1], -v[2]),
            cs.horzcat(v[0], 0, v[2], -v[1]),
            cs.horzcat(v[1], -v[2], 0, v[0]),
            cs.horzcat(v[2], v[1], -v[0], 0))

    def quad_dynamics(self):
        # Standard dynamics using explicit wind parameter
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics(), self.u_dynamics())
        # Function signature compatible with acados.py
        return cs.Function('x_dot', [self.x, self.u_dot, cs.vertcat(self.p_param, self.wind)], [x_dot], ['x', 'u', 'p'], ['x_dot'])

    def p_dynamics(self):
        return self.v

    def q_dynamics(self):
        return 1 / 2 * cs.mtimes(self.skew_symmetric(self.r), self.q)
    
    def v_dynamics(self):
        # Uses self.wind (the symbolic parameter)
        wind_across = self.wind[0]
        wind_along = self.wind[1]
        wind_vertical = self.wind[2]

        drag_force = cs.vertcat(
            -self.drag_coeff_z * (self.v[0] - wind_across),
            -self.drag_coeff_z * (self.v[1] - wind_along),
            -self.drag_coeff_z * (self.v[2] - wind_vertical)
        )
        
        a_thrust = cs.vertcat(0.0, 0.0, self.thrust_ratio * self.u[2])
        g_vec = cs.vertcat(0.0, 0.0, 9.81)
        
        return self.v_dot_q(a_thrust, self.q) - g_vec + drag_force

    def betaflight_rates(self, x):
        ax  = cs.sqrt(x*x + 1e-6)
        sgn = x / ax   
        h_abs = ax * (cs.power(ax, 5) * self.rate_expo  + ax * (1.0 - self.rate_expo ))
        j_abs = self.centre_rate_deg * ax + (self.max_rate_deg  - self.centre_rate_deg) * h_abs
        j = sgn * j_abs
        return j
    
    def w_dynamics(self):
        r_cmd = cs.vertcat(
            (self.betaflight_rates(self.u[0]) * np.pi) / 180.0,
            (self.betaflight_rates(self.u[1]) * np.pi) / 180.0,
            (self.betaflight_rates(-self.u[3]) * np.pi) / 180.0
        )
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot
    
    def u_dynamics(self):
        return self.u_dot


# ---------------------------------------------------------
# New Energy-Aware Class (The "Solution")
# ---------------------------------------------------------
class EnergyDynamics(QuadDynamics):
    def __init__(self):
        super().__init__()
        # Initialize the wind map
        self.wind_u, self.wind_v, self.wind_w = create_wind_interpolant()

    def v_dynamics(self):
        """
        OVERRIDE: Calculates drag using the spatial wind map instead of the static wind parameter.
        """
        # 1. Get current position from state
        pos = self.p  # [x, y, z]

        # 2. Query the CasADi Interpolant
        # This allows the solver to 'see' the wind field gradients
        w_x = self.wind_u(pos)
        w_y = self.wind_v(pos)
        w_z = self.wind_w(pos)

        # 3. Calculate Relative Velocity
        # Note: We IGNORE self.wind (the external parameter) here!
        v_air_x = self.v[0] - w_x
        v_air_y = self.v[1] - w_y
        v_air_z = self.v[2] - w_z

        # 4. Drag Force
        # Only linear drag for now, as per original model
        drag_force = cs.vertcat(
            -self.drag_coeff_z * v_air_x,
            -self.drag_coeff_z * v_air_y,
            -self.drag_coeff_z * v_air_z
        )
        
        a_thrust = cs.vertcat(0.0, 0.0, self.thrust_ratio * self.u[2])
        g_vec = cs.vertcat(0.0, 0.0, 9.81)
        
        return self.v_dot_q(a_thrust, self.q) - g_vec + drag_force

    def quad_dynamics(self):
        """
        OVERRIDE: Returns the function with the exact same signature as original,
        so 'acados.py' works without changes. The 'wind' parameter passed in 'p' 
        will simply be ignored by v_dynamics above.
        """
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(), self.w_dynamics(), self.u_dynamics())
        
        # We still include self.wind in the input list to maintain compatibility 
        # with the OCP wrapper that expects it.
        return cs.Function('x_dot', [self.x, self.u_dot, cs.vertcat(self.p_param, self.wind)], [x_dot], ['x', 'u', 'p'], ['x_dot'])