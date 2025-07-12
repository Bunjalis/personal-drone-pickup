import numpy as np
import scipy.linalg


class L1Controller:
    def __init__(self, filter_size=20, adaptation_gain=0.10, dt=1.0 / 30.0, omega_cutoff=0.5):
        self.filter_size = filter_size
        self.u_l1_buffer = np.zeros(filter_size)  # Buffer for Z-axis control input
        self.adaptation_gain = adaptation_gain  # Reduced adaptation gain for smoother response
        self.dt = dt
        self.omega_cutoff = omega_cutoff

        self.sigma_hat_m = 0.0  # Matched uncertainty estimate for Z-axis
        self.u_l1 = 0.0  # Control input for Z-axis

        self.As = -6  # More negative value for overdamped response

        self.z_tilda = 0.0  # Predicted Z-axis velocity


    def update(self, z, u):

        
        z_actual = z[9]  # Extract Z-axis velocity from state


        # Compute state error
        self.state_error = self.z_tilda - z_actual

        # f_thrust = 


        print(f"u: {u[2]}")
        f = -9.81 + 4 * 1.42e-06 * (u[2] *4631)**2 / 0.65  # Simplified dynamics for Z-axis
        g =  0.5 #4 * 1.42e-06 * (u[2] *4631)**2 / 0.65

        Phi = (1 / self.As) * (np.exp(self.As * self.dt) - 1)
        mu = np.exp(self.As * self.dt) * self.state_error
        self.sigma_hat_m = -self.adaptation_gain * (1 / g) * (1 / Phi) * mu

        self.u_l1 = self.u_l1 * np.exp(-self.omega_cutoff * self.dt) - self.sigma_hat_m * (1 - np.exp(-self.omega_cutoff * self.dt))
        self.u_l1 = np.clip(self.u_l1, -0.25, 0.25) 


        self.z_tilda += (f + g * self.u_l1 + self.As * self.state_error) * self.dt



        print(f"Z: {z_actual} f: {f* self.dt} g: {self.state_error}")

        return self.u_l1
