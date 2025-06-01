import numpy as np
import scipy.linalg


class L1Controller:
    def __init__(self, filter_size=20, adaptation_gain=0.1, dt=1.0 / 30.0):
        self.filter_size = filter_size
        self.u_l1_buffer = np.zeros((filter_size, 1))
        self.adaptation_gain = adaptation_gain
        self.dt = dt

        self.sigma_hat_m = np.zeros(1)
        self.u_l1 = np.zeros(1)

        self.As = np.eye(1) * -0.2  # Change to a 1x1 matrix for compatibility with z_error
        self.g = np.array([[1.0]])  # Adjust g to match the dimensionality of As

    def rolling_average_filter(self, new_u_l1):
        self.u_l1_buffer = np.roll(self.u_l1_buffer, -1, axis=0)
        self.u_l1_buffer[-1, :] = new_u_l1
        return np.mean(self.u_l1_buffer, axis=0)

    def update(self, error):
        # Use only the z velocity component of the error
        z_error = np.array([error[9]])  # Create a 1D vector with only the z velocity component

        # Compute Phi and mu for the 1D case
        Phi = (1 / self.As) * (np.exp(self.As * self.dt) - 1)  # Simplified for 1x1 As
        mu = np.exp(self.As * self.dt) * z_error  # Simplified for 1D case

        # Compute sigma_hat for the 1D case
        sigma_hat = -self.adaptation_gain * (1 / Phi) * mu

        self.sigma_hat_m = sigma_hat 

        # Apply physical limiter to the augmentation term
        self.u_l1 = np.clip(-self.sigma_hat_m, -0.2, 0.2)  # Limit to [-0.2, 0.2]

        # Apply rolling average filter
        smoothed_u_l1 = self.rolling_average_filter(self.u_l1)

        return smoothed_u_l1
