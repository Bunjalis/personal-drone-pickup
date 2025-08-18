import casadi as cs
import numpy as np
import matplotlib.pyplot as plt

# Use the QuadDynamics class from this file
class QuadDynamics:
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

        self.thrust_ratio = cs.MX.sym('kT', 1)
        self.tau_rate = cs.MX.sym('tau', 1)
        self.max_rate_deg = cs.MX.sym('max_rate_deg', 1)
        
        # Betaflight rates parameters (hardcoded values)
        self.rates_d_val = 100  # Centre Rates
        self.rates_f_val = 200  # Max Rates
        self.rates_g_val = 1.0  # EXPO
        
        self.p_param = cs.vertcat(self.thrust_ratio, self.tau_rate, self.max_rate_deg)
        self.g = 9.81

    def betaflight_rates(self, x):
        """Betaflight rates formula using CasADi"""
        x_clamped = cs.fmax(-1.0, cs.fmin(1.0, x))
        x_abs = cs.fabs(x_clamped)
        h = x_abs * (cs.power(x_abs, 5) * self.rates_g_val + x_abs * (1 - self.rates_g_val))
        j_abs = (self.rates_d_val * x_abs) + ((self.rates_f_val - self.rates_d_val) * h)
        j = cs.sign(x_clamped) * j_abs
        return j

    def w_dynamics(self):
        """Angular rate dynamics using Betaflight rates formula"""
        r_cmd = cs.vertcat(
            (self.betaflight_rates(self.u[0]) * np.pi) / 180.0,
            (self.betaflight_rates(self.u[1]) * np.pi) / 180.0,
            (self.betaflight_rates(-self.u[3]) * np.pi) / 180.0 
        )
        r_dot = (1 / self.tau_rate) * (r_cmd - self.r)
        return r_dot

def test_w_dynamics_sweep():
    """Test w_dynamics with input sweep from -1 to 1"""
    
    # Create dynamics instance
    dynamics = QuadDynamics()
    
    # Create input sweep from -1 to 1
    u_values = np.linspace(-1, 1, 1000)
    
    # Create a CasADi function for w_dynamics
    w_dot_expr = dynamics.w_dynamics()
    w_dot_func = cs.Function('w_dot', 
                            [dynamics.x, dynamics.u_dot, dynamics.p_param], 
                            [w_dot_expr])
    
    # Set up test parameters [thrust_ratio, tau_rate, max_rate_deg]
    params = np.array([38.0, 0.07, 75.0])
    
    # Set up a test state (17-dimensional)
    test_state = np.zeros(17)
    test_state[3] = 1.0  # quaternion w component
    
    # u_dot (rate of change of control inputs) - set to zero
    u_dot = np.array([0.0, 0.0, 0.0, 0.0])
    
    # Storage for results
    roll_rates = []
    pitch_rates = []
    yaw_rates = []
    
    print("Testing w_dynamics with input sweep from -1 to 1...")
    
    for u_val in u_values:
        # Set control input in the state vector
        # State structure: [p(3), q(4), v(3), r(3), u(4)]
        # u is at indices 13-16
        test_state[13] = u_val    # roll input
        test_state[14] = 0.0      # pitch input  
        test_state[15] = 0.0      # throttle input
        test_state[16] = 0.0      # yaw input
        
        # Evaluate w_dynamics
        w_dot_result = w_dot_func(test_state, u_dot, params)
        w_dot_array = np.array(w_dot_result).flatten()
        
        # Store results
        roll_rates.append(w_dot_array[0])
        pitch_rates.append(w_dot_array[1])
        yaw_rates.append(w_dot_array[2])
    
    # Convert to numpy arrays
    roll_rates = np.array(roll_rates)
    pitch_rates = np.array(pitch_rates)
    yaw_rates = np.array(yaw_rates)
    
    # Plot the results
    plt.figure(figsize=(15, 5))
    
    # Roll rates vs input
    plt.subplot(1, 3, 1)
    plt.plot(u_values, roll_rates, 'b-', linewidth=2, label='Betaflight w_dynamics')
    plt.plot(u_values, u_values * 75 * np.pi / 180, 'r--', alpha=0.7, label='Linear comparison')
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Roll Rate Output [rad/s]')
    plt.title('w_dynamics Roll Output')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Pitch rates (should be close to zero since we only vary roll)
    plt.subplot(1, 3, 2)
    plt.plot(u_values, pitch_rates, 'g-', linewidth=2)
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Pitch Rate Output [rad/s]')
    plt.title('w_dynamics Pitch Output (roll input)')
    plt.grid(True, alpha=0.3)
    
    # Yaw rates (should be close to zero since we only vary roll)
    plt.subplot(1, 3, 3)
    plt.plot(u_values, yaw_rates, 'm-', linewidth=2)
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Yaw Rate Output [rad/s]')
    plt.title('w_dynamics Yaw Output (roll input)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print some sample values
    print("\nSample w_dynamics outputs (raw, no conversion):")
    test_inputs = [-1.0, -0.5, 0.0, 0.5, 1.0]
    for u_test in test_inputs:
        idx = int((u_test + 1) * 499.5)  # Convert to array index
        idx = min(idx, len(roll_rates) - 1)
        print(f"Input {u_test:4.1f} -> Roll rate: {roll_rates[idx]:8.4f} rad/s")

if __name__ == "__main__":
    test_w_dynamics_sweep()