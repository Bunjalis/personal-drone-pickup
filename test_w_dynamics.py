import casadi as cs
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import shutil
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# Add the path to import the dynamics
sys.path.append('/home/mitchell/Documents/PhD/drone_cage_control/src/controller_ukf/controller_ukf')
from dynamics import QuadDynamics

def test_w_dynamics_sweep():
    """Test w_dynamics with input sweep from -1 to 1"""
    
    # Create dynamics instance
    dynamics = QuadDynamics()
    
    # Create input sweep from -1 to 1
    u_values = np.linspace(-1, 1, 1000)
    
    # Set up a test state (we only care about w_dynamics output)
    test_state = np.zeros(17)  # 17-dimensional state
    test_state[3] = 1.0  # Set quaternion w component to 1 (unit quaternion)
    
    # Parameters for the dynamics [thrust_ratio, tau_rate, max_rate_deg]
    params = np.array([38.0, 0.07, 75.0])
    
    # Storage for results
    roll_rates = []
    pitch_rates = []
    yaw_rates = []
    
    for u_val in u_values:
        # Create control input vector [roll, pitch, throttle, yaw]
        u_input = np.array([u_val, u_val, 0.0, u_val])  # Only vary roll input
        
        # Set the state with this control input
        test_state[13:17] = u_input  # u is at indices 13-16 in the state vector
        
        # Create u_dot (rate of change of control inputs) - set to zero
        u_dot = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Evaluate w_dynamics
        w_dynamics_func = dynamics.w_dynamics()
        
        # For CasADi evaluation, we need to create a function and evaluate it
        x_sym = dynamics.x
        u_dot_sym = dynamics.u_dot
        p_sym = dynamics.p_param
        
        w_dot_expr = dynamics.w_dynamics()
        w_dot_func = cs.Function('w_dot', [x_sym, u_dot_sym, p_sym], [w_dot_expr])
        
        # Evaluate the function
        w_dot_result = w_dot_func(test_state, u_dot, params)
        w_dot_array = np.array(w_dot_result).flatten()
        
        # Store the commanded rates (this is what we're interested in)
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
    plt.plot(u_values, roll_rates, 'b-', linewidth=2, label='Betaflight Rates')
    plt.plot(u_values, u_values, 'r--', alpha=0.7, label='Linear')
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Roll Rate Command [rad/s]')
    plt.title('Roll Rate vs Control Input')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Pitch rates (should be zero since we only vary roll)
    plt.subplot(1, 3, 2)
    plt.plot(u_values, pitch_rates, 'g-', linewidth=2)
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Pitch Rate Command [rad/s]')
    plt.title('Pitch Rate vs Roll Input (should be ~0)')
    plt.grid(True, alpha=0.3)
    
    # Yaw rates (should be zero since we only vary roll)
    plt.subplot(1, 3, 3)
    plt.plot(u_values, yaw_rates, 'm-', linewidth=2)
    plt.xlabel('Control Input [-1, 1]')
    plt.ylabel('Yaw Rate Command [rad/s]')
    plt.title('Yaw Rate vs Roll Input (should be ~0)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print some sample values
    print("Sample w_dynamics outputs:")
    test_inputs = [-1.0, -0.5, 0.0, 0.5, 1.0]
    for i, u_test in enumerate(test_inputs):
        idx = int((u_test + 1) * 500)  # Find closest index
        if idx >= len(roll_rates):
            idx = len(roll_rates) - 1
        print(f"Input {u_test:4.1f} -> Roll rate: {roll_rates[idx]:8.4f} rad/s")

if __name__ == "__main__":
    test_w_dynamics_sweep()
