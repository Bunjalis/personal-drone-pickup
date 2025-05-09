import casadi as ca
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

# Load the CSV file
data = pd.read_csv('thrust_data_step_19.csv', header=None, names=['Throttle', 'Thrust'])

# Extract throttle input and measured thrust
throttle = data['Throttle'].values
measured_thrust = data['Thrust'].values * 9.80665  # Convert thrust from kg to N
dt = 1/80  # Time step
time = np.arange(len(throttle)) * dt  # Generate time array

# Define the cost function for optimizing a and b
def cost_function_ab(params):
    a, b,c = params
    scaled_throttle = a * (throttle * c)**2 + b * (throttle * c)
    return np.sum((scaled_throttle - measured_thrust) ** 2)

# Initial guesses for a and b
initial_guess_ab = [8.1443e-08, 1.5962e-04, 6000]

# Perform optimization to fit the scaled throttle
result_ab = minimize(cost_function_ab, initial_guess_ab, bounds=[(1e-10, 1e-6), (1e-6, 1e-2), (1000, 10000)])
a_opt, b_opt,c_opt = result_ab.x


scaled_throttle = a_opt * (throttle * c_opt)**2 + b_opt * (throttle * c_opt)


# Plot the results
plt.figure(figsize=(10, 6))

# Plot measured thrust
plt.plot(time, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('First-Order System Response With Optimized Parameters')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()









# Update the simulate_casadi_ode_with_thrust function to use static a and b
def simulate_casadi_ode_with_thrust(params, throttle, dt, a, b,c):
    tau, K = params

    # Define CasADi variables
    omega = ca.MX.sym('omega')  # State variable (angular velocity)
    u = ca.MX.sym('u')          # Input variable (throttle)

    # Define the ODE
    omega_dot = (1 / tau) * (-omega + K * u)

    # Create an integrator for the ODE
    ode = {'x': omega, 'p': u, 'ode': omega_dot}
    opts = {'tf': dt}  # Integration time step
    integrator = ca.integrator('integrator', 'cvodes', ode, opts)

    # Simulate the system
    omega_val = 0.1  # Initial condition
    thrust_history = []
    for u_val in throttle:
        res = integrator(x0=omega_val, p=u_val)
        omega_val = res['xf'].full().flatten()[0]
        thrust_val = a * (omega_val * c)**2 + b * (omega_val * c)
        thrust_history.append(thrust_val)

    return np.array(thrust_history)

# Define the cost function for optimizing tau and K
def cost_function_tau_K(params):
    simulated_thrust = simulate_casadi_ode_with_thrust(params, throttle, dt, a_opt, b_opt,c_opt)
    return np.sum((simulated_thrust - measured_thrust) ** 2)

# Initial guesses for tau and K
initial_guess_tau_K = [0.1, 1.0]

# Perform optimization to fit tau and K
result_tau_K = minimize(cost_function_tau_K, initial_guess_tau_K, bounds=[(0.001, 10), (-10, 10)])
tau_opt, K_opt = result_tau_K.x

# Simulate the system with the optimized parameters
simulated_thrust = simulate_casadi_ode_with_thrust([tau_opt, K_opt], throttle, dt, a_opt, b_opt,c_opt)

# Scale the throttle for comparison
scaled_throttle = a_opt * (throttle * c_opt)**2 + b_opt * (throttle * c_opt)

# Plot the results
plt.figure(figsize=(10, 6))

# Plot simulated thrust with optimized parameters
plt.plot(time, simulated_thrust, label=f'Simulated Thrust (tau={tau_opt:.3f}, K={K_opt:.3f}, a={a_opt:.2e}, b={b_opt:.2e})', color='blue')

# Plot measured thrust
plt.plot(time, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('First-Order System Response With Optimized Parameters')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()

print(f"Optimized Parameters: tau = {tau_opt:.3f}, K = {K_opt:.3f}, a = {a_opt:.2e}, b = {b_opt:.2e} c = {c_opt:.2f}")

# Load the ramp test data
ramp_data = pd.read_csv('thrust_data_ramp_1.csv', header=None, names=['Throttle', 'Thrust'])

# Extract throttle input and measured thrust for the ramp test
ramp_throttle = ramp_data['Throttle'].values
ramp_measured_thrust = ramp_data['Thrust'].values * 9.80665  # Convert thrust from kg to N

# Simulate the system with the ramp test data
ramp_simulated_thrust = simulate_casadi_ode_with_thrust([tau_opt, K_opt], ramp_throttle, dt, a_opt, b_opt,c_opt)

# Scale the throttle for comparison
ramp_scaled_throttle = a_opt * (ramp_throttle * c_opt)**2 + b_opt * (ramp_throttle * c_opt)

# Plot the results for the ramp test
plt.figure(figsize=(10, 6))

# Plot simulated thrust with optimized parameters
plt.plot(time[:len(ramp_simulated_thrust)], ramp_simulated_thrust, label=f'Simulated Thrust (tau={tau_opt:.3f}, K={K_opt:.3f}, a={a_opt:.2e}, b={b_opt:.2e})', color='blue')

# Plot measured thrust
plt.plot(time[:len(ramp_measured_thrust)], ramp_measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time[:len(ramp_scaled_throttle)], ramp_scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('Ramp Test: First-Order System Response With Optimized Parameters')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()


