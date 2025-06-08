import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib.ticker as ticker
import scipy.optimize as opt
import casadi as ca
from scipy.optimize import minimize

filename = 'thrust_data_step_18.csv'
data_thrust = pd.read_csv(filename, names=['Throttle', 'Thrust'])

# Invert the thrust column
data_thrust['Thrust'] = -data_thrust['Thrust']

time = np.arange(len(data_thrust))

time_seconds = time / 80
plt.figure(figsize=(10, 6))

plt.plot(time_seconds, data_thrust['Thrust'], label='Thrust', color='b')

plt.plot(time_seconds, data_thrust['Throttle'], label='Throttle', color='r', linestyle='--')

plt.title('Thrust vs Time (Seconds)')
plt.xlabel('Time (seconds)')
plt.ylabel('Thrust / Throttle')
plt.legend()
plt.grid(True)

ax = plt.gca()
ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=10))

plt.tight_layout()
plt.show()

# Extract throttle input and measured thrust
data_thrust['Thrust'] = data_thrust['Thrust'] * 9.80665  # Convert thrust from kg to N
throttle = data_thrust['Throttle'].values
measured_thrust = data_thrust['Thrust'].values
dt = 1 / 80  # Time step

# Define the cost function for optimizing a and b
def cost_function_ab(params):
    a, b, c = params
    scaled_throttle = a * (throttle * c)**2 + b * (throttle * c)
    return np.sum((scaled_throttle - measured_thrust) ** 2)

# Initial guesses for a, b, and c
initial_guess_ab = [1e-5, 1e-3, 4000]

# Perform optimization to fit the scaled throttle
result_ab = minimize(cost_function_ab, initial_guess_ab, bounds=[(1e-10, 1e-2), (1e-6, 1e-2), (1000, 10000)])
a_opt, b_opt, c_opt = result_ab.x

print(f"Optimized Parameters: a = {a_opt:.2e}, b = {b_opt:.2e}, c = {c_opt:.2f}")

scaled_throttle = a_opt * (throttle * c_opt)**2 + b_opt * (throttle * c_opt)

# Plot the results
plt.figure(figsize=(10, 6))

# Plot measured thrust
plt.plot(time_seconds, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time_seconds, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('First-Order System Response With Optimized Parameters')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()

# Define the simulate_casadi_ode_with_thrust function
def simulate_casadi_ode_with_thrust(params, throttle, dt, a, b, c):
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
    simulated_thrust = simulate_casadi_ode_with_thrust(params, throttle, dt, a_opt, b_opt, c_opt)
    return np.sum((simulated_thrust - measured_thrust) ** 2)

# Initial guesses for tau and K
initial_guess_tau_K = [0.1, 1.0]

# Perform optimization to fit tau and K
result_tau_K = minimize(cost_function_tau_K, initial_guess_tau_K, bounds=[(0.001, 10), (-10, 10)])
tau_opt, K_opt = result_tau_K.x

# Simulate the system with the optimized parameters
simulated_thrust = simulate_casadi_ode_with_thrust([tau_opt, K_opt], throttle, dt, a_opt, b_opt, c_opt)

# Plot the results
plt.figure(figsize=(10, 6))

# Plot simulated thrust with optimized parameters
plt.plot(time_seconds, simulated_thrust, label=f'Simulated Thrust (tau={tau_opt:.3f}, K={K_opt:.3f}, a={a_opt:.2e}, b={b_opt:.2e})', color='blue')

# Plot measured thrust
plt.plot(time_seconds, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time_seconds, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('First-Order System Response With Optimized Parameters')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()

# Modify the second-order system ODE to include acceleration delay
def simulate_casadi_ode_with_thrust_second_order(params, throttle, dt, a, b, c):
    tau, K, zeta = params

    # Define CasADi variables
    omega = ca.MX.sym('omega')  # State variable (angular velocity)
    u = ca.MX.sym('u')          # Input variable (throttle)

    # Define the ODE for second-order system with acceleration delay
    omega_dot = (1 / tau**2) * (-omega + K * u - 2 * zeta * tau * omega)

    # Create an integrator for the ODE
    ode = {'x': omega, 'p': u, 'ode': omega_dot}
    opts = {'tf': dt}  # Integration time step
    integrator = ca.integrator('integrator', 'cvodes', ode, opts)

    # Simulate the system
    omega_val = 0.0  # Initial condition to simulate acceleration delay
    thrust_history = []
    for u_val in throttle:
        res = integrator(x0=omega_val, p=u_val)
        omega_val = res['xf'].full().flatten()[0]
        thrust_val = a * (omega_val * c)**2 + b * (omega_val * c)
        thrust_history.append(thrust_val)

    return np.array(thrust_history)

# Define the cost function for optimizing tau, K, and zeta
def cost_function_tau_K_zeta(params):
    simulated_thrust = simulate_casadi_ode_with_thrust_second_order(params, throttle, dt, a_opt, b_opt, c_opt)
    return np.sum((simulated_thrust - measured_thrust) ** 2)

# Initial guesses for tau, K, and zeta
initial_guess_tau_K_zeta = [0.1, 1.0, 0.1]

# Perform optimization to fit tau, K, and zeta
result_tau_K_zeta = minimize(cost_function_tau_K_zeta, initial_guess_tau_K_zeta, bounds=[(0.001, 10), (-10, 10), (0.01, 2)])
if result_tau_K_zeta.success:
    tau_opt, K_opt, zeta_opt = result_tau_K_zeta.x
else:
    raise ValueError("Optimization for second-order system parameters failed.")

# Simulate the system with the optimized parameters
simulated_thrust_second_order = simulate_casadi_ode_with_thrust_second_order([tau_opt, K_opt, zeta_opt], throttle, dt, a_opt, b_opt, c_opt)

# Plot the results
plt.figure(figsize=(10, 6))

# Plot simulated thrust with optimized parameters
plt.plot(time_seconds, simulated_thrust_second_order, label=f'Simulated Thrust (Second Order: tau={tau_opt:.3f}, K={K_opt:.3f}, zeta={zeta_opt:.3f})', color='blue')

# Plot measured thrust
plt.plot(time_seconds, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

# Plot scaled throttle
plt.plot(time_seconds, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

# Add labels, title, and legend
plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('Second-Order System Response With Acceleration Delay')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()

print(f"Updated Parameters: tau = {tau_opt:.3f}, K = {K_opt:.3f}, zeta = {zeta_opt:.3f}, a = {a_opt:.2e}, b = {b_opt:.2e}, c = {c_opt:.2f}")
