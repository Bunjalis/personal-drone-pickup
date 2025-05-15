import casadi as ca
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

# Load the CSV file
data = pd.read_csv('thrust_data_step_16.csv', header=None, names=['Throttle', 'Thrust'])

# Extract throttle input and measured thrust
throttle = data['Throttle'].values
measured_thrust = data['Thrust'].values
dt = 1/80  # Time step
time = np.arange(len(throttle)) * dt  # Generate time array

# Define the CasADi ODE model with embedded time delay
def simulate_casadi_ode_with_thrust(params, throttle, dt, delay_time):
    tau, K, C = params

    omega = ca.MX.sym('omega')
    u = ca.MX.sym('u')
    u_delayed = ca.MX.sym('u_delayed')

    omega_dot = (1 / tau) * (-omega + K * u_delayed)

    ode = {'x': omega, 'p': ca.vertcat(u, u_delayed), 'ode': omega_dot}
    opts = {'tf': dt}
    integrator = ca.integrator('integrator', 'cvodes', ode, opts)

    delay_steps = int(delay_time / dt)
    delayed_throttle = np.roll(throttle, delay_steps)
    delayed_throttle[:delay_steps] = 0

    omega_val = 0 
    thrust_history = []
    for u_val, u_delayed_val in zip(throttle, delayed_throttle):
        res = integrator(x0=omega_val, p=ca.vertcat(u_val, u_delayed_val))
        omega_val = res['xf'].full().flatten()[0]
        thrust_val = C * omega_val**2 * np.sign(omega_val)
        thrust_history.append(thrust_val)

    return np.array(thrust_history)


def cost_function_with_thrust(params):
    simulated_thrust = simulate_casadi_ode_with_thrust(params, throttle, dt, delay_time=0.05)
    return np.sum((simulated_thrust - measured_thrust) ** 2)

initial_guess = [0.1, -1.0, 3.75]

result = minimize(cost_function_with_thrust, initial_guess, bounds=[(0.001, 10), (-10, 10), (0.5,10)])
tau_opt, K_opt, C_opt = result.x


simulated_thrust = simulate_casadi_ode_with_thrust([tau_opt, K_opt,C_opt], throttle, dt, delay_time=0.05)


scaled_throttle = np.zeros_like(throttle)
scaled_throttle = np.where(throttle == 0.0, 0.0, 
                   np.where(throttle == 0.1, -0.0525, 
                   np.where(throttle == 0.2, -0.1560,
                   np.where(throttle == -0.1, 0.0525,
                   np.where(throttle == -0.2, 0.1560, scaled_throttle)))))

plt.figure(figsize=(10, 6))

plt.plot(time, simulated_thrust, label=f'Simulated Thrust (tau={tau_opt:.3f}, K={K_opt:.3f}),C={C_opt:.3f})', color='blue')

plt.plot(time, measured_thrust, label='Measured Thrust', color='green', linestyle='--')

plt.plot(time, scaled_throttle, label='Scaled Throttle', color='red', linestyle=':')

plt.xlabel('Time (s)')
plt.ylabel('Thrust / Scaled Throttle')
plt.title('First-Order System Response with Thrust Output (C * omega^2)')
plt.grid(True)
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()

