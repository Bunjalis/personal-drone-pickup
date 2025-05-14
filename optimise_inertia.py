import numpy as np
import pandas as pd
from scipy.optimize import minimize
from src.controller_mpc.controller_mpc.acados import generate_ocp_controller
from src.controller_mpc.controller_mpc.dynamics import QuadDynamics

sampling_frequency = 30  # Hz
time_step = 1 / sampling_frequency
cr = pd.read_csv('3_control_results.csv')
time_control = cr.index * time_step

# Load observed data for the y-axis
observed_y = cr['py'].values

# Load observed data for the x-axis
observed_x = cr['px'].values

state = [cr['px'], cr['py'], cr['pz'], cr['rw'], cr['rx'], cr['ry'], cr['rz'],
         cr['vx'], cr['vy'], cr['vz'], cr['wx'], cr['wy'], cr['wz'],cr['o_u0'], cr['o_u1'], cr['o_u2'], cr['o_u3']]
control = [cr['u0'], cr['u1'], cr['u2'], cr['u3']]

# Define the cost function to minimize for both x and y axes
def cost_function(inertia):
    inertia_y = inertia

    print(f"Optimizing with inertia_y: {inertia_y}")

    # Update the Acados model with the new inertia values
    quad_dynamics = QuadDynamics()
    quad_dynamics.J[2] = inertia_y  # Update y-axis inertia

    # Generate the OCP solver and simulation solver
    ocp_solver, sim_solver = generate_ocp_controller(quad_dynamics)

    error = 0

    for i in range(len(time_control) - 1):
        x_current = [state_var[i] for state_var in state]  # Current state from the file
        u_current = [control_var[i] for control_var in control]  # Current control input from the file

        sim_solver.set("x", np.array(x_current))
        sim_solver.set("u", np.array(u_current))
        sim_solver.solve()

        x_estimated_next = sim_solver.get("x")
        measured_next_state = np.array([state_var[i+1] for state_var in state])

        # Normalize the error by dividing by the number of states and timesteps
        error += np.sum(((x_estimated_next[0:13] - measured_next_state[0:13]) ** 2) / len(state))

    # Normalize the total error by the number of timesteps
    error /= len(time_control) - 1

    print(f"Total error: {error}")
    return error

# Perform the optimization for both x and y axes
initial_inertia = [ 0.002782410904]  # Initial guesses for x and y-axis inertia
result = minimize(cost_function, initial_inertia, bounds=[(0.0001, 0.1)], options={'maxiter': 1000})

# Print the optimized inertia values

print(f"Optimized y-axis inertia: {result.x[0]}")

#Inertia_x = 0.00543596413047688
#Inertia_y = 0.0024851688303563517
#Inertia_z = 0.0022638643527926913 ? 
