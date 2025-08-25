import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def extract_state_from_row(state_row_data, control_row_data):
    """Extract 17D state vector from CSV row data
    State format: [p(3), q(4=wxyz), v(3), r(3), u(4)] -> 17 states
    - First 13 from state CSV: [p(3), q(4), v(3), r(3)]
    - Last 4 from control CSV first 4 columns: [u(4)] actuator states
    """
    return np.array([
        state_row_data[0], state_row_data[1], state_row_data[2],                     # position (3)
        state_row_data[3], state_row_data[4], state_row_data[5], state_row_data[6], # quaternion wxyz (4) 
        state_row_data[7], state_row_data[8], state_row_data[9],                    # velocity (3)
        state_row_data[10], state_row_data[11], state_row_data[12],                 # angular rates (3)
        control_row_data[0], control_row_data[1], control_row_data[2], control_row_data[3]  # actuator states u(4)
    ])

def extract_control_input_from_row(control_row_data):
    """Extract 4D control input vector (u_dot) from CSV row data
    Control input is the last 4 columns of control CSV
    """
    return np.array([
        control_row_data[4], control_row_data[5], control_row_data[6], control_row_data[7]
    ])

def extract_measured_state_from_row(state_row_data, predicted_actuator_states):
    """Extract 17D measured state vector using predicted actuator states
    This ensures we compare apples to apples - predicted vs measured dynamics states
    with the same actuator states
    """
    return np.array([
        state_row_data[0], state_row_data[1], state_row_data[2],                     # position (3)
        state_row_data[3], state_row_data[4], state_row_data[5], state_row_data[6], # quaternion wxyz (4) 
        state_row_data[7], state_row_data[8], state_row_data[9],                    # velocity (3)
        state_row_data[10], state_row_data[11], state_row_data[12],                 # angular rates (3)
        predicted_actuator_states[0], predicted_actuator_states[1], 
        predicted_actuator_states[2], predicted_actuator_states[3]                  # use predicted actuator states
    ])

def compute_model_error(parameters=None, state_csv_file='estimated_state_history.csv', control_csv_file='control_history.csv'):
    """
    Compute model prediction errors using fixed parameters
    
    Args:
        parameters: Array of 6 parameters [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo]
                   If None, uses default values
        state_csv_file: Path to the state history CSV file
        control_csv_file: Path to the control history CSV file
    
    Returns:
        Dictionary containing error statistics and data
    """
    print(f"Using state CSV file: {state_csv_file}")
    print(f"Using control CSV file: {control_csv_file}")
    
    # Import required modules
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, AcadosSim, AcadosSimSolver
    from dynamics import QuadDynamics
    
    # Generate the dynamics model
    quad_dynamics = QuadDynamics()
    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = quad_dynamics.x  # [p(3), q(4=wxyz), v(3), r(3), u(4)] -> 17 states
    model.u = quad_dynamics.u_dot  # control input is now u_dot (4 inputs)
    model.p = quad_dynamics.p_param  # parameters

    model.f_expl_expr = dynamics_expr(quad_dynamics.x, quad_dynamics.u_dot, quad_dynamics.p_param)
    
    # Create simulation configuration
    print("Generating simulation integrator...")
    sim = AcadosSim()
    sim.model = model
    sim.solver_options.T = 1.0 / 30.0
    

    print(f"Using parameters: {parameters}")
    print("Parameter meanings: [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z, tau_thrust]")
    
    sim.parameter_values = parameters
    sim_integrator = AcadosSimSolver(sim)

    # Load the CSV data
    print("Loading CSV data...")
    state_data = np.loadtxt(state_csv_file, delimiter=',')
    control_data = np.loadtxt(control_csv_file, delimiter=',')
    
    n_steps = min(len(state_data), len(control_data)) - 1  # Number of prediction steps we can make

    print(f"Processing {n_steps} prediction steps...")

    # Initialize arrays to store results
    time_steps = []
    position_errors = []
    velocity_errors = []
    angular_rate_errors = []
    quaternion_errors = []

    # Individual component arrays
    position_x_errors = []
    position_y_errors = []
    position_z_errors = []
    velocity_x_errors = []
    velocity_y_errors = []
    velocity_z_errors = []
    angular_x_errors = []
    angular_y_errors = []
    angular_z_errors = []

    # Position data for plotting
    positions_x = []
    positions_y = []
    positions_z = []

    # Velocity data for plotting (predicted vs measured)
    predicted_vx = []
    predicted_vy = []
    predicted_vz = []
    measured_vx = []
    measured_vy = []
    measured_vz = []

    # Angular velocity data for plotting (predicted vs measured)
    predicted_wx = []
    predicted_wy = []
    predicted_wz = []
    measured_wx = []
    measured_wy = []
    measured_wz = []

    # Initialize timing
    dt = 1.0 / 30.0  # 30 Hz as per the simulator settings
    
    # Process each step - with lag compensation for better timing alignment
    lag_compensation = 0  # Compensate for observed 2-timestep lag
    for i in range(n_steps - lag_compensation):
        current_state_row = state_data[i]
        # Compare with state that is lag_compensation steps ahead
        next_state_row = state_data[i + 1 + lag_compensation] 
        current_control_row = control_data[i]
        
        # Extract current state using both state and control data
        current_state = extract_state_from_row(current_state_row, current_control_row)
        
        # Extract control input (u_dot) from current timestep - this drives the transition to next state
        control_input = extract_control_input_from_row(current_control_row)
        
        # Store position data
        positions_x.append(current_state[0])
        positions_y.append(current_state[1])
        positions_z.append(current_state[2])
        
        # Run simulation with current parameters
        sim_integrator.set("x", current_state)
        sim_integrator.set("u", control_input)
        status = sim_integrator.solve()
        if status != 0:
            print(f"Warning: Simulation failed at step {i} with status {status}")
            continue
        
        # Get predicted next state
        predicted_next_state = sim_integrator.get("x")
        
        # For measured state, use the state at the lag-compensated timestep
        next_control_index = min(i + 1 + lag_compensation, len(control_data) - 1)
        measured_next_state = extract_state_from_row(next_state_row, control_data[next_control_index])
        
        # Store velocity data for comparison plots - with corrected timing alignment
        # Predicted velocity: predicted from current state i to next state i+1
        # Measured velocity: use current measured state (not lag-compensated) for proper alignment
        predicted_vx.append(predicted_next_state[7])  # predicted vx at time i+1
        predicted_vy.append(predicted_next_state[8])  # predicted vy at time i+1  
        predicted_vz.append(predicted_next_state[9])  # predicted vz at time i+1
        
        # Use current measured state for timing alignment (shift measured back one frame)
        current_measured_state = extract_state_from_row(current_state_row, current_control_row)
        measured_vx.append(current_measured_state[7])    # measured vx at time i (aligned with predicted at i+1)
        measured_vy.append(current_measured_state[8])    # measured vy at time i (aligned with predicted at i+1)
        measured_vz.append(current_measured_state[9])    # measured vz at time i (aligned with predicted at i+1)
        
        # Store angular velocity data for comparison plots - with corrected timing alignment
        predicted_wx.append(predicted_next_state[10]) # predicted wx at time i+1
        predicted_wy.append(predicted_next_state[11]) # predicted wy at time i+1
        predicted_wz.append(predicted_next_state[12]) # predicted wz at time i+1
        measured_wx.append(current_measured_state[10])   # measured wx at time i (aligned with predicted at i+1)
        measured_wy.append(current_measured_state[11])   # measured wy at time i (aligned with predicted at i+1)
        measured_wz.append(current_measured_state[12])   # measured wz at time i (aligned with predicted at i+1)
        
        # Calculate errors
        error = predicted_next_state - measured_next_state
        
        # Store results - time corresponds to the prediction timestep (i+1)
        time_steps.append((i + 1) * dt)  # Time of the predicted state and aligned measured state
        
        # Position error (RMS of x,y,z)
        pos_error = np.sqrt(np.mean(error[:3]**2))
        position_errors.append(pos_error)
        
        # Individual position errors
        position_x_errors.append(abs(error[0]))
        position_y_errors.append(abs(error[1]))
        position_z_errors.append(abs(error[2]))
        
        # Velocity error (RMS of vx,vy,vz)
        vel_error = np.sqrt(np.mean(error[7:10]**2))
        velocity_errors.append(vel_error)
        
        # Individual velocity errors
        velocity_x_errors.append((error[7]))
        velocity_y_errors.append((error[8]))
        velocity_z_errors.append((error[9]))
        
        # Angular rate error (RMS of wx,wy,wz)
        ang_error = np.sqrt(np.mean(error[10:13]**2))
        angular_rate_errors.append(ang_error)
        
        # Individual angular rate errors
        angular_x_errors.append((error[10]))
        angular_y_errors.append((error[11]))
        angular_z_errors.append((error[12]))
        
        # Quaternion error (RMS of qw,qx,qy,qz)
        quat_error = np.sqrt(np.mean(error[3:7]**2))
        quaternion_errors.append(quat_error)
        
        if i % 50 == 0:
            print(f"Step {i}: pos_err={pos_error:.6f}, vel_err={vel_error:.6f}, ang_err={ang_error:.6f}, quat_err={quat_error:.6f}")

    print(f"Processing complete. Final statistics:")
    print(f"Mean position error: {np.mean(position_errors):.6f} m")
    print(f"Mean velocity error: {np.mean(velocity_errors):.6f} m/s")
    print(f"Mean angular rate error: {np.mean(angular_rate_errors):.6f} rad/s")
    print(f"Mean quaternion error: {np.mean(quaternion_errors):.6f}")
    print(f"Mean Z position error: {np.mean(position_z_errors):.6f} m")
    print(f"Mean Z velocity error: {np.mean([abs(e) for e in velocity_z_errors]):.6f} m/s")
    
    # Return results dictionary
    results = {
        'parameters': parameters,
        'time_steps': time_steps,
        'position_errors': position_errors,
        'velocity_errors': velocity_errors,
        'angular_rate_errors': angular_rate_errors,
        'quaternion_errors': quaternion_errors,
        'position_x_errors': position_x_errors,
        'position_y_errors': position_y_errors,
        'position_z_errors': position_z_errors,
        'velocity_x_errors': velocity_x_errors,
        'velocity_y_errors': velocity_y_errors,
        'velocity_z_errors': velocity_z_errors,
        'angular_x_errors': angular_x_errors,
        'angular_y_errors': angular_y_errors,
        'angular_z_errors': angular_z_errors,
        'positions_x': positions_x,
        'positions_y': positions_y,
        'positions_z': positions_z,
        'predicted_vx': predicted_vx,
        'predicted_vy': predicted_vy,
        'predicted_vz': predicted_vz,
        'measured_vx': measured_vx,
        'measured_vy': measured_vy,
        'measured_vz': measured_vz,
        'predicted_wx': predicted_wx,
        'predicted_wy': predicted_wy,
        'predicted_wz': predicted_wz,
        'measured_wx': measured_wx,
        'measured_wy': measured_wy,
        'measured_wz': measured_wz,
        'mean_position_error': np.mean(position_errors),
        'mean_velocity_error': np.mean(velocity_errors),
        'mean_angular_rate_error': np.mean(angular_rate_errors),
        'mean_quaternion_error': np.mean(quaternion_errors),
        'mean_z_position_error': np.mean(position_z_errors),
        'mean_z_velocity_error': np.mean([abs(e) for e in velocity_z_errors])
    }
    
    return results


def plot_results(results):
    """Plot the analysis results"""
    # Extract data from results dictionary
    time_steps = results['time_steps']
    position_errors = results['position_errors']
    velocity_errors = results['velocity_errors']
    angular_rate_errors = results['angular_rate_errors']
    quaternion_errors = results['quaternion_errors']
    position_x_errors = results['position_x_errors']
    position_y_errors = results['position_y_errors']
    position_z_errors = results['position_z_errors']
    positions_x = results['positions_x']
    positions_y = results['positions_y']
    positions_z = results['positions_z']
    predicted_vx = results['predicted_vx']
    predicted_vy = results['predicted_vy']
    predicted_vz = results['predicted_vz']
    measured_vx = results['measured_vx']
    measured_vy = results['measured_vy']
    measured_vz = results['measured_vz']
    predicted_wx = results['predicted_wx']
    predicted_wy = results['predicted_wy']
    predicted_wz = results['predicted_wz']
    measured_wx = results['measured_wx']
    measured_wy = results['measured_wy']
    measured_wz = results['measured_wz']
    
    # Create the figure with multiple subplots
    fig = plt.figure(figsize=(20, 16))

    # Plot 1: Position vs Time
    plt.subplot(4, 3, 1)
    plt.plot(time_steps, positions_x, 'r-', linewidth=2, label='X Position')
    plt.title('X Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 2)
    plt.plot(time_steps, positions_y, 'g-', linewidth=2, label='Y Position')
    plt.title('Y Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position (m)')
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 3)
    plt.plot(time_steps, positions_z, 'b-', linewidth=2, label='Z Position')
    plt.title('Z Position vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.grid(True, alpha=0.3)

    # Plot 2: Position Error Components
    plt.subplot(4, 3, 4)
    plt.plot(time_steps, position_x_errors, 'r-', linewidth=2)
    plt.title('X Position Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 5)
    plt.plot(time_steps, position_y_errors, 'g-', linewidth=2)
    plt.title('Y Position Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 6)
    plt.plot(time_steps, position_z_errors, 'b-', linewidth=2)
    plt.title('Z Position Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.grid(True, alpha=0.3)

    # Plot 3: Linear Velocity Comparison (Predicted vs Measured)
    plt.subplot(4, 3, 7)
    plt.plot(time_steps, predicted_vx, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_vx, 'b:', linewidth=2, label='Measured')
    plt.title('X Linear Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 8)
    plt.plot(time_steps, predicted_vy, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_vy, 'b:', linewidth=2, label='Measured')
    plt.title('Y Linear Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 9)
    plt.plot(time_steps, predicted_vz, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_vz, 'b:', linewidth=2, label='Measured')
    plt.title('Z Linear Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot 4: Angular Velocity Comparison (Predicted vs Measured)
    plt.subplot(4, 3, 10)
    plt.plot(time_steps, predicted_wx, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_wx, 'b:', linewidth=2, label='Measured')
    plt.title('X Angular Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 11)
    plt.plot(time_steps, predicted_wy, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_wy, 'b:', linewidth=2, label='Measured')
    plt.title('Y Angular Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(4, 3, 12)
    plt.plot(time_steps, predicted_wz, 'r:', linewidth=2, label='Predicted')
    plt.plot(time_steps, measured_wz, 'b:', linewidth=2, label='Measured')
    plt.title('Z Angular Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('detailed_model_analysis_with_velocity_comparison.png', dpi=300, bbox_inches='tight')

    # Create a second figure with RMS errors
    fig2, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig2.suptitle('Model Prediction RMS Errors vs Time', fontsize=16)

    # Position error plot
    axes[0, 0].plot(time_steps, position_errors, 'b-', linewidth=2)
    axes[0, 0].set_title('Position Error (RMS)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Position Error (m)')
    axes[0, 0].grid(True, alpha=0.3)

    # Velocity error plot
    axes[0, 1].plot(time_steps, velocity_errors, 'r-', linewidth=2)
    axes[0, 1].set_title('Velocity Error (RMS)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Velocity Error (m/s)')
    axes[0, 1].grid(True, alpha=0.3)

    # Angular rate error plot
    axes[1, 0].plot(time_steps, angular_rate_errors, 'g-', linewidth=2)
    axes[1, 0].set_title('Angular Rate Error (RMS)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Angular Rate Error (rad/s)')
    axes[1, 0].grid(True, alpha=0.3)

    # Quaternion error plot
    axes[1, 1].plot(time_steps, quaternion_errors, 'm-', linewidth=2)
    axes[1, 1].set_title('Quaternion Error (RMS)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Quaternion Error')
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('model_prediction_rms_errors.png', dpi=300, bbox_inches='tight')
    plt.tight_layout()
    plt.savefig('model_prediction_rms_errors.png', dpi=300, bbox_inches='tight')
    plt.show()

    print("\nPlots saved as:")
    print("  - 'detailed_model_analysis_with_velocity_comparison.png'")
    print("  - 'model_prediction_rms_errors.png'")


def objective_function(parameters, state_csv_file='estimated_state_history.csv', control_csv_file='control_history.csv'):
    """
    Objective function for parameter optimization
    Minimizes only the z-axis position and velocity errors
    
    Args:
        parameters: Array of parameters to optimize 
                   [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z]
        state_csv_file: Path to state history CSV
        control_csv_file: Path to control history CSV
    
    Returns:
        Z-axis position and velocity error to minimize
    """
    try:
        results = compute_model_error(parameters, state_csv_file, control_csv_file)
        
        # Focus only on z-axis position and velocity errors
        z_position_error = results['mean_z_position_error']
        z_velocity_error = results['mean_z_velocity_error']
        
        # Weighted combination of z-axis errors
        pos_weight = 0.5
        vel_weight = 1.0
        
        total_z_error = pos_weight * z_position_error + vel_weight * z_velocity_error
        
        param_names = ['thrust_ratio', 'tau_rate', 'tau_throttle', 'centre_rate', 'max_rate', 'rate_expo', 'drag_z']
        param_str = ', '.join([f'{name}={val:.4f}' for name, val in zip(param_names[:len(parameters)], parameters)])
        print(f"Params: [{param_str}] -> Z-Pos: {z_position_error:.6f}, Z-Vel: {z_velocity_error:.6f}, Total: {total_z_error:.6f}")
        
        return total_z_error
        
    except Exception as e:
        print(f"Error in objective function: {e}")
        return float('inf')


def objective_function_thrust_only(thrust_ratio, base_parameters, state_csv_file='estimated_state_history.csv', control_csv_file='control_history.csv'):
    """
    Objective function for thrust-only optimization (backward compatibility)
    """
    full_parameters = np.array([thrust_ratio] + list(base_parameters))
    return objective_function(full_parameters, state_csv_file, control_csv_file)


# Main execution
if __name__ == "__main__":
    # ===== CONFIGURATION =====
    # Define the CSV files to use
    state_csv_file = 'estimated_state_history.csv'
    control_csv_file = 'control_history.csv'

    # Base parameters (fixed): [tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z]
    base_params = np.array([0.07, 0.07, 80.0, 250.0, 0.5, 0.0])  # Start with zero drag
    
    # Initial throttle ratio guess
    initial_thrust_ratio = 40.0
    
    # Full parameters for initial analysis: [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z]
    initial_full_params = np.array([initial_thrust_ratio] + list(base_params))

    # Run with initial parameters
    print("Running model error analysis with initial parameters...")
    print(f"Initial parameters: {initial_full_params}")
    print("Parameter meanings: [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z]")
    print("Optimizing ONLY thrust_ratio to minimize Z-axis position and velocity errors")
    
    results = compute_model_error(parameters=initial_full_params, 
                                state_csv_file=state_csv_file, 
                                control_csv_file=control_csv_file)
    
    # Plot initial results
    if results is not None:
        plot_results(results)
        print(f"\nInitial Z-axis errors:")
        print(f"  Z Position error: {results['mean_z_position_error']:.6f} m")
        print(f"  Z Velocity error: {results['mean_z_velocity_error']:.6f} m/s")
    else:
        print("Error: Failed to compute model errors.")
        exit(1)

    # Throttle ratio optimization options
    from scipy.optimize import minimize_scalar, minimize
    
    print("\n" + "="*60)
    print("Starting parameter optimization to minimize Z-axis errors...")
    print("="*60)
    
    # Option 1: Optimize only thrust ratio (fast)
    print("\nOption 1: Optimizing ONLY thrust_ratio...")
    def objective_func_thrust(thrust_ratio):
        return objective_function_thrust_only(thrust_ratio, base_params, state_csv_file, control_csv_file)
    
    result_thrust = minimize_scalar(objective_func_thrust, 
                                  bounds=(20.0, 80.0),
                                  method='bounded',
                                  options={'disp': True, 'maxiter': 25})
    
    print(f"Thrust-only optimization result: {result_thrust.x:.4f}, error: {result_thrust.fun:.6f}")
    
    # Option 2: Optimize thrust and drag together (potentially better but slower)
    print("\nOption 2: Optimizing thrust_ratio AND drag_coeff_z...")
    
    # Use thrust-only result as starting point, with small initial drag
    initial_thrust_drag = np.array([result_thrust.x] + list(base_params[:-1]) + [0.1])  # Start with small drag
    
    # Parameter bounds: [thrust_ratio, fixed params..., drag_coeff_z]
    param_bounds = [(20.0, 80.0)] + [(None, None)] * 5 + [(-2.0, 2.0)]  # Allow negative drag (can act as lift)
    
    # Only optimize thrust and drag, keep others fixed
    def objective_func_thrust_drag(x):
        # x contains [thrust_ratio, drag_coeff_z]
        full_params = np.array([x[0]] + list(base_params[:-1]) + [x[1]])
        return objective_function(full_params, state_csv_file, control_csv_file)
    
    result_thrust_drag = minimize(objective_func_thrust_drag,
                                [result_thrust.x, 0.1],  # [thrust_ratio, drag_coeff_z]
                                method='L-BFGS-B',
                                bounds=[(20.0, 80.0), (-2.0, 2.0)],
                                options={'disp': True, 'maxiter': 30})
    
    print(f"Thrust+Drag optimization result: thrust={result_thrust_drag.x[0]:.4f}, drag={result_thrust_drag.x[1]:.4f}, error: {result_thrust_drag.fun:.6f}")
    
    # Choose the better result
    if result_thrust_drag.fun < result_thrust.fun:
        print(f"\nUsing thrust+drag optimization (error improved by {((result_thrust.fun - result_thrust_drag.fun) / result_thrust.fun * 100):.2f}%)")
        final_params = np.array([result_thrust_drag.x[0]] + list(base_params[:-1]) + [result_thrust_drag.x[1]])
        final_error = result_thrust_drag.fun
        optimization_type = "thrust+drag"
    else:
        print(f"\nUsing thrust-only optimization (drag didn't improve results significantly)")
        final_params = np.array([result_thrust.x] + list(base_params))
        final_error = result_thrust.fun
        optimization_type = "thrust-only"
    
    print("\n" + "="*60)
    print("OPTIMIZATION RESULTS")
    print("="*60)
    print(f"Initial thrust ratio: {initial_thrust_ratio:.4f}")
    print(f"Initial drag coefficient: {base_params[-1]:.4f}")
    print(f"Optimal thrust ratio: {final_params[0]:.4f}")
    print(f"Optimal drag coefficient: {final_params[-1]:.4f}")
    print(f"Initial Z-position error: {results['mean_z_position_error']:.6f} m")
    print(f"Initial Z-velocity error: {results['mean_z_velocity_error']:.6f} m/s")
    print(f"Optimized total Z-error: {final_error:.6f}")
    print(f"Optimization type: {optimization_type}")
    
    # Run final analysis with optimized parameters
    print(f"\nRunning final analysis with optimized parameters...")
    optimized_results = compute_model_error(parameters=final_params,
                                          state_csv_file=state_csv_file,
                                          control_csv_file=control_csv_file)
    
    if optimized_results is not None:
        # Save optimized results plots with different filename
        import matplotlib.pyplot as plt
        
        # Clear any existing plots
        plt.close('all')
        
        # Plot optimized results
        plot_results(optimized_results)
        
        # Rename the generated plots to distinguish them
        import os
        if os.path.exists('detailed_model_analysis_with_velocity_comparison.png'):
            os.rename('detailed_model_analysis_with_velocity_comparison.png', 
                     'optimized_detailed_model_analysis_with_velocity_comparison.png')
        if os.path.exists('model_prediction_rms_errors.png'):
            os.rename('model_prediction_rms_errors.png', 
                     'optimized_model_prediction_rms_errors.png')
        
        print("\nOptimized plots saved as:")
        print("  - 'optimized_detailed_model_analysis_with_velocity_comparison.png'")
        print("  - 'optimized_model_prediction_rms_errors.png'")
        
        print("\n" + "="*60)
        print("FINAL COMPARISON")
        print("="*60)
        print("Z-axis error comparison (Initial -> Optimized):")
        print(f"Z Position error: {results['mean_z_position_error']:.6f} -> {optimized_results['mean_z_position_error']:.6f} "
              f"({((optimized_results['mean_z_position_error'] - results['mean_z_position_error']) / results['mean_z_position_error'] * 100):+.2f}%)")
        print(f"Z Velocity error: {results['mean_z_velocity_error']:.6f} -> {optimized_results['mean_z_velocity_error']:.6f} "
              f"({((optimized_results['mean_z_velocity_error'] - results['mean_z_velocity_error']) / results['mean_z_velocity_error'] * 100):+.2f}%)")
        
        # Calculate Z-axis improvement
        initial_z_total = 0.5*results['mean_z_position_error'] + 1.0*results['mean_z_velocity_error']
        optimized_z_total = 0.5*optimized_results['mean_z_position_error'] + 1.0*optimized_results['mean_z_velocity_error']
        
        print(f"\nWeighted Z-axis total error: {initial_z_total:.6f} -> {optimized_z_total:.6f} "
              f"({((optimized_z_total - initial_z_total) / initial_z_total * 100):+.2f}%)")
        
        print("\nAdditional error metrics for reference:")
        print(f"Overall position error: {results['mean_position_error']:.6f} -> {optimized_results['mean_position_error']:.6f}")
        print(f"Overall velocity error: {results['mean_velocity_error']:.6f} -> {optimized_results['mean_velocity_error']:.6f}")
        
        print("\nParameter optimization complete. Check the generated plots for detailed results.")
        
        # Save optimized parameters to file
        np.savetxt('optimized_parameters_with_drag.txt', final_params, 
                  header=f'Optimized parameters ({optimization_type}): [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo, drag_coeff_z]')
        print("Optimized parameters saved to 'optimized_parameters_with_drag.txt'")
        
    else:
        print("Error: Failed to compute optimized model errors.")