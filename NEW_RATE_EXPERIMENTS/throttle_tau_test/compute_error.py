import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def extract_state_from_row(row_data):
    """Extract 17D state vector from CSV row data
    State format: [p(3), q(4=wxyz), v(3), r(3), u(4)] -> 17 states
    """
    return np.array([
        row_data[0], row_data[1], row_data[2],                       # position (3)
        row_data[3], row_data[4], row_data[5], row_data[6],         # quaternion wxyz (4) 
        row_data[7], row_data[8], row_data[9],                      # velocity (3)
        row_data[10], row_data[11], row_data[12],                   # angular rates (3)
        0.0, 0.0, 0.0, 0.0                                         # actuator states (4) - placeholder
    ])

def extract_command_from_row(row_data):
    """Extract 4D command vector from CSV row data"""
    return np.array([
        row_data[0], row_data[1], row_data[2], row_data[3]
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
    
    initial_params = np.array([40.0, 0.07, 0.07, 80.0, 250.0, 0.5])
    
    print(f"Using fixed parameters: {initial_params}")
    
    sim.parameter_values = initial_params
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

    dt = 1.0 / 30.0  # 30 Hz as per the simulator settings

    # Process each step
    for i in range(n_steps):
        current_state_row = state_data[i]
        next_state_row = state_data[i + 1]
        control_row = control_data[i]
        
        # Extract current state and command
        current_state = extract_state_from_row(current_state_row)
        command = extract_command_from_row(control_row)
        
        # Extract next measured state
        measured_next_state = extract_state_from_row(next_state_row)
        
        # Store position data
        positions_x.append(current_state[0])
        positions_y.append(current_state[1])
        positions_z.append(current_state[2])
        
        # Run simulation with fixed parameters
        sim_integrator.set("x", current_state)
        sim_integrator.set("u", command)
        status = sim_integrator.solve()
        if status != 0:
            print(f"Warning: Simulation failed at step {i} with status {status}")
            continue
        
        # Get predicted next state
        predicted_next_state = sim_integrator.get("x")
        
        # Store velocity data for comparison plots
        predicted_vx.append(predicted_next_state[7])  # predicted vx
        predicted_vy.append(predicted_next_state[8])  # predicted vy
        predicted_vz.append(predicted_next_state[9])  # predicted vz
        measured_vx.append(measured_next_state[7])    # measured vx
        measured_vy.append(measured_next_state[8])    # measured vy
        measured_vz.append(measured_next_state[9])    # measured vz
        
        # Store angular velocity data for comparison plots
        predicted_wx.append(predicted_next_state[10]) # predicted wx
        predicted_wy.append(predicted_next_state[11]) # predicted wy
        predicted_wz.append(predicted_next_state[12]) # predicted wz
        measured_wx.append(measured_next_state[10])   # measured wx
        measured_wy.append(measured_next_state[11])   # measured wy
        measured_wz.append(measured_next_state[12])   # measured wz
        
        # Calculate errors
        error = predicted_next_state - measured_next_state
        
        # Store results
        time_steps.append(i * dt)
        
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
    
    # Return results dictionary
    results = {
        'parameters': initial_params,
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
        'mean_quaternion_error': np.mean(quaternion_errors)
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
    Returns the total RMS error to minimize
    """
    results = compute_model_error(parameters, state_csv_file, control_csv_file)
    
    # Combine different error types with weights
    pos_weight = 0.1
    vel_weight = 1.0
    ang_weight = 1.0
    quat_weight = 0.1
    
    total_error = (pos_weight * results['mean_position_error'] +
                  vel_weight * results['mean_velocity_error'] +
                  ang_weight * results['mean_angular_rate_error'] +
                  quat_weight * results['mean_quaternion_error'])
    
    print(f"Parameters: {parameters}, Total Error: {total_error:.6f}")
    return total_error


# Main execution
if __name__ == "__main__":
    # ===== CONFIGURATION =====
    # Define the CSV files to use
    state_csv_file = 'estimated_state_history.csv'
    control_csv_file = 'control_history.csv'

    # Define fixed parameters: [thrust_ratio, tau_rate, tau_throttle, centre_rate_deg, max_rate_deg, rate_expo]
    fixed_params = np.array([38.0, 0.07, 0.01, 80.0, 250.0, 0.5])

    # Run with fixed parameters
    print("Running model error analysis with fixed parameters...")
    results = compute_model_error(parameters=fixed_params, 
                                state_csv_file=state_csv_file, 
                                control_csv_file=control_csv_file)
    
    # Plot results
    if results is not None:
        plot_results(results)
        print("\nAnalysis complete. Check the generated plots for detailed results.")
    else:
        print("Error: Failed to compute model errors.")