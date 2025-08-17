import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from acados_settings import generate_ocp_controller

def extract_state_from_row(row):
    """Extract 29D state vector from CSV row"""
    return np.array([
        row['x0_px'], row['x0_py'], row['x0_pz'],                    # position (3)
        row['x0_qw'], row['x0_qx'], row['x0_qy'], row['x0_qz'],         # quaternion wxyz (4) 
        row['x0_vx'], row['x0_vy'], row['x0_vz'],
        row['x0_wx'], row['x0_wy'], row['x0_wz'],                    # angular rates (3)
        row['x0_u0_act'], row['x0_u1_act'], row['x0_u2_act'], row['x0_u3_act'],  # actual actuators (8)
        row['x0_u4_act'], row['x0_u5_act'], row['x0_u6_act'], row['x0_u7_act'],
        row['x0_u0_des'], row['x0_u1_des'], row['x0_u2_des'], row['x0_u3_des'],  # desired actuators (8) 
        row['x0_u4_des'], row['x0_u5_des'], row['x0_u6_des'], row['x0_u7_des']
    ])

def extract_command_from_row(row):
    """Extract 8D command vector from CSV row"""
    return np.array([
        row['u_dot_0'], row['u_dot_1'], row['u_dot_2'], row['u_dot_3'],
        row['u_dot_4'], row['u_dot_5'], row['u_dot_6'], row['u_dot_7']
    ])

def compute_model_error(parameters=None, csv_file='control_results_hover_1.csv'):
    """
    Compute model prediction errors for given parameters
    
    Args:
        parameters: Array of 3 parameters to optimize [param1, param2, param3]
                   If None, uses default values
        csv_file: Path to the CSV file to use for analysis
    
    Returns:
        Dictionary containing error statistics and data
    """
    # Default parameter values if none provided
    if parameters is None:
        print("ERROR: No parameters provided for model error computation.")
        return  
    
    print(f"Using parameters: {parameters}")
    print(f"Using CSV file: {csv_file}")
    
    # Import required modules
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel, AcadosSim, AcadosSimSolver
    from dynamics import QuadDynamics
    
    # Generate the dynamics model
    quad_dynamics = QuadDynamics()
    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = quad_dynamics.x  # [p(3), q(4=wxyz), v(3), r(3), actuators(8), u_desired(8)] -> 29 states
    model.u = quad_dynamics.u_dot  # control input is now u_dot (8 inputs)
    model.p = quad_dynamics.p_param  # parameters

    model.f_expl_expr = dynamics_expr(quad_dynamics.x, quad_dynamics.u_dot, quad_dynamics.p_param)
    
    # Create simulation configuration
    print("Generating simulation integrator...")
    sim = AcadosSim()
    sim.model = model
    sim.solver_options.T = 1.0 / 30.0
    sim.parameter_values = parameters
    sim_integrator = AcadosSimSolver(sim)

    # Load the CSV data
    print("Loading CSV data...")
    df = pd.read_csv(csv_file)
    n_steps = len(df) - 10  # Number of prediction steps we can make

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
        current_row = df.iloc[i]
        next_row = df.iloc[i + 1]
        
        # Extract current state and command
        current_state = extract_state_from_row(current_row)
        command = extract_command_from_row(current_row)
        
        # Extract next measured state
        measured_next_state = extract_state_from_row(next_row)
        
        # Store position data
        positions_x.append(current_state[0])
        positions_y.append(current_state[1])
        positions_z.append(current_state[2])
        
        # Run simulation
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
        
        if i % 10 == 0:
            print(f"Step {i}: pos_err={pos_error:.6f}, vel_err={vel_error:.6f}, ang_err={ang_error:.6f}, quat_err={quat_error:.6f}")

    print(f"Processing complete. Final statistics:")
    print(f"Mean position error: {np.mean(position_errors):.6f} m")
    print(f"Mean velocity error: {np.mean(velocity_errors):.6f} m/s")
    print(f"Mean angular rate error: {np.mean(angular_rate_errors):.6f} rad/s")
    print(f"Mean quaternion error: {np.mean(quaternion_errors):.6f}")
    
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
    plt.show()

    print("\nPlots saved as 'detailed_model_analysis_with_velocity_comparison.png' and 'model_prediction_rms_errors.png'")


def objective_function(parameters, csv_file='control_results_hover_1.csv'):
    """
    Objective function for parameter optimization
    Returns the total RMS error to minimize
    """
    results = compute_model_error(parameters, csv_file)
    
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
    # Define the CSV file to use for optimization
    csv_file = 'control_results_hover_10_updated.csv'
    
    # Initial parameter guess: [thrust_constant, actuator_time_constant, motor_position, Jx, Jy, Jz]
    initial_params = np.array([7.42678162e-07, 0.08, 0.02, 0.02, 0.02])  # Thrust, Tau, Position, Jx, Jy, Jz

    # Run with default parameters
    print("Running model error analysis with default parameters...")
    results = compute_model_error(parameters=initial_params, csv_file=csv_file)
    
    # Plot results
    plot_results(results)
    
    # Parameter optimization:
    from scipy.optimize import minimize
    
    # Create objective function with fixed csv_file
    def objective_func_fixed(parameters):
        return objective_function(parameters, csv_file)
    
    # Run optimization
    print("\nStarting parameter optimization...")
    result = minimize(objective_func_fixed, initial_params, 
                      method='Nelder-Mead',
                      options={'maxiter': 50, 'disp': True})
    
    print("Initial parameters: [{:.10f}, {:.10f},  {:.6f}, {:.6f}, {:.6f}]".format(
        initial_params[0], initial_params[1], initial_params[2], 
        initial_params[3], initial_params[4]))
    print("Optimal parameters: [{:.10f}, {:.10f},  {:.6f}, {:.6f}, {:.6f}]".format(
        result.x[0], result.x[1], result.x[2], 
        result.x[3], result.x[4]))
    print("Minimum error: {:.10f}".format(result.fun))