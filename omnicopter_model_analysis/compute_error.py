import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from acados_settings import generate_ocp_controller

def extract_state_from_row(row):
    """Extract 29D state vector from CSV row"""
    return np.array([
        row['px'], row['py'], row['pz'],                    # position (3)
        row['rw'], row['rx'], row['ry'], row['rz'],         # quaternion wxyz (4) 
        row['vx'], row['vy'], row['vz'],                    # velocity (3)
        row['wx'], row['wy'], row['wz'],                    # angular rates (3)
        row['u0_act'], row['u1_act'], row['u2_act'], row['u3_act'],  # actual actuators (8)
        row['u4_act'], row['u5_act'], row['u6_act'], row['u7_act'],
        row['u0_des'], row['u1_des'], row['u2_des'], row['u3_des'],  # desired actuators (8) 
        row['u4_des'], row['u5_des'], row['u6_des'], row['u7_des']
    ])

def extract_command_from_row(row):
    """Extract 8D command vector from CSV row"""
    return np.array([
        row['u0_dot'], row['u1_dot'], row['u2_dot'], row['u3_dot'],
        row['u4_dot'], row['u5_dot'], row['u6_dot'], row['u7_dot']
    ])

# Generate the OCP solver and simulator
print("Generating OCP solver and simulator...")
ocp, sim_integrator = generate_ocp_controller()

# Load the CSV data
print("Loading CSV data...")
df = pd.read_csv('control_results_omni_test_7.csv')
n_steps = len(df) - 1  # Number of prediction steps we can make

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
    velocity_x_errors.append(abs(error[7]))
    velocity_y_errors.append(abs(error[8]))
    velocity_z_errors.append(abs(error[9]))
    
    # Angular rate error (RMS of wx,wy,wz)
    ang_error = np.sqrt(np.mean(error[10:13]**2))
    angular_rate_errors.append(ang_error)
    
    # Individual angular rate errors
    angular_x_errors.append(abs(error[10]))
    angular_y_errors.append(abs(error[11]))
    angular_z_errors.append(abs(error[12]))
    
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

# Plot 3: Velocity Error Components
plt.subplot(4, 3, 7)
plt.plot(time_steps, velocity_x_errors, 'r-', linewidth=2)
plt.title('X Velocity Error')
plt.xlabel('Time (s)')
plt.ylabel('Velocity Error (m/s)')
plt.grid(True, alpha=0.3)

plt.subplot(4, 3, 8)
plt.plot(time_steps, velocity_y_errors, 'g-', linewidth=2)
plt.title('Y Velocity Error')
plt.xlabel('Time (s)')
plt.ylabel('Velocity Error (m/s)')
plt.grid(True, alpha=0.3)

plt.subplot(4, 3, 9)
plt.plot(time_steps, velocity_z_errors, 'b-', linewidth=2)
plt.title('Z Velocity Error')
plt.xlabel('Time (s)')
plt.ylabel('Velocity Error (m/s)')
plt.grid(True, alpha=0.3)

# Plot 4: Angular Rate Error Components
plt.subplot(4, 3, 10)
plt.plot(time_steps, angular_x_errors, 'r-', linewidth=2)
plt.title('X Angular Rate Error')
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate Error (rad/s)')
plt.grid(True, alpha=0.3)

plt.subplot(4, 3, 11)
plt.plot(time_steps, angular_y_errors, 'g-', linewidth=2)
plt.title('Y Angular Rate Error')
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate Error (rad/s)')
plt.grid(True, alpha=0.3)

plt.subplot(4, 3, 12)
plt.plot(time_steps, angular_z_errors, 'b-', linewidth=2)
plt.title('Z Angular Rate Error')
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate Error (rad/s)')
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('detailed_model_analysis.png', dpi=300, bbox_inches='tight')

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

print("\nPlots saved as 'detailed_model_analysis.png' and 'model_prediction_rms_errors.png'")



