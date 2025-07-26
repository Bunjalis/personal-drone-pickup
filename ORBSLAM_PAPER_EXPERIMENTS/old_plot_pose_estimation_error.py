import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 1000  # Adjusted for your data length

# Plot settings
FIGURE_SIZE_3D = (12, 8)
FIGURE_SIZE_2D = (12, 6)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 38
AXIS_LABEL_SIZE = 30
TICK_LABEL_SIZE = 24
LEGEND_SIZE = 26
LINE_WIDTH = 3
GRID_ALPHA = 0.3

# Tick settings - Control the number of ticks on axes
MAX_TICKS_3D = 5        # Maximum number of ticks per axis on 3D plot
MAX_TICKS_2D = 6        # Maximum number of ticks per axis on 2D plot

# 3D plot label positioning
LABEL_PAD_3D = 20       # Distance of axis labels from the axis (in points)
TICK_PAD_Z = 10         # Distance of z-axis tick labels from the axis (in points)

# Colors for state error plots
OBSERVED_ERROR_COLOR = '#00429d'    # Dark blue - observed state error
ESTIMATED_ERROR_COLOR = '#e74c3c'   # Red - estimated state error

# Line styles
OBSERVED_ERROR_STYLE = '-'          # Solid for observed
ESTIMATED_ERROR_STYLE = '-'         # Solid for estimated

# Labels
OBSERVED_ERROR_LABEL = 'Observed State Error'
ESTIMATED_ERROR_LABEL = 'Estimated State Error'

# Axis labels
XLABEL = 'Time (s)'

# Dataset name
orbslam_dataset = 'BIGQUAD_XYZ_SINE_4_MOTION_CAPTURE_ORB_SLAM'

def calculate_error_stats(error_data):
    """Calculate RMSE and standard deviation for error data"""
    rmse = np.sqrt(np.mean(error_data**2))
    std_dev = np.std(error_data)
    return rmse, std_dev

def calculate_mae(predicted, actual):
    """Calculate Mean Absolute Error (L1 norm) between predicted and actual trajectories"""
    if len(predicted) != len(actual):
        min_len = min(len(predicted), len(actual))
        predicted = predicted[:min_len]
        actual = actual[:min_len]
    
    # Calculate errors for all state components
    state_errors = predicted - actual
    
    # Calculate MAE for position (first 3 columns: x, y, z)
    position_errors = state_errors[:, :3]
    mae_x = np.mean(np.abs(position_errors[:, 0]))
    mae_y = np.mean(np.abs(position_errors[:, 1]))
    mae_z = np.mean(np.abs(position_errors[:, 2]))
    
    # Calculate overall state MAE (L1 norm across all 13 components)
    mae_total_state = np.mean(np.sum(np.abs(state_errors), axis=1))
    
    return mae_x, mae_y, mae_z, mae_total_state

def load_trial_data(trial_directory):
    """Load all CSV files from a trial directory"""
    data = {}
    
    # Define the expected CSV files
    csv_files = {
        'control_history': 'control_history.csv',
        'observed_state_history': 'observed_state_history.csv',
        'motion_capture_history': 'motion_capture_history.csv',
        'parameter_estimation_history': 'parameter_estimation_history.csv',
        'estimated_state_history': 'estimated_state_history.csv',
        'delay_state_estimation_history': 'delay_state_estimation_history.csv',
        'UKF_state_estimation_history': 'UKF_state_estimation_history.csv',
        'trajectory': 'trajectory.csv'
    }
    
    for key, filename in csv_files.items():
        filepath = os.path.join(trial_directory, filename)
        if os.path.exists(filepath):
            try:
                data[key] = np.loadtxt(filepath, delimiter=',')
                print(f"Loaded {filename}: shape {data[key].shape}")
            except Exception as e:
                print(f"Warning: Could not load {filename}: {e}")
                data[key] = None
        else:
            print(f"Warning: {filename} not found in {trial_directory}")
            data[key] = None
    
    return data

def main():
    # Set up the path to the dataset
    base_path = os.path.dirname(os.path.abspath(__file__))
    orbslam_path = os.path.join(base_path, orbslam_dataset)
    
    # Load dataset
    orbslam_data = load_trial_data(orbslam_path)
    
    # Extract the data
    observed_state = orbslam_data['observed_state_history']
    estimated_state = orbslam_data['estimated_state_history'] 
    motion_capture = orbslam_data['motion_capture_history']
    delay_state = orbslam_data['delay_state_estimation_history']
    
    # Determine the minimum length to synchronize data
    min_length = min(len(observed_state), len(estimated_state), len(motion_capture), len(delay_state))
    
    # Trim all data to common length
    observed_state = observed_state[:min_length]
    estimated_state = estimated_state[:min_length]
    motion_capture = motion_capture[:min_length]
    delay_state = delay_state[:min_length]
    
    # Remove the first few seconds if needed
    skip_samples = 0 * 30  # Skip samples (0 seconds * 30 Hz)
    observed_state = observed_state[skip_samples:]
    estimated_state = estimated_state[skip_samples:]
    motion_capture = motion_capture[skip_samples:]
    delay_state = delay_state[skip_samples:]
    
    print(f"Using {len(observed_state)} samples for analysis")
    print(f"Delay state shape: {delay_state.shape}")
    
    # Align motion capture data with observed and estimated states
    # Calculate initial position offsets (first 3 components: x, y, z)
    observed_initial_pos = observed_state[0, :3]
    estimated_initial_pos = estimated_state[0, :3]
    motion_capture_initial_pos = motion_capture[0, :3]
    
    print(f"\nInitial positions before alignment:")
    print(f"  Observed:      [{observed_initial_pos[0]:.4f}, {observed_initial_pos[1]:.4f}, {observed_initial_pos[2]:.4f}]")
    print(f"  Estimated:     [{estimated_initial_pos[0]:.4f}, {estimated_initial_pos[1]:.4f}, {estimated_initial_pos[2]:.4f}]")
    print(f"  Motion Capture: [{motion_capture_initial_pos[0]:.4f}, {motion_capture_initial_pos[1]:.4f}, {motion_capture_initial_pos[2]:.4f}]")
    
    # Calculate offset to align motion capture with the average of observed and estimated initial positions
    target_initial_pos = (observed_initial_pos + estimated_initial_pos) / 2
    motion_capture_offset = target_initial_pos - motion_capture_initial_pos
    
    print(f"  Target (avg):   [{target_initial_pos[0]:.4f}, {target_initial_pos[1]:.4f}, {target_initial_pos[2]:.4f}]")
    print(f"  Applied offset: [{motion_capture_offset[0]:.4f}, {motion_capture_offset[1]:.4f}, {motion_capture_offset[2]:.4f}]")
    
    # Apply offset to motion capture position (first 3 columns)
    motion_capture_aligned = motion_capture.copy()
    motion_capture_aligned[:, :3] += motion_capture_offset
    
    print(f"  After alignment: [{motion_capture_aligned[0, 0]:.4f}, {motion_capture_aligned[0, 1]:.4f}, {motion_capture_aligned[0, 2]:.4f}]")
    
    # Create time vector in seconds
    time_steps = np.arange(len(observed_state)) / 30.0  # 30 Hz sampling
    
    # Calculate state errors using aligned motion capture data
    observed_error = observed_state - motion_capture_aligned
    estimated_error = estimated_state - motion_capture_aligned
    
    # Calculate complete state errors (all 13 components) using L1 norm
    observed_state_error = np.linalg.norm(observed_error, ord=1, axis=1)
    estimated_state_error = np.linalg.norm(estimated_error, ord=1, axis=1)
    
    # Calculate position errors (first 3 components: x, y, z) using L1 norm
    observed_position_error = np.linalg.norm(observed_error[:, :3], ord=1, axis=1)
    estimated_position_error = np.linalg.norm(estimated_error[:, :3], ord=1, axis=1)
    
    # Calculate rotation errors (quaternion: components 3-6) using L1 norm
    observed_rotation_error = np.linalg.norm(observed_error[:, 3:7], ord=1, axis=1)
    estimated_rotation_error = np.linalg.norm(estimated_error[:, 3:7], ord=1, axis=1)
    
    # Calculate linear velocity errors (components 7-9) using L1 norm
    observed_linear_vel_error = np.linalg.norm(observed_error[:, 7:10], ord=1, axis=1)
    estimated_linear_vel_error = np.linalg.norm(estimated_error[:, 7:10], ord=1, axis=1)
    
    # Calculate angular velocity errors (components 10-12) using L1 norm
    observed_angular_vel_error = np.linalg.norm(observed_error[:, 10:13], ord=1, axis=1)
    estimated_angular_vel_error = np.linalg.norm(estimated_error[:, 10:13], ord=1, axis=1)
    
    # Calculate MAE values (L1 norm) using aligned motion capture data
    mae_observed = calculate_mae(observed_state, motion_capture_aligned)
    mae_estimated = calculate_mae(estimated_state, motion_capture_aligned)
    
    # Calculate error statistics for each component
    obs_pos_rmse, obs_pos_std = calculate_error_stats(observed_position_error)
    est_pos_rmse, est_pos_std = calculate_error_stats(estimated_position_error)
    
    obs_rot_rmse, obs_rot_std = calculate_error_stats(observed_rotation_error)
    est_rot_rmse, est_rot_std = calculate_error_stats(estimated_rotation_error)
    
    obs_lin_rmse, obs_lin_std = calculate_error_stats(observed_linear_vel_error)
    est_lin_rmse, est_lin_std = calculate_error_stats(estimated_linear_vel_error)
    
    obs_ang_rmse, obs_ang_std = calculate_error_stats(observed_angular_vel_error)
    est_ang_rmse, est_ang_std = calculate_error_stats(estimated_angular_vel_error)
    
    print("\nMAE Analysis (L1 Norm):")
    print(f"Observed State vs Motion Capture:")
    print(f"  Total State (all 13 components): {mae_observed[3]:.4f}")
    print(f"Estimated State vs Motion Capture:")
    print(f"  Total State (all 13 components): {mae_estimated[3]:.4f}")
    
    print("\nError Statistics (RMSE / Std Dev):")
    print("Position Error:")
    print(f"  Observed:  RMSE={obs_pos_rmse:.4f}, σ={obs_pos_std:.4f}")
    print(f"  Estimated: RMSE={est_pos_rmse:.4f}, σ={est_pos_std:.4f}")
    print("Rotation Error (quaternion):")
    print(f"  Observed:  RMSE={obs_rot_rmse:.4f}, σ={obs_rot_std:.4f}")
    print(f"  Estimated: RMSE={est_rot_rmse:.4f}, σ={est_rot_std:.4f}")
    print("Linear Velocity Error:")
    print(f"  Observed:  RMSE={obs_lin_rmse:.4f}, σ={obs_lin_std:.4f}")
    print(f"  Estimated: RMSE={est_lin_rmse:.4f}, σ={est_lin_std:.4f}")
    print("Angular Velocity Error:")
    print(f"  Observed:  RMSE={obs_ang_rmse:.4f}, σ={obs_ang_std:.4f}")
    print(f"  Estimated: RMSE={est_ang_rmse:.4f}, σ={est_ang_std:.4f}")
    
    # Create six-subplot figure: state error, position, rotation, linear vel, angular vel, and delay state
    fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(FIGURE_SIZE_2D[0], FIGURE_SIZE_2D[1]*3.5), sharex=True)
    
    # Top subplot: Complete state error
    ax1.plot(time_steps, observed_state_error, color=OBSERVED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=OBSERVED_ERROR_LABEL, alpha=0.8)
    ax1.plot(time_steps, estimated_state_error, color=ESTIMATED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=ESTIMATED_ERROR_LABEL, alpha=0.8)
    
    ax1.set_ylabel('Complete State Error (L1 Norm)', fontsize=AXIS_LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax1.legend(fontsize=LEGEND_SIZE)
    ax1.grid(True, alpha=GRID_ALPHA)
    ax1.set_ylim(bottom=0)
    
    # Second subplot: Position error
    ax2.plot(time_steps, observed_position_error, color=OBSERVED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=OBSERVED_ERROR_LABEL, alpha=0.8)
    ax2.plot(time_steps, estimated_position_error, color=ESTIMATED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=ESTIMATED_ERROR_LABEL, alpha=0.8)
    
    ax2.set_ylabel('Position Error (L1 Norm)', fontsize=AXIS_LABEL_SIZE)
    ax2.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax2.legend(fontsize=LEGEND_SIZE)
    ax2.grid(True, alpha=GRID_ALPHA)
    ax2.set_ylim(bottom=0)
    
    # Third subplot: Rotation error (quaternion)
    ax3.plot(time_steps, observed_rotation_error, color=OBSERVED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=OBSERVED_ERROR_LABEL, alpha=0.8)
    ax3.plot(time_steps, estimated_rotation_error, color=ESTIMATED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=ESTIMATED_ERROR_LABEL, alpha=0.8)
    
    ax3.set_ylabel('Rotation Error (L1 Norm)', fontsize=AXIS_LABEL_SIZE)
    ax3.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax3.legend(fontsize=LEGEND_SIZE)
    ax3.grid(True, alpha=GRID_ALPHA)
    ax3.set_ylim(bottom=0)
    
    # Fourth subplot: Linear velocity error
    ax4.plot(time_steps, observed_linear_vel_error, color=OBSERVED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=OBSERVED_ERROR_LABEL, alpha=0.8)
    ax4.plot(time_steps, estimated_linear_vel_error, color=ESTIMATED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=ESTIMATED_ERROR_LABEL, alpha=0.8)
    
    ax4.set_ylabel('Linear Velocity Error (L1 Norm)', fontsize=AXIS_LABEL_SIZE)
    ax4.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax4.legend(fontsize=LEGEND_SIZE)
    ax4.grid(True, alpha=GRID_ALPHA)
    ax4.set_ylim(bottom=0)
    
    # Fifth subplot: Angular velocity error
    ax5.plot(time_steps, observed_angular_vel_error, color=OBSERVED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=OBSERVED_ERROR_LABEL, alpha=0.8)
    ax5.plot(time_steps, estimated_angular_vel_error, color=ESTIMATED_ERROR_COLOR, 
            linewidth=LINE_WIDTH, label=ESTIMATED_ERROR_LABEL, alpha=0.8)
    
    ax5.set_ylabel('Angular Velocity Error (L1 Norm)', fontsize=AXIS_LABEL_SIZE)
    ax5.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax5.legend(fontsize=LEGEND_SIZE)
    ax5.grid(True, alpha=GRID_ALPHA)
    ax5.set_ylim(bottom=0)
    
    # Bottom subplot: Delay state estimation history
    if delay_state.ndim == 1:
        # If delay_state is 1-dimensional, plot it as a single line
        ax6.plot(time_steps, delay_state, color='#2ecc71', linewidth=LINE_WIDTH, label='Delay State', alpha=0.8)
        ax6.set_ylabel('Delay State Estimation', fontsize=AXIS_LABEL_SIZE)
    else:
        # If delay_state is 2-dimensional, plot position components
        ax6.plot(time_steps, delay_state[:, 0], color='#2ecc71', linewidth=LINE_WIDTH, label='X Position', alpha=0.8)
        ax6.plot(time_steps, delay_state[:, 1], color='#e67e22', linewidth=LINE_WIDTH, label='Y Position', alpha=0.8)
        ax6.plot(time_steps, delay_state[:, 2], color='#9b59b6', linewidth=LINE_WIDTH, label='Z Position', alpha=0.8)
        ax6.set_ylabel('Delay State Estimation (m)', fontsize=AXIS_LABEL_SIZE)
    
    ax6.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax6.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax6.legend(fontsize=LEGEND_SIZE)
    ax6.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    fig.savefig('complete_state_error_analysis_comprehensive.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.4)

    plt.show()

if __name__ == "__main__":
    main()