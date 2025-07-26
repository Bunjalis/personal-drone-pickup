import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 1000

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

# Tick settings
MAX_TICKS_3D = 5
MAX_TICKS_2D = 6

# 3D plot label positioning
LABEL_PAD_3D = 20
TICK_PAD_Z = 10

# Colors
DESIRED_TRAJECTORY_COLOR = '#6795a0'
MOTION_CAPTURE_COLOR = '#e97d00'
OBSERVED_POSE_COLOR = '#1f4e79'

# Line styles
DESIRED_TRAJECTORY_STYLE = ':'
MOTION_CAPTURE_STYLE = '-'
OBSERVED_POSE_STYLE = '-'

# Labels
DESIRED_TRAJECTORY_LABEL = 'Desired Trajectory'
MOTION_CAPTURE_LABEL = 'Motion Capture Pose'
OBSERVED_POSE_LABEL = 'Observed Pose'

# Axis labels
XLABEL = 'Time (s)'

# Dataset names
orbslam_dataset = 'BIGQUAD_XYZ_SINE_4_MOTION_CAPTURE_ORB_SLAM'

def calculate_rmse(predicted, actual):
    """Calculate Root Mean Square Error between predicted and actual trajectories"""
    if len(predicted) != len(actual):
        min_len = min(len(predicted), len(actual))
        predicted = predicted[:min_len]
        actual = actual[:min_len]
    
    # Calculate position errors (first 3 columns: x, y, z)
    position_errors = predicted[:, :3] - actual[:, :3]
    
    # Calculate RMSE for each axis
    rmse_x = np.sqrt(np.mean(position_errors[:, 0]**2))
    rmse_y = np.sqrt(np.mean(position_errors[:, 1]**2))
    rmse_z = np.sqrt(np.mean(position_errors[:, 2]**2))
    
    # Calculate overall position RMSE
    rmse_position = np.sqrt(np.mean(np.sum(position_errors**2, axis=1)))
    
    return rmse_x, rmse_y, rmse_z, rmse_position

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
    # Set up the paths to both datasets
    base_path = os.path.dirname(os.path.abspath(__file__))
    orbslam_path = os.path.join(base_path, orbslam_dataset)
    
    # Load both datasets
    print("Loading motion capture data...")
    print("Loading ORB-SLAM data...")
    orbslam_data = load_trial_data(orbslam_path)
    
    # Determine the minimum length to synchronize data
    min_length = min(
        len(orbslam_data['observed_state_history']),
        len(orbslam_data['motion_capture_history']),
        len(orbslam_data['trajectory'])
    ) - 90  # Remove last 90 samples for stability
    
    # Extract trajectories - trim to common length
    desired_trajectory = orbslam_data['trajectory'][:min_length]
    motion_capture_pose = orbslam_data['motion_capture_history'][:min_length]  # Ground truth
    observed_pose = orbslam_data['observed_state_history'][:min_length]  # State estimation from motion capture
    
    # Skip initial samples if needed (remove startup transients)
    skip_samples = 0 * 30  # 0 seconds * 30 Hz
    desired_trajectory = desired_trajectory[skip_samples:]
    motion_capture_pose = motion_capture_pose[skip_samples:]
    observed_pose = observed_pose[skip_samples:]
    
    print(f"Using {len(desired_trajectory)} samples after removing {skip_samples} initial samples")
    
    # Align motion_capture_pose to observed_pose using initial position offset
    observed_initial_pos = observed_pose[0, :3]
    motion_capture_initial_pos = motion_capture_pose[0, :3]
    
    # Calculate offset to align motion_capture_pose to observed_pose
    position_offset = observed_initial_pos - motion_capture_initial_pos
    
    # Apply offset to motion_capture_pose to align with observed_pose
    motion_capture_pose_aligned = motion_capture_pose.copy()
    motion_capture_pose_aligned[:, :3] += position_offset
    
    print(f"Applied position offset to align poses: {position_offset}")
    
    # Apply additional manual shifts if needed
    # motion_capture_pose_aligned[:, 0] -= 0.2  # Shift along x-axis
    # motion_capture_pose_aligned[:, 1] -= 0.1  # Shift along y-axis
    
    # Extract position coordinates (first 3 columns: x, y, z)
    x_des, y_des, z_des = desired_trajectory[:, 0], desired_trajectory[:, 1], desired_trajectory[:, 2]
    x_mc, y_mc, z_mc = motion_capture_pose_aligned[:, 0], motion_capture_pose_aligned[:, 1], motion_capture_pose_aligned[:, 2]
    x_obs, y_obs, z_obs = observed_pose[:, 0], observed_pose[:, 1], observed_pose[:, 2]
    
    # Create time vector in seconds
    time_steps = np.arange(len(desired_trajectory)) / 30.0
    
    # Calculate RMSE values
    rmse_mc = calculate_rmse(motion_capture_pose_aligned, desired_trajectory)
    rmse_obs = calculate_rmse(observed_pose, desired_trajectory)
    rmse_pose_error = calculate_rmse(observed_pose, motion_capture_pose_aligned)  # Pose estimation accuracy
    
    print("\nRMSE Analysis (m):")
    print(f"Motion Capture vs Desired:")
    print(f"  X: {rmse_mc[0]:.4f}, Y: {rmse_mc[1]:.4f}, Z: {rmse_mc[2]:.4f}, Overall: {rmse_mc[3]:.4f}")
    print(f"Observed Pose vs Desired:")
    print(f"  X: {rmse_obs[0]:.4f}, Y: {rmse_obs[1]:.4f}, Z: {rmse_obs[2]:.4f}, Overall: {rmse_obs[3]:.4f}")
    print(f"Pose Estimation Error (Observed vs Motion Capture):")
    print(f"  X: {rmse_pose_error[0]:.4f}, Y: {rmse_pose_error[1]:.4f}, Z: {rmse_pose_error[2]:.4f}, Overall: {rmse_pose_error[3]:.4f}")
    
    # Camera position settings for 3D plot
    camera_azimuth = 45
    camera_elevation = 55
    
    # Calculate global data ranges for consistent axis bounds
    all_x = np.concatenate([x_des, x_mc, x_obs])
    all_y = np.concatenate([y_des, y_mc, y_obs])
    all_z = np.concatenate([z_des, z_mc, z_obs])
    
    # Calculate ranges for consistent scaling
    x_range = np.max(all_x) - np.min(all_x)
    y_range = np.max(all_y) - np.min(all_y)
    z_range = np.max(all_z) - np.min(all_z)
    
    # Use the maximum range for equal scale
    max_range = max(x_range, y_range, z_range)
    
    # Set equal limits for all axes
    x_center = (np.max(all_x) + np.min(all_x)) / 2
    y_center = (np.max(all_y) + np.min(all_y)) / 2
    z_center = (np.max(all_z) + np.min(all_z)) / 2

    # Create main 3D trajectory plot
    fig_main = plt.figure(figsize=FIGURE_SIZE_3D)
    ax_main = fig_main.add_subplot(111, projection='3d')
    
    # Plot trajectories
    ax_main.plot(x_des, y_des, z_des, color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
                 linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL, alpha=0.8)
    ax_main.plot(x_mc, y_mc, z_mc, color=MOTION_CAPTURE_COLOR, linestyle=MOTION_CAPTURE_STYLE, 
                 linewidth=LINE_WIDTH, label=MOTION_CAPTURE_LABEL, alpha=0.9)
    ax_main.plot(x_obs, y_obs, z_obs, color=OBSERVED_POSE_COLOR, linestyle=OBSERVED_POSE_STYLE, 
                 linewidth=LINE_WIDTH, label=OBSERVED_POSE_LABEL, alpha=0.9)
    
    # Set labels and formatting
    ax_main.set_xlabel('X (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax_main.set_ylabel('Y (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax_main.set_zlabel('Z (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax_main.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax_main.tick_params(axis='z', which='major', pad=TICK_PAD_Z)
    
    # Control tick density
    from matplotlib.ticker import MaxNLocator
    ax_main.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax_main.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax_main.zaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    
    # Add subplot label
    ax_main.text2D(0.0, 0.9, '(a)', transform=ax_main.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
                   fontweight='bold', verticalalignment='bottom')
    
    # Set equal axis limits
    ax_main.set_xlim([x_center - max_range/2, x_center + max_range/2])
    ax_main.set_ylim([y_center - max_range/2, y_center + max_range/2])
    ax_main.set_zlim([z_center - max_range/2, z_center + max_range/2])
    
    # Set camera position and formatting
    ax_main.view_init(elev=camera_elevation, azim=camera_azimuth)
    ax_main.set_box_aspect([1,1,1])
    ax_main.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Create legend figure
    fig_legend = plt.figure(figsize=(12, 1.5))
    ax_legend = fig_legend.add_subplot(111)
    ax_legend.axis('off')
    
    # Create dummy plots for legend
    ax_legend.plot([], [], color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
                   linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL)
    ax_legend.plot([], [], color=MOTION_CAPTURE_COLOR, linestyle=MOTION_CAPTURE_STYLE, 
                   linewidth=LINE_WIDTH, label=MOTION_CAPTURE_LABEL)
    ax_legend.plot([], [], color=OBSERVED_POSE_COLOR, linestyle=OBSERVED_POSE_STYLE, 
                   linewidth=LINE_WIDTH, label=OBSERVED_POSE_LABEL)
    
    # Create the legend
    legend = ax_legend.legend(fontsize=LEGEND_SIZE, ncol=2, loc='center', frameon=False)
    plt.tight_layout()
    
    # Create error analysis plot
    fig_error = plt.figure(figsize=FIGURE_SIZE_2D)
    ax_error = fig_error.add_subplot(111)
    
    # Calculate position errors over time
    error_mc = np.linalg.norm(motion_capture_pose_aligned[:, :3] - desired_trajectory[:, :3], axis=1)
    error_obs = np.linalg.norm(observed_pose[:, :3] - desired_trajectory[:, :3], axis=1)
    
    # Plot error over time
    ax_error.plot(time_steps, error_mc, color=MOTION_CAPTURE_COLOR, linestyle=MOTION_CAPTURE_STYLE, 
                  linewidth=LINE_WIDTH, label=MOTION_CAPTURE_LABEL, alpha=1.0)
    ax_error.plot(time_steps, error_obs, color=OBSERVED_POSE_COLOR, linestyle=OBSERVED_POSE_STYLE, 
                  linewidth=LINE_WIDTH, label=OBSERVED_POSE_LABEL, alpha=1.0)
    
    # Set labels and formatting
    ax_error.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax_error.set_ylabel('Trajectory Tracking\nPosition Error (m)', fontsize=AXIS_LABEL_SIZE)
    
    # Add subplot label
    ax_error.text(-0.05, 1.01, '(b)', transform=ax_error.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
                  fontweight='bold', verticalalignment='bottom')
    ax_error.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    
    # Control tick density  
    ax_error.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax_error.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    
    # Set y-axis to start from 0
    ax_error.set_ylim(bottom=0)
    ax_error.legend(fontsize=LEGEND_SIZE)
    ax_error.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Save the plots as PDFs
    fig_main.savefig('motion_capture_trajectory_main.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.4)
    fig_legend.savefig('motion_capture_trajectory_legend.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.1)
    fig_error.savefig('motion_capture_trajectory_analysis.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.1)
    
    print("\nPlots saved as:")
    print("  - 'motion_capture_trajectory_main.pdf'")
    print("  - 'motion_capture_trajectory_legend.pdf'")
    print("  - 'motion_capture_trajectory_analysis.pdf'")
    
    plt.show()
    

if __name__ == "__main__":
    main()
