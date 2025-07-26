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

# Colors - Custom palette (converted from D3.js)
# Palette: ['#00429d', '#415395', '#59668a', '#657b7d', '#68926c', '#5dab55', '#31c52f']
DESIRED_TRAJECTORY_COLOR = '#6795a0'    # Gray-green - reference line


# Line styles
DESIRED_TRAJECTORY_STYLE = ':'         # Dashed for reference
WITH_UKF_STYLE = '-'                   # Solid for with UKF
WITHOUT_UKF_STYLE = '-'                # Solid for without UKF

# Labels
DESIRED_TRAJECTORY_LABEL = 'Desired Trajectory'
WITH_UKF_LABEL = 'w UKF enabled'
WITHOUT_UKF_LABEL = 'w/o UKF enabled'

# Axis labels
XLABEL = 'Time (s)'
PARAMETER_YLABEL = 'Est. Thrust Ratio'

# Dataset names for comparison
motion_capture_dataset = 'BIGQUAD_CIRCLE_4_MOTION_CAPTURE'
orbslam_dataset = 'BIGQUAD_CIRCLE_6_MOTION_CAPTURE_ORB_SLAM'

motion_capture_dataset = 'BIGQUAD_XYZ_SINE_3_MOTION_CAPTURE'
orbslam_dataset = 'BIGQUAD_XYZ_SINE_4_MOTION_CAPTURE_ORB_SLAM'

# Additional colors for the new trajectories
# MOTION_CAPTURE_COLOR = '#4a90e2'        # Navy blue - motion capture input
# ORBSLAM_ESTIMATED_COLOR = '#1f4e79'     # Bright blue - ORB-SLAM estimated
ORBSLAM_ACTUAL_COLOR = '#e97d00'        # Bright green - actual trajectory with ORB-SLAM

# Labels for the plot
# MOTION_CAPTURE_LABEL = 'Motion Capture pose'
# ORBSLAM_ESTIMATED_LABEL = 'ORB-SLAM pose w/ Latency Compensation'
ORBSLAM_ACTUAL_LABEL = 'Motion Capture Pose'

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
    motion_capture_path = os.path.join(base_path, motion_capture_dataset)
    orbslam_path = os.path.join(base_path, orbslam_dataset)
    
    # Load both datasets
    motion_capture_data = load_trial_data(motion_capture_path)
    orbslam_data = load_trial_data(orbslam_path)
    
    # Determine the minimum length to synchronize data
    min_length = min(
        len(motion_capture_data['UKF_state_estimation_history']),
        len(orbslam_data['UKF_state_estimation_history']),
        len(orbslam_data['motion_capture_history']),
        len(orbslam_data['estimated_state_history'])
    ) - 90 # Remove last 240 samples as in original code

    # Extract trajectories - trim to common length
    desired_trajectory = orbslam_data['trajectory'][:min_length]
    motion_capture_result = motion_capture_data['observed_state_history'][:min_length]
    orbslam_estimated = orbslam_data['estimated_state_history'][:min_length]  # ORB-SLAM pose estimates
    orbslam_actual = orbslam_data['motion_capture_history'][:min_length]     # Ground truth when using ORB-SLAM
    
    # Remove the first 8 seconds of movement (240 samples at 30Hz)
    skip_samples = 0 * 30  # 8 seconds * 30 Hz = 240 samples
    desired_trajectory = desired_trajectory[skip_samples:]
    motion_capture_result = motion_capture_result[skip_samples:]
    orbslam_estimated = orbslam_estimated[skip_samples:]
    orbslam_actual = orbslam_actual[skip_samples:]
    
    print(f"Removed first {skip_samples} samples ({skip_samples/30:.1f} seconds) from all trajectories")
    
    # Keep all motion capture data unchanged - no normalization needed
    # Align orbslam_actual trajectory so its first state lines up with orbslam_estimated
    
    # Get initial positions for alignment
    orb_estimated_initial_pos = orbslam_estimated[0, :3]  # ORB-SLAM estimated start position
    orb_actual_initial_pos = orbslam_actual[0, :3]  # Motion capture ground truth start position
    
    # Calculate offset to align orbslam_actual to orbslam_estimated
    orb_offset = orb_estimated_initial_pos - orb_actual_initial_pos
    
    # Apply offset to orbslam_actual to align with orbslam_estimated
    orbslam_actual[:, :3] += orb_offset
    
    # Shift motion_capture_result trajectory
    motion_capture_result[:, 0] -= 0.2  # Shift back along x-axis by 0.2
    motion_capture_result[:, 1] -= 0.1  # Shift back along y-axis by 0.1
    
    print(f"Shifted motion_capture_result trajectory: x -= 0.2, y -= 0.1")
    
    # Extract position coordinates (first 3 columns: x, y, z) from final data
    x_des, y_des, z_des = desired_trajectory[:, 0], desired_trajectory[:, 1], desired_trajectory[:, 2]
    x_mc, y_mc, z_mc = motion_capture_result[:, 0], motion_capture_result[:, 1], motion_capture_result[:, 2]
    x_orb_est, y_orb_est, z_orb_est = orbslam_estimated[:, 0], orbslam_estimated[:, 1], orbslam_estimated[:, 2]
    x_orb_act, y_orb_act, z_orb_act = orbslam_actual[:, 0], orbslam_actual[:, 1], orbslam_actual[:, 2]

    # Create time vector in seconds (accounting for removed samples)
    remaining_length = len(desired_trajectory)
    time_steps = np.arange(remaining_length) / 30.0

    # Calculate RMSE values using final trajectories
    # rmse_mc = calculate_rmse(motion_capture_result, desired_trajectory)
    # rmse_orb_est = calculate_rmse(orbslam_estimated, desired_trajectory)
    rmse_orb_act = calculate_rmse(orbslam_actual, desired_trajectory)
    rmse_orb_pose_error = calculate_rmse(orbslam_estimated, orbslam_actual)  # ORB-SLAM pose estimation accuracy
    
    print("\nRMSE Analysis (m):")
    # print(f"Motion Capture Input vs Desired:")
    # print(f"  X: {rmse_mc[0]:.4f}, Y: {rmse_mc[1]:.4f}, Z: {rmse_mc[2]:.4f}, Overall: {rmse_mc[3]:.4f}")
    # print(f"ORB-SLAM Estimated vs Desired:")
    # print(f"  X: {rmse_orb_est[0]:.4f}, Y: {rmse_orb_est[1]:.4f}, Z: {rmse_orb_est[2]:.4f}, Overall: {rmse_orb_est[3]:.4f}")
    print(f"Actual w/ ORB-SLAM Input vs Desired:")
    print(f"  X: {rmse_orb_act[0]:.4f}, Y: {rmse_orb_act[1]:.4f}, Z: {rmse_orb_act[2]:.4f}, Overall: {rmse_orb_act[3]:.4f}")
    print(f"ORB-SLAM Pose Estimation Error (Estimated vs Ground Truth):")
    print(f"  X: {rmse_orb_pose_error[0]:.4f}, Y: {rmse_orb_pose_error[1]:.4f}, Z: {rmse_orb_pose_error[2]:.4f}, Overall: {rmse_orb_pose_error[3]:.4f}")

    # Camera position settings for 3D plot
    camera_azimuth = 45
    camera_elevation = 55

    # Calculate global data ranges for all trajectories to ensure consistent axis bounds
    all_x_global = np.concatenate([x_des, x_orb_est, x_orb_act])
    all_y_global = np.concatenate([y_des, y_orb_est, y_orb_act])
    all_z_global = np.concatenate([z_des, z_orb_est, z_orb_act])
    
    # Calculate ranges for consistent scaling
    x_range_global = np.max(all_x_global) - np.min(all_x_global)
    y_range_global = np.max(all_y_global) - np.min(all_y_global)
    z_range_global = np.max(all_z_global) - np.min(all_z_global)
    
    # Use the maximum range for all three axes to make them equal scale
    max_range_global = max(x_range_global, y_range_global, z_range_global)
    
    # Set equal limits for all axes using the same scale (global)
    x_center_global = (np.max(all_x_global) + np.min(all_x_global)) / 2
    y_center_global = (np.max(all_y_global) + np.min(all_y_global)) / 2
    z_center_global = (np.max(all_z_global) + np.min(all_z_global)) / 2

    # Create figure: ORB-SLAM comparison
    fig2 = plt.figure(figsize=FIGURE_SIZE_3D)
    ax2 = fig2.add_subplot(111, projection='3d')
    
    # Plot desired trajectory and ORB-SLAM trajectories
    ax2.plot(x_des, y_des, z_des, color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
             linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL, alpha=0.8)
    # ax2.plot(x_orb_est, y_orb_est, z_orb_est, color=ORBSLAM_ESTIMATED_COLOR, linestyle='-', 
    #          linewidth=LINE_WIDTH, label=ORBSLAM_ESTIMATED_LABEL, alpha=0.9)
    ax2.plot(x_orb_act, y_orb_act, z_orb_act, color=ORBSLAM_ACTUAL_COLOR, linestyle='-', 
             linewidth=LINE_WIDTH, label=ORBSLAM_ACTUAL_LABEL, alpha=0.9)

    # Set labels and formatting for second plot
    ax2.set_xlabel('X (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax2.set_ylabel('Y (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax2.set_zlabel('Z (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax2.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax2.tick_params(axis='z', which='major', pad=TICK_PAD_Z)
    
    # Control tick density
    from matplotlib.ticker import MaxNLocator
    ax2.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax2.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax2.zaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    
    # Add subplot label
    ax2.text2D(0.0, 0.9, '(a)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
               fontweight='bold', verticalalignment='bottom')

    # Use the same global bounds for consistency between plots
    ax2.set_xlim([x_center_global - max_range_global/2, x_center_global + max_range_global/2])
    ax2.set_ylim([y_center_global - max_range_global/2, y_center_global + max_range_global/2])
    ax2.set_zlim([z_center_global - max_range_global/2, z_center_global + max_range_global/2])
    
    # Set camera position and formatting
    ax2.view_init(elev=camera_elevation, azim=camera_azimuth)
    ax2.set_box_aspect([1,1,1])  # Equal aspect ratio for all three axes
    ax2.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Create single combined legend for both 3D plots
    fig_legend = plt.figure(figsize=(12, 1.5))
    ax_legend = fig_legend.add_subplot(111)
    ax_legend.axis('off')
    
    # Create dummy plots for all trajectory types
    ax_legend.plot([], [], color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
                   linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL)
    # ax_legend.plot([], [], color=MOTION_CAPTURE_COLOR, linestyle='-', 
    #                linewidth=LINE_WIDTH, label=MOTION_CAPTURE_LABEL)
    # ax_legend.plot([], [], color=ORBSLAM_ESTIMATED_COLOR, linestyle='-', 
    #                linewidth=LINE_WIDTH, label=ORBSLAM_ESTIMATED_LABEL)
    ax_legend.plot([], [], color=ORBSLAM_ACTUAL_COLOR, linestyle='-', 
                   linewidth=LINE_WIDTH, label=ORBSLAM_ACTUAL_LABEL)
    
    # Create the combined legend
    legend = ax_legend.legend(fontsize=LEGEND_SIZE, ncol=2, loc='center', frameon=False)
    plt.tight_layout()
    
    # Create third figure: Error analysis over time
    fig3 = plt.figure(figsize=FIGURE_SIZE_2D)
    ax3 = fig3.add_subplot(111)
    
    # Calculate position errors over time using final trajectories
    # error_mc = np.linalg.norm(motion_capture_result[:, :3] - desired_trajectory[:, :3], axis=1)
    # error_orb_est = np.linalg.norm(orbslam_estimated[:, :3] - desired_trajectory[:, :3], axis=1)
    error_orb_act = np.linalg.norm(orbslam_actual[:, :3] - desired_trajectory[:, :3], axis=1)
    error_orb_pose = np.linalg.norm(orbslam_estimated[:, :3] - orbslam_actual[:, :3], axis=1)  # ORB-SLAM pose error (for terminal output only)
    
    # Plot error over time (excluding ORB-SLAM pose error)
    # ax3.plot(time_steps, error_mc, color=MOTION_CAPTURE_COLOR, linestyle='-', 
    #          linewidth=LINE_WIDTH, alpha=1.0)
    # ax3.plot(time_steps, error_orb_est, color=ORBSLAM_ESTIMATED_COLOR, linestyle='-', 
    #          linewidth=LINE_WIDTH, alpha=1.0)
    ax3.plot(time_steps, error_orb_act, color=ORBSLAM_ACTUAL_COLOR, linestyle='-', 
             linewidth=LINE_WIDTH, alpha=1.0)
    
    # Set labels and formatting
    ax3.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax3.set_ylabel('Position Error (m)', fontsize=AXIS_LABEL_SIZE)
    
    # Add subplot label
    ax3.text(-0.05, 1.01, '(b)', transform=ax3.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
             fontweight='bold', verticalalignment='bottom')
    ax3.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    
    # Control tick density  
    from matplotlib.ticker import MaxNLocator
    ax3.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax3.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    
    # Set y-axis to start from 0
    ax3.set_ylim(bottom=0)
    
    ax3.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Save the plots as PDFs
    fig2.savefig('motion_capture_trajectory_main.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.4)
    fig_legend.savefig('motion_capture_trajectory_legend.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.1)
    fig3.savefig('motion_capture_trajectory_analysis.pdf', format='pdf', dpi=400, bbox_inches='tight', pad_inches=0.1)
    
    print("\nPlots saved as:")
    print("  - 'orbslam_3D_comparison.pdf'")
    print("  - '3D_trajectories_legend.pdf'")
    print("  - 'trajectory_error_analysis.pdf'")
    
    plt.show()
    

if __name__ == "__main__":
    main()
