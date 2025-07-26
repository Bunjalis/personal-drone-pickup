import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 8*30  # Adjusted for your data length

# Plot settings
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
ERROR_WITH_COMPENSATION_COLOR = '#00429d'    # Dark blue - observed state error
ERROR_WITHOUT_COMPENSATION_COLOR = '#008e00'   # Red - estimated state error

# Line styles
OBSERVED_ERROR_STYLE = '-'          # Solid for observed
ESTIMATED_ERROR_STYLE = '-'         # Solid for estimated

# Labels
POSE_ERROR_LABEL = 'Total Pose Error (Position + Orientation)'
VELOCITY_ERROR_LABEL = 'Total Velocity Error (Linear + Angular)'

# Axis labels
XLABEL = 'Time (s)'
POSE_ERROR_YLABEL = 'Est. Pose Error'
VELOCITY_ERROR_YLABEL = 'Est. Velocity Error'

# Dataset name
with_compensation_dataset = 'BIGQUAD_HOVER_3_LATENCY_FROM_1'
without_compensation_dataset = 'BIGQUAD_HOVER_NO_LATENCY_COMPENSATION'

def align_motion_capture_data(motion_capture_data, reference_data):
    """Align motion capture data with reference data using first position offset"""
    if motion_capture_data is None or reference_data is None:
        return motion_capture_data
    
    # Calculate offset for position (first 3 columns: x, y, z)
    if len(motion_capture_data) > 0 and len(reference_data) > 0:
        position_offset = reference_data[0, :3] - motion_capture_data[0, :3]
        
        # Apply offset to motion capture position
        aligned_data = motion_capture_data.copy()
        aligned_data[:, :3] += position_offset
        
        return aligned_data
    
    return motion_capture_data

def calculate_pose_error_norm(estimated_state, motion_capture_state):
    """Calculate the L2 norm of total pose error (position + orientation) between estimated and motion capture states"""
    if estimated_state is None or motion_capture_state is None:
        return None
    
    # Ensure same length
    min_len = min(len(estimated_state), len(motion_capture_state))
    
    # Position error (columns 0-2: x, y, z)
    estimated_pos = estimated_state[:min_len, :3]
    motion_capture_pos = motion_capture_state[:min_len, :3]
    position_errors = estimated_pos - motion_capture_pos
    position_error_norm = np.linalg.norm(position_errors, axis=1)
    
    # Orientation error (columns 3-6: quaternion qw, qx, qy, qz)
    # For quaternion error, we calculate the angular distance
    estimated_quat = estimated_state[:min_len, 3:7]  # qw, qx, qy, qz
    motion_capture_quat = motion_capture_state[:min_len, 3:7]  # qw, qx, qy, qz
    
    # Calculate quaternion error using dot product and angular distance
    orientation_errors = np.zeros(min_len)
    for i in range(min_len):
        q1 = estimated_quat[i]
        q2 = motion_capture_quat[i]
        
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Calculate angular distance between quaternions
        dot_product = np.abs(np.dot(q1, q2))
        dot_product = np.clip(dot_product, 0, 1)  # Ensure valid range for arccos
        orientation_errors[i] = 2 * np.arccos(dot_product)  # Angular distance in radians
    
    # Combine position and orientation errors (weighted combination)
    # Position error in meters, orientation error in radians
    # Scale orientation error to make it comparable to position error
    orientation_weight = 1.0  # Adjust this weight as needed
    total_pose_error = position_error_norm + orientation_weight * orientation_errors
    
    return total_pose_error

def calculate_velocity_error_norm(estimated_state, motion_capture_state):
    """Calculate the L2 norm of total velocity error (linear + angular) between estimated and motion capture states"""
    if estimated_state is None or motion_capture_state is None:
        return None
    
    # Check if velocity data is available (assuming structure: pos(0-2), quat(3-6), lin_vel(7-9), ang_vel(10-12))
    if estimated_state.shape[1] < 13 or motion_capture_state.shape[1] < 13:
        print("Warning: Full velocity data not available, using available columns")
        return None
    
    # Ensure same length
    min_len = min(len(estimated_state), len(motion_capture_state))
    
    # Linear velocity error (columns 7-9: vx, vy, vz)
    estimated_lin_vel = estimated_state[:min_len, 7:10]
    motion_capture_lin_vel = motion_capture_state[:min_len, 7:10]
    linear_velocity_errors = estimated_lin_vel - motion_capture_lin_vel
    linear_velocity_error_norm = np.linalg.norm(linear_velocity_errors, axis=1)
    
    # Angular velocity error (columns 10-12: wx, wy, wz)
    estimated_ang_vel = estimated_state[:min_len, 10:13]
    motion_capture_ang_vel = motion_capture_state[:min_len, 10:13]
    angular_velocity_errors = estimated_ang_vel - motion_capture_ang_vel
    angular_velocity_error_norm = np.linalg.norm(angular_velocity_errors, axis=1)
    
    # Combine linear and angular velocity errors (weighted combination)
    # Linear velocity in m/s, angular velocity in rad/s
    # Scale angular velocity error to make it comparable to linear velocity error
    angular_weight = 0.1  # Adjust this weight as needed
    total_velocity_error = linear_velocity_error_norm + angular_weight * angular_velocity_error_norm
    
    return total_velocity_error


def calculate_individual_errors_over_time(estimated_state, motion_capture_state):
    """Calculate individual error components over time: position, orientation, linear velocity, angular velocity"""
    if estimated_state is None or motion_capture_state is None:
        return None, None, None, None
    
    # Ensure same length
    min_len = min(len(estimated_state), len(motion_capture_state))
    
    # Position errors (columns 0-2: x, y, z)
    estimated_pos = estimated_state[:min_len, :3]
    motion_capture_pos = motion_capture_state[:min_len, :3]
    position_errors = estimated_pos - motion_capture_pos
    position_error_norm = np.linalg.norm(position_errors, axis=1)  # L2 norm for each timestep
    
    # Orientation errors (columns 3-6: quaternion qw, qx, qy, qz)
    estimated_quat = estimated_state[:min_len, 3:7]
    motion_capture_quat = motion_capture_state[:min_len, 3:7]
    
    orientation_errors = np.zeros(min_len)
    for i in range(min_len):
        q1 = estimated_quat[i]
        q2 = motion_capture_quat[i]
        
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Calculate angular distance between quaternions
        dot_product = np.abs(np.dot(q1, q2))
        dot_product = np.clip(dot_product, 0, 1)
        orientation_errors[i] = 2 * np.arccos(dot_product)
    
    # Keep orientation errors in radians
    orientation_errors_rad = orientation_errors
    
    # Linear velocity errors (columns 7-9: vx, vy, vz)
    linear_velocity_errors = None
    if estimated_state.shape[1] >= 10 and motion_capture_state.shape[1] >= 10:
        estimated_lin_vel = estimated_state[:min_len, 7:10]
        motion_capture_lin_vel = motion_capture_state[:min_len, 7:10]
        lin_vel_errors = estimated_lin_vel - motion_capture_lin_vel
        linear_velocity_errors = np.linalg.norm(lin_vel_errors, axis=1)  # L2 norm for each timestep
    
    # Angular velocity errors (columns 10-12: wx, wy, wz)
    angular_velocity_errors = None
    if estimated_state.shape[1] >= 13 and motion_capture_state.shape[1] >= 13:
        estimated_ang_vel = estimated_state[:min_len, 10:13]
        motion_capture_ang_vel = motion_capture_state[:min_len, 10:13]
        ang_vel_errors = estimated_ang_vel - motion_capture_ang_vel
        angular_velocity_errors = np.linalg.norm(ang_vel_errors, axis=1)  # L2 norm for each timestep in rad/s
        
        # Filter out angular velocity errors > 300 degrees (convert to radians: 300 deg = 5.236 rad)
        angular_velocity_errors_filtered = angular_velocity_errors.copy()
        outlier_threshold = np.radians(300)  # 300 degrees in radians
        
        for i in range(len(angular_velocity_errors_filtered)):
            if angular_velocity_errors_filtered[i] > outlier_threshold:
                if i > 0:
                    angular_velocity_errors_filtered[i] = angular_velocity_errors_filtered[i-1]
                else:
                    # If first measurement is outlier, find first valid measurement
                    for j in range(1, len(angular_velocity_errors_filtered)):
                        if angular_velocity_errors_filtered[j] <= outlier_threshold:
                            angular_velocity_errors_filtered[i] = angular_velocity_errors_filtered[j]
                            break
        
        angular_velocity_errors = angular_velocity_errors_filtered
    
    return position_error_norm, orientation_errors_rad, linear_velocity_errors, angular_velocity_errors



def calculate_individual_rmse(estimated_state, motion_capture_state):
    """Calculate RMSE for individual components: position, orientation, linear velocity, angular velocity"""
    # Use the error calculation function and then compute RMSE
    pos_errors, ori_errors, lin_vel_errors, ang_vel_errors = calculate_individual_errors_over_time(estimated_state, motion_capture_state)
    
    pos_rmse = np.sqrt(np.mean(pos_errors**2)) if pos_errors is not None else None
    ori_rmse = np.sqrt(np.mean(ori_errors**2)) if ori_errors is not None else None
    lin_vel_rmse = np.sqrt(np.mean(lin_vel_errors**2)) if lin_vel_errors is not None else None
    ang_vel_rmse = np.sqrt(np.mean(ang_vel_errors**2)) if ang_vel_errors is not None else None
    
    return pos_rmse, ori_rmse, lin_vel_rmse, ang_vel_rmse


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
    # Set up the paths to both datasets
    base_path = os.path.dirname(os.path.abspath(__file__))
    with_compensation_path = os.path.join(base_path, with_compensation_dataset)
    without_compensation_path = os.path.join(base_path, without_compensation_dataset)
    
    # Load both datasets
    print("Loading with compensation data...")
    with_comp_data = load_trial_data(with_compensation_path)
    print("Loading without compensation data...")
    without_comp_data = load_trial_data(without_compensation_path)
    
    # Create a figure with 4x1 grid of subplots (4 rows, 1 column)
    figure, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 24))
    
    # Add subplot labels (a), (b), (c), (d) for academic papers
    ax1.text(-0.1, 1.05, '(a)', transform=ax1.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax2.text(-0.1, 1.05, '(b)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax3.text(-0.1, 1.05, '(c)', transform=ax3.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax4.text(-0.1, 1.05, '(d)', transform=ax4.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    
    # Process with compensation data
    if with_comp_data['estimated_state_history'] is not None and with_comp_data['motion_capture_history'] is not None:
        # Align motion capture data with estimated state data
        aligned_mc_data_with = align_motion_capture_data(
            with_comp_data['motion_capture_history'][:MAX_TIMESTEPS], 
            with_comp_data['estimated_state_history'][:MAX_TIMESTEPS]
        )
        
        # Calculate individual error components over time
        pos_error_with, ori_error_with, lin_vel_error_with, ang_vel_error_with = calculate_individual_errors_over_time(
            with_comp_data['estimated_state_history'][:MAX_TIMESTEPS],
            aligned_mc_data_with
        )
        
        # Plot each error component
        if pos_error_with is not None:
            time_with = np.arange(len(pos_error_with)) / 30.0
            ax1.plot(time_with, pos_error_with, color=ERROR_WITH_COMPENSATION_COLOR, 
                    linestyle=OBSERVED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/ Latency Compensation', alpha=0.9)
        
        if ori_error_with is not None:
            time_with = np.arange(len(ori_error_with)) / 30.0
            ax2.plot(time_with, ori_error_with, color=ERROR_WITH_COMPENSATION_COLOR, 
                    linestyle=OBSERVED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/ Latency Compensation', alpha=0.9)
        
        if lin_vel_error_with is not None:
            time_with = np.arange(len(lin_vel_error_with)) / 30.0
            ax3.plot(time_with, lin_vel_error_with, color=ERROR_WITH_COMPENSATION_COLOR, 
                    linestyle=OBSERVED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/ Latency Compensation', alpha=0.9)
        
        if ang_vel_error_with is not None:
            time_with = np.arange(len(ang_vel_error_with)) / 30.0
            ax4.plot(time_with, ang_vel_error_with, color=ERROR_WITH_COMPENSATION_COLOR, 
                    linestyle=OBSERVED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/ Latency Compensation', alpha=0.9)
    
    # Process without compensation data
    if without_comp_data['estimated_state_history'] is not None and without_comp_data['motion_capture_history'] is not None:
        # Align motion capture data with estimated state data
        aligned_mc_data_without = align_motion_capture_data(
            without_comp_data['motion_capture_history'][:MAX_TIMESTEPS], 
            without_comp_data['estimated_state_history'][:MAX_TIMESTEPS]
        )
        
        # Calculate individual error components over time
        pos_error_without, ori_error_without, lin_vel_error_without, ang_vel_error_without = calculate_individual_errors_over_time(
            without_comp_data['estimated_state_history'][:MAX_TIMESTEPS],
            aligned_mc_data_without
        )
        
        # Plot each error component
        if pos_error_without is not None:
            time_without = np.arange(len(pos_error_without)) / 30.0
            ax1.plot(time_without, pos_error_without, color=ERROR_WITHOUT_COMPENSATION_COLOR, 
                    linestyle=ESTIMATED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/o Latency Compensation', alpha=0.9)
        
        if ori_error_without is not None:
            time_without = np.arange(len(ori_error_without)) / 30.0
            ax2.plot(time_without, ori_error_without, color=ERROR_WITHOUT_COMPENSATION_COLOR, 
                    linestyle=ESTIMATED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/o Latency Compensation', alpha=0.9)
        
        if lin_vel_error_without is not None:
            time_without = np.arange(len(lin_vel_error_without)) / 30.0
            ax3.plot(time_without, lin_vel_error_without, color=ERROR_WITHOUT_COMPENSATION_COLOR, 
                    linestyle=ESTIMATED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/o Latency Compensation', alpha=0.9)
        
        if ang_vel_error_without is not None:
            time_without = np.arange(len(ang_vel_error_without)) / 30.0
            ax4.plot(time_without, ang_vel_error_without, color=ERROR_WITHOUT_COMPENSATION_COLOR, 
                    linestyle=ESTIMATED_ERROR_STYLE, linewidth=LINE_WIDTH, 
                    label='w/o Latency Compensation', alpha=0.9)
    
    # Configure subplot 1 (Position Error)
    ax1.set_ylabel('Position\nError (m)', fontsize=AXIS_LABEL_SIZE)
    ax1.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax1.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax1.legend(fontsize=LEGEND_SIZE)
    ax1.grid(True, alpha=GRID_ALPHA)
    
    # Configure subplot 2 (Orientation Error)
    ax2.set_ylabel('Orientation\nError (rad)', fontsize=AXIS_LABEL_SIZE)
    ax2.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax2.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax2.legend(fontsize=LEGEND_SIZE)
    ax2.grid(True, alpha=GRID_ALPHA)
    
    # Configure subplot 3 (Linear Velocity Error)
    ax3.set_ylabel('Linear Vel.\nError (m/s)', fontsize=AXIS_LABEL_SIZE)
    ax3.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax3.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax3.legend(fontsize=LEGEND_SIZE)
    ax3.grid(True, alpha=GRID_ALPHA)
    
    # Configure subplot 4 (Angular Velocity Error)
    ax4.set_ylabel('Angular Vel.\nError (rad/s)', fontsize=AXIS_LABEL_SIZE)
    ax4.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax4.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    ax4.legend(fontsize=LEGEND_SIZE)
    ax4.grid(True, alpha=GRID_ALPHA)
    
    # Control the number of ticks on 2D plots
    from matplotlib.ticker import MaxNLocator
    ax1.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax1.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax2.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax2.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax3.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax3.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax4.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax4.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    
    # Adjust layout to prevent overlapping
    plt.tight_layout()
    
    # Save the plot as a PDF
    plt.savefig('MPC_input_state_error_comparison.pdf', format='pdf', dpi=400, bbox_inches='tight')
    print("Plot saved as 'MPC_input_state_error_comparison.pdf'")

    # Calculate and print individual RMSE statistics
    print("\n" + "="*70)
    print("INDIVIDUAL COMPONENT RMSE ANALYSIS")
    print("="*70)
    
    # Calculate individual RMSE for with compensation
    if with_comp_data['estimated_state_history'] is not None and with_comp_data['motion_capture_history'] is not None:
        aligned_mc_data_with = align_motion_capture_data(
            with_comp_data['motion_capture_history'][:MAX_TIMESTEPS], 
            with_comp_data['estimated_state_history'][:MAX_TIMESTEPS]
        )
        
        pos_rmse_with, ori_rmse_with, lin_vel_rmse_with, ang_vel_rmse_with = calculate_individual_rmse(
            with_comp_data['estimated_state_history'][:MAX_TIMESTEPS],
            aligned_mc_data_with
        )
        
        print(f"\nWITH LATENCY COMPENSATION:")
        if pos_rmse_with is not None:
            print(f"  Position RMSE: {pos_rmse_with:.4f} m")
        if ori_rmse_with is not None:
            print(f"  Orientation RMSE: {ori_rmse_with:.4f} rad")
        if lin_vel_rmse_with is not None:
            print(f"  Linear Velocity RMSE: {lin_vel_rmse_with:.4f} m/s")
        if ang_vel_rmse_with is not None:
            print(f"  Angular Velocity RMSE: {ang_vel_rmse_with:.4f} rad/s")
    
    # Calculate individual RMSE for without compensation
    if without_comp_data['estimated_state_history'] is not None and without_comp_data['motion_capture_history'] is not None:
        aligned_mc_data_without = align_motion_capture_data(
            without_comp_data['motion_capture_history'][:MAX_TIMESTEPS], 
            without_comp_data['estimated_state_history'][:MAX_TIMESTEPS]
        )
        
        pos_rmse_without, ori_rmse_without, lin_vel_rmse_without, ang_vel_rmse_without = calculate_individual_rmse(
            without_comp_data['estimated_state_history'][:MAX_TIMESTEPS],
            aligned_mc_data_without
        )
        
        print(f"\nWITHOUT LATENCY COMPENSATION:")
        if pos_rmse_without is not None:
            print(f"  Position RMSE: {pos_rmse_without:.4f} m")
        if ori_rmse_without is not None:
            print(f"  Orientation RMSE: {ori_rmse_without:.4f} rad")
        if lin_vel_rmse_without is not None:
            print(f"  Linear Velocity RMSE: {lin_vel_rmse_without:.4f} m/s")
        if ang_vel_rmse_without is not None:
            print(f"  Angular Velocity RMSE: {ang_vel_rmse_without:.4f} rad/s")
    
    # Calculate improvement percentages
    print(f"\nPERFORMANCE IMPROVEMENT (Reduction in RMSE):")
    if 'pos_rmse_with' in locals() and 'pos_rmse_without' in locals() and pos_rmse_with is not None and pos_rmse_without is not None:
        pos_improvement = ((pos_rmse_without - pos_rmse_with) / pos_rmse_without) * 100
        print(f"  Position RMSE Improvement: {pos_improvement:.1f}%")
    
    if 'ori_rmse_with' in locals() and 'ori_rmse_without' in locals() and ori_rmse_with is not None and ori_rmse_without is not None:
        ori_improvement = ((ori_rmse_without - ori_rmse_with) / ori_rmse_without) * 100
        print(f"  Orientation RMSE Improvement: {ori_improvement:.1f}%")
    
    if 'lin_vel_rmse_with' in locals() and 'lin_vel_rmse_without' in locals() and lin_vel_rmse_with is not None and lin_vel_rmse_without is not None:
        lin_vel_improvement = ((lin_vel_rmse_without - lin_vel_rmse_with) / lin_vel_rmse_without) * 100
        print(f"  Linear Velocity RMSE Improvement: {lin_vel_improvement:.1f}%")
    
    if 'ang_vel_rmse_with' in locals() and 'ang_vel_rmse_without' in locals() and ang_vel_rmse_with is not None and ang_vel_rmse_without is not None:
        ang_vel_improvement = ((ang_vel_rmse_without - ang_vel_rmse_with) / ang_vel_rmse_without) * 100
        print(f"  Angular Velocity RMSE Improvement: {ang_vel_improvement:.1f}%")

    print("="*70)
    
    # Show the plot
    plt.show()


if __name__ == "__main__":
    main()