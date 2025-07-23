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
LINE_WIDTH = 4
GRID_ALPHA = 0.3

# Tick settings - Control the number of ticks on axes
MAX_TICKS_3D = 5        # Maximum number of ticks per axis on 3D plot
MAX_TICKS_2D = 6        # Maximum number of ticks per axis on 2D plot

# 3D plot label positioning
LABEL_PAD_3D = 20       # Distance of axis labels from the axis (in points)
TICK_PAD_Z = 10         # Distance of z-axis tick labels from the axis (in points)

# Colors - Custom palette (converted from D3.js)
# Palette: ['#00429d', '#415395', '#59668a', '#657b7d', '#68926c', '#5dab55', '#31c52f']
DESIRED_TRAJECTORY_COLOR = '#657b7d'    # Gray-green - reference line
WITH_UKF_COLOR = '#00429d'              # Dark blue - with UKF
WITHOUT_UKF_COLOR = '#31c52f'           # Bright green - without UKF
PARAMETER_WITH_UKF_COLOR = '#415395'    # Medium blue - parameter estimation with UKF
PARAMETER_WITHOUT_UKF_COLOR = '#5dab55' # Medium green - parameter estimation without UKF

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

with_UKF_dataset = 'BIGQUAD_CIRCLE_2_WITH_UKF'
without_UKF_dataset = 'BIGQUAD_CIRCLE_3_WITHOUT_UKF'




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
    with_ukf_path = os.path.join(base_path, with_UKF_dataset)
    without_ukf_path = os.path.join(base_path, without_UKF_dataset)
    
    # Load both datasets
    ukf_data = load_trial_data(with_ukf_path)
    no_ukf_data = load_trial_data(without_ukf_path)
    
    # Extract data from UKF dataset
    ukf_states = ukf_data['UKF_state_estimation_history'][:-180]
    no_ukf_states = no_ukf_data['UKF_state_estimation_history'][:-180]
    desired_trajectory = ukf_data['trajectory'][:-180]
    
    # Extract parameter estimation data
    ukf_params = ukf_data['parameter_estimation_history'][:-180]
    no_ukf_params = no_ukf_data['parameter_estimation_history'][:-180]
    
    # Convert parameter estimation to thrust ratio by dividing by 9.81
    ukf_thrust_ratio = ukf_params / 9.81
    no_ukf_thrust_ratio = no_ukf_params / 9.81
    
    # Extract position coordinates (first 3 columns: x, y, z)
    # UKF dataset
    x_ukf, y_ukf, z_ukf = ukf_states[:, 0], ukf_states[:, 1], ukf_states[:, 2]
    x_no_ukf, y_no_ukf, z_no_ukf = no_ukf_states[:, 0], no_ukf_states[:, 1], no_ukf_states[:, 2]
    x_des, y_des, z_des = desired_trajectory[:, 0], desired_trajectory[:, 1], desired_trajectory[:, 2]

    # Create time vector for parameter estimation plot in seconds
    time_steps = np.arange(len(ukf_thrust_ratio)) / 30.0

    # Camera position settings for 3D plot (azimuth, elevation)
    # You can modify these values to change the default camera view
    camera_azimuth = -140  # Rotation around z-axis (degrees)
    camera_elevation = 20  # Angle above the xy-plane (degrees)

    # Create first figure: 3D trajectory plot
    fig1 = plt.figure(figsize=FIGURE_SIZE_3D)
    ax1 = fig1.add_subplot(111, projection='3d')
    
    # Plot trajectories with consistent styling
    ax1.plot(x_ukf, y_ukf, z_ukf, color=WITH_UKF_COLOR, linestyle=WITH_UKF_STYLE, 
             linewidth=LINE_WIDTH, label=WITH_UKF_LABEL, alpha=0.9)
    ax1.plot(x_no_ukf, y_no_ukf, z_no_ukf, color=WITHOUT_UKF_COLOR, linestyle=WITHOUT_UKF_STYLE, 
             linewidth=LINE_WIDTH, label=WITHOUT_UKF_LABEL, alpha=0.9)
    ax1.plot(x_des, y_des, z_des, color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
             linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL, alpha=0.8)

    # Set labels and title for 3D plot with consistent styling
    ax1.set_xlabel('X (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax1.set_ylabel('Y (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax1.set_zlabel('Z (m)', fontsize=AXIS_LABEL_SIZE, labelpad=LABEL_PAD_3D)
    ax1.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    
    # Move z-axis tick labels further away
    ax1.tick_params(axis='z', which='major', pad=TICK_PAD_Z)
    
    # Control the number of ticks on 3D plot
    from matplotlib.ticker import MaxNLocator
    ax1.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax1.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    ax1.zaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_3D))
    
    # Add subplot label (a) closer to the plot area
    ax1.text2D(0.0, 0.9, '(a)', transform=ax1.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
               fontweight='bold', verticalalignment='bottom')

    # Set camera position
    ax1.view_init(elev=camera_elevation, azim=camera_azimuth)
    ax1.set_box_aspect([1,1,1])
    ax1.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Create separate figure for the 3D plot legend
    fig_legend = plt.figure(figsize=(8, 2))
    ax_legend = fig_legend.add_subplot(111)
    ax_legend.axis('off')  # Hide the axes
    
    # Create dummy plots for the legend
    ax_legend.plot([], [], color=WITH_UKF_COLOR, linestyle=WITH_UKF_STYLE, 
                   linewidth=LINE_WIDTH, label=WITH_UKF_LABEL)
    ax_legend.plot([], [], color=WITHOUT_UKF_COLOR, linestyle=WITHOUT_UKF_STYLE, 
                   linewidth=LINE_WIDTH, label=WITHOUT_UKF_LABEL)
    ax_legend.plot([], [], color=DESIRED_TRAJECTORY_COLOR, linestyle=DESIRED_TRAJECTORY_STYLE, 
                   linewidth=LINE_WIDTH, label=DESIRED_TRAJECTORY_LABEL)
    
    # Create the legend
    legend = ax_legend.legend(fontsize=LEGEND_SIZE, ncol=3, loc='center', frameon=False)
    
    plt.tight_layout()
    
    # Create second figure: Parameter estimation plot
    fig2 = plt.figure(figsize=FIGURE_SIZE_2D)
    ax2 = fig2.add_subplot(111)
    
    # Plot parameter estimation history with consistent styling
    ax2.plot(time_steps, ukf_thrust_ratio, color=PARAMETER_WITH_UKF_COLOR, linestyle=WITH_UKF_STYLE, 
             linewidth=LINE_WIDTH, label=WITH_UKF_LABEL, alpha=1.0)
    ax2.plot(time_steps, no_ukf_thrust_ratio, color=PARAMETER_WITHOUT_UKF_COLOR, linestyle=WITHOUT_UKF_STYLE, 
             linewidth=LINE_WIDTH, label=WITHOUT_UKF_LABEL, alpha=1.0)
    
    # Set labels for parameter estimation plot with consistent styling
    ax2.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
    ax2.set_ylabel(PARAMETER_YLABEL, fontsize=AXIS_LABEL_SIZE)
    
    # Add subplot label (b) closer to the plot area
    ax2.text(-0.05, 1.01, '(b)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, 
             fontweight='bold', verticalalignment='bottom')
    ax2.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
    
    # Control the number of ticks on 2D plot
    ax2.xaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    ax2.yaxis.set_major_locator(MaxNLocator(nbins=MAX_TICKS_2D))
    
    ax2.legend(fontsize=LEGEND_SIZE)
    ax2.grid(True, alpha=GRID_ALPHA)
    
    plt.tight_layout()
    
    # Save the plots as PDFs
    fig1.savefig('UKF_3D_trajectory_comparison.pdf', format='pdf', dpi=400, bbox_inches='tight')
    fig_legend.savefig('UKF_3D_trajectory_legend.pdf', format='pdf', dpi=400, bbox_inches='tight')
    fig2.savefig('UKF_parameter_estimation_comparison.pdf', format='pdf', dpi=400, bbox_inches='tight')
    print("Plots saved as:")
    print("  - 'UKF_3D_trajectory_comparison.pdf'")
    print("  - 'UKF_3D_trajectory_legend.pdf'")
    print("  - 'UKF_parameter_estimation_comparison.pdf'")
    
    plt.show()
    

if __name__ == "__main__":
    main()
