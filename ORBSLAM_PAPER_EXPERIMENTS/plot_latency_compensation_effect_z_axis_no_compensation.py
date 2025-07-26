import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 8*30

# Plot settings
FIGURE_SIZE = (1, 4)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 38
AXIS_LABEL_SIZE = 30
TICK_LABEL_SIZE = 26
LEGEND_SIZE = 26
LINE_WIDTH = 3
GRID_ALPHA = 0.3

# Colors - Custom palette (converted from D3.js)
# Palette: ['#00429d', '#415395', '#59668a', '#657b7d', '#68926c', '#5dab55', '#31c52f']
DESIRED_HEIGHT_COLOR = '#657b7d'        # Dark blue - reference line
OBSERVED_STATE_COLOR = '#008e00'        # Medium blue - observed state
ESTIMATED_STATE_COLOR = '#415395'       # Blue-gray - estimated state  
MOTION_CAPTURE_COLOR = '#e97d00'        # Green - motion capture data
LATENCY_COLOR = '#008e00'               # Gray-green - latency compensation enabled
NO_DATA_TEXT_SIZE = 48

# Line styles - Different patterns for black/white distinction
DESIRED_HEIGHT_STYLE = ':'              # Dotted for reference
OBSERVED_STATE_STYLE = '-'              # Solid for observed data
ESTIMATED_STATE_STYLE = '--'            # Dashed for estimated data
MOTION_CAPTURE_STYLE = '-'              # Solid for motion capture data
LATENCY_STYLE = '-'                     # Solid for primary data

# Labels - All text labels used in the plots
DESIRED_HEIGHT_LABEL = 'Desired Height'
UKF_HEIGHT_LABEL = 'UKF State Height'
MOTION_CAPTURE_LABEL = 'Motion Capture'
LATENCY_LABEL = 'Delay State Estimation'

# Plot-specific labels
OBSERVED_HEIGHT_LABEL = 'Observed Height'
ESTIMATED_HEIGHT_LABEL = 'Estimated Height'  
MOTION_CAPTURE_HEIGHT_LABEL = 'Motion Capture Height'
LATENCY_ENABLED_LABEL = 'Delay State Est.'
THRUST_RATIO_LABEL = 'Est. Thrust Ratio'
OBSERVED_VELOCITY_LABEL = 'Observed Velocity'
ESTIMATED_VELOCITY_LABEL = 'Estimated Velocity'
MOTION_CAPTURE_VELOCITY_LABEL = 'Motion Capture Velocity'

# Axis labels
HEIGHT_YLABEL = 'z pos (m)'
LATENCY_YLABEL = 'Est. Delay'
VELOCITY_YLABEL = 'z vel (m/s)'
THRUST_RATIO_YLABEL = 'Est. Ratio'
XLABEL = 'Time (s)'

# Error messages
NO_LATENCY_DATA_MSG = 'No latency estimation data available'
NO_VELOCITY_DATA_MSG = 'No velocity data available'
NO_THRUST_DATA_MSG = 'No thrust ratio data available'

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
    #latency_estimation_dir = 'BIGQUAD_HOVER_3_LATENCY_FROM_1'
    latency_estimation_dir = 'BIGQUAD_HOVER_NO_LATENCY_COMPENSATION'
    
    # Load data for latency compensation enabled trial
    print("Loading latency compensation enabled data...")
    data_latency_enabled = load_trial_data(latency_estimation_dir)
    
    # Create a figure with 4x1 grid of subplots (4 rows, 1 column) with equal heights of 0.75
    figure, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 24), 
                                               gridspec_kw={'height_ratios': [1.0, 1.0, 0.75, 0.75]})
    
    # Add subplot labels (a), (b), (c), (d) for academic papers
    ax1.text(-0.1, 1.05, '(a)', transform=ax1.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax2.text(-0.1, 1.05, '(b)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax3.text(-0.1, 1.05, '(c)', transform=ax3.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax4.text(-0.1, 1.05, '(d)', transform=ax4.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')

    def plot_height_comparison(ax, data_enabled):
        """Helper function to plot height comparison data for latency compensation enabled condition"""
        # Plot data for latency compensation enabled
        trajectory_height_enabled = None
        if data_enabled['trajectory'] is not None:
            traj = data_enabled['trajectory'].T
            if len(traj) >= 3:
                if data_enabled['observed_state_history'] is not None:
                    trajectory_height_enabled = traj[2, :min(len(data_enabled['observed_state_history']), MAX_TIMESTEPS)] if len(traj[2]) >= min(len(data_enabled['observed_state_history']), MAX_TIMESTEPS) else traj[2][:MAX_TIMESTEPS]
                else:
                    trajectory_height_enabled = traj[2][:MAX_TIMESTEPS]

        if trajectory_height_enabled is not None:
            time_traj = np.arange(len(trajectory_height_enabled)) / 30.0
            ax.plot(time_traj, trajectory_height_enabled, label=DESIRED_HEIGHT_LABEL, color=DESIRED_HEIGHT_COLOR, 
                   linestyle=DESIRED_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        # Get reference data length from observed_state_history for alignment
        reference_length = len(data_enabled['observed_state_history']) if data_enabled['observed_state_history'] is not None else MAX_TIMESTEPS
        actual_length = min(reference_length, MAX_TIMESTEPS)
        
        observed_height = None
        estimated_height = None
        motion_capture_height = None
        
        # Plot observed state height
        if data_enabled['observed_state_history'] is not None and data_enabled['observed_state_history'].shape[1] >= 3:
            observed_height = data_enabled['observed_state_history'][:actual_length, 2]
            time_observed = np.arange(len(observed_height)) / 30.0
            ax.plot(time_observed, observed_height, label=OBSERVED_HEIGHT_LABEL, color=OBSERVED_STATE_COLOR, 
                   linestyle=OBSERVED_STATE_STYLE, linewidth=LINE_WIDTH)
        
        # Plot estimated state height
        if data_enabled['estimated_state_history'] is not None and data_enabled['estimated_state_history'].shape[1] >= 3:
            estimated_height = data_enabled['estimated_state_history'][:actual_length, 2]
            time_estimated = np.arange(len(estimated_height)) / 30.0
        
        # Plot motion capture data - align with observed state length and apply offset correction
        if data_enabled['motion_capture_history'] is not None and data_enabled['motion_capture_history'].shape[1] >= 3:
            motion_capture_height_raw = data_enabled['motion_capture_history'][:actual_length, 2]
            
            # Apply offset correction to align motion capture with observed state initial value
            if observed_height is not None and len(motion_capture_height_raw) > 0 and len(observed_height) > 0:
                # Calculate offset using the first valid values
                offset = observed_height[0] - motion_capture_height_raw[0]
                motion_capture_height = motion_capture_height_raw + offset
            else:
                motion_capture_height = motion_capture_height_raw
            
            time_motion_capture = np.arange(len(motion_capture_height)) / 30.0
            ax.plot(time_motion_capture, motion_capture_height, label=MOTION_CAPTURE_HEIGHT_LABEL, color=MOTION_CAPTURE_COLOR, 
                   linestyle=MOTION_CAPTURE_STYLE, linewidth=LINE_WIDTH)
        
        ax.set_ylabel(HEIGHT_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    def plot_latency_estimation(ax, data_enabled):
        """Helper function to plot delay state estimation for latency compensation enabled condition"""
        # Get reference data length from observed_state_history for alignment
        reference_length = len(data_enabled['observed_state_history']) if data_enabled['observed_state_history'] is not None else MAX_TIMESTEPS
        actual_length = min(reference_length, MAX_TIMESTEPS)
        
        if data_enabled['delay_state_estimation_history'] is not None:
            delay_estimation_enabled = data_enabled['delay_state_estimation_history'].flatten()[:actual_length] - 1
            time_enabled = np.arange(len(delay_estimation_enabled)) / 30.0
            ax.plot(time_enabled, delay_estimation_enabled, label=LATENCY_ENABLED_LABEL, color=LATENCY_COLOR, 
                   linestyle=LATENCY_STYLE, linewidth=LINE_WIDTH)
        
        if data_enabled['delay_state_estimation_history'] is None:
            ax.text(0.5, 0.5, NO_LATENCY_DATA_MSG, 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel(LATENCY_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)
        ax.set_ylim(-1, 5)  # Set y-axis limits from 0 to 6

    def plot_vertical_velocity(ax, data_enabled):
        """Helper function to plot vertical velocity (z dot) for latency compensation enabled condition"""
        # Get reference data length from observed_state_history for alignment
        reference_length = len(data_enabled['observed_state_history']) if data_enabled['observed_state_history'] is not None else MAX_TIMESTEPS
        actual_length = min(reference_length, MAX_TIMESTEPS)
        
        observed_velocity_z = None
        estimated_velocity_z = None
        motion_capture_velocity_z = None
        
        # Plot observed state vertical velocity data
        if data_enabled['observed_state_history'] is not None and data_enabled['observed_state_history'].shape[1] >= 10:
            # Assuming structure: position (0-2), quaternion (3-6), linear velocity (7-9), angular velocity (10+)
            # So vz (vertical velocity) is column 9 (index 9)
            observed_velocity_z = data_enabled['observed_state_history'][:actual_length, 9]
            time_observed = np.arange(len(observed_velocity_z)) / 30.0
            ax.plot(time_observed, observed_velocity_z, label=OBSERVED_VELOCITY_LABEL, color=OBSERVED_STATE_COLOR, 
                   linestyle=OBSERVED_STATE_STYLE, linewidth=LINE_WIDTH)
        
        # Plot estimated state vertical velocity data
        if data_enabled['estimated_state_history'] is not None and data_enabled['estimated_state_history'].shape[1] >= 10:
            # Assuming structure: position (0-2), quaternion (3-6), linear velocity (7-9), angular velocity (10+)
            # So vz (vertical velocity) is column 9 (index 9)
            estimated_velocity_z = data_enabled['estimated_state_history'][:actual_length, 9]
            time_estimated = np.arange(len(estimated_velocity_z)) / 30.0
        
        # Plot motion capture vertical velocity data - align with observed state length and apply offset correction
        if data_enabled['motion_capture_history'] is not None and data_enabled['motion_capture_history'].shape[1] >= 10:
            # Assuming motion capture has same structure: position (0-2), quaternion (3-6), linear velocity (7-9)
            # So vz (vertical velocity) is column 9 (index 9)
            motion_capture_velocity_z_raw = data_enabled['motion_capture_history'][:actual_length, 9]
            
            # Apply offset correction to align motion capture with observed state initial value
            if observed_velocity_z is not None and len(motion_capture_velocity_z_raw) > 0 and len(observed_velocity_z) > 0:
                # Calculate offset using the first valid values
                offset = observed_velocity_z[0] - motion_capture_velocity_z_raw[0]
                motion_capture_velocity_z = motion_capture_velocity_z_raw + offset
            else:
                motion_capture_velocity_z = motion_capture_velocity_z_raw
            
            time_motion_capture = np.arange(len(motion_capture_velocity_z)) / 30.0
            ax.plot(time_motion_capture, motion_capture_velocity_z, label=MOTION_CAPTURE_VELOCITY_LABEL, color=MOTION_CAPTURE_COLOR, 
                   linestyle=MOTION_CAPTURE_STYLE, linewidth=LINE_WIDTH)
        
        # Check if no data is available
        has_observed_data = (data_enabled['observed_state_history'] is not None and 
                            data_enabled['observed_state_history'].shape[1] >= 10)
        has_estimated_data = (data_enabled['estimated_state_history'] is not None and 
                             data_enabled['estimated_state_history'].shape[1] >= 10)
        has_motion_capture_data = (data_enabled['motion_capture_history'] is not None and 
                                  data_enabled['motion_capture_history'].shape[1] >= 10)
        
        if not has_observed_data and not has_estimated_data and not has_motion_capture_data:
            ax.text(0.5, 0.5, NO_VELOCITY_DATA_MSG, 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel(VELOCITY_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    def plot_thrust_ratio(ax, data_enabled):
        """Helper function to plot estimated thrust ratio for latency compensation enabled condition"""
        # Get reference data length from observed_state_history for alignment
        reference_length = len(data_enabled['observed_state_history']) if data_enabled['observed_state_history'] is not None else MAX_TIMESTEPS
        actual_length = min(reference_length, MAX_TIMESTEPS)
        
        if data_enabled['parameter_estimation_history'] is not None:
            # Convert parameter estimation to thrust ratio by dividing by 9.81 (gravity)
            thrust_ratio_enabled = data_enabled['parameter_estimation_history'].flatten()[:actual_length] / 9.81
            time_enabled = np.arange(len(thrust_ratio_enabled)) / 30.0
            ax.plot(time_enabled, thrust_ratio_enabled, label=THRUST_RATIO_LABEL, color=LATENCY_COLOR, 
                   linestyle=LATENCY_STYLE, linewidth=LINE_WIDTH)
        
        if data_enabled['parameter_estimation_history'] is None:
            ax.text(0.5, 0.5, NO_THRUST_DATA_MSG, 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel(THRUST_RATIO_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    # Plot all comparisons
    plot_height_comparison(ax1, data_latency_enabled)
    plot_vertical_velocity(ax2, data_latency_enabled)
    plot_latency_estimation(ax3, data_latency_enabled)
    plot_thrust_ratio(ax4, data_latency_enabled)

    # Adjust layout to prevent overlapping
    plt.tight_layout()
    
    # Save the plot as a PDF
    plt.savefig('latency_comparison_no_compensation.pdf', format='pdf', dpi=400, bbox_inches='tight')
    print("Plot saved as 'latency_comparison_no_compensation.pdf'")
    
    # Show the plot
    plt.show()



if __name__ == "__main__":
    main()
