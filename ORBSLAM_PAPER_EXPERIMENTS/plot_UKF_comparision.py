import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 500

# Plot settings
FIGURE_SIZE = (12, 18)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 36
AXIS_LABEL_SIZE = 28
TICK_LABEL_SIZE = 24
LEGEND_SIZE = 20
LINE_WIDTH = 4
GRID_ALPHA = 0.3
DPI = 600  # High resolution for journal publication

# Colors - Custom palette (converted from D3.js)
# Palette: ['#00429d', '#415395', '#59668a', '#657b7d', '#68926c', '#5dab55', '#31c52f']
DESIRED_HEIGHT_COLOR = '#657b7d'        # Gray-green - reference line
UKF_HEIGHT_COLOR = '#00429d'            # Dark blue - UKF enabled
THRUST_RATIO_COLOR = '#59668a'          # Blue-gray - UKF enabled
NO_UKF_HEIGHT_COLOR = '#31c52f'         # Bright green - UKF disabled
NO_UKF_THRUST_COLOR = '#5dab55'         # Medium green - UKF disabled
NO_DATA_TEXT_SIZE = 48

# Line styles - Different patterns for black/white distinction
DESIRED_HEIGHT_STYLE = ':'              # Dotted for reference
UKF_HEIGHT_STYLE = '-'                  # Solid for UKF enabled
THRUST_RATIO_STYLE = '-'                # Solid for UKF enabled
NO_UKF_STYLE = '-'                      # Solid for UKF disabled

# Labels - All text labels used in the plots
DESIRED_HEIGHT_LABEL = 'Desired Height'
UKF_HEIGHT_LABEL = 'UKF State Height'
THRUST_RATIO_LABEL = 'Estimated Thrust Ratio'

# Plot-specific labels
UKF_HEIGHT_ENABLED_LABEL = 'w/ UKF'
UKF_HEIGHT_DISABLED_LABEL = 'w/o UKF'
THRUST_RATIO_ENABLED_LABEL = 'w/ UKF'
THRUST_RATIO_DISABLED_LABEL = 'w/o UKF'

# Axis labels
HEIGHT_YLABEL = 'Height (m)'
THRUST_YLABEL = 'Est. Thrust Ratio'
XLABEL = 'Time Steps'

# Error messages
NO_THRUST_DATA_MSG = 'No thrust ratio data available'

# Additional labels for dual-dataset plotting
LABEL_HEIGHT_REFERENCE = 'Reference Height'
LABEL_HEIGHT_ESTIMATED_UKF = 'UKF Enabled'
LABEL_HEIGHT_ESTIMATED_NO_UKF = 'UKF Disabled'
LABEL_THRUST_UKF = 'UKF Enabled'
LABEL_THRUST_NO_UKF = 'UKF Disabled'
LABEL_SPEED_UKF = 'UKF Enabled'
LABEL_SPEED_NO_UKF = 'UKF Disabled'

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
    ukf_disabled_dir = 'BIGQUAD_HOVER_NO_UKF'
    ukf_enabled_dir = 'BIGQUAD_HOVER_2'
    
    # Load data for both trials
    print("Loading UKF enabled data...")
    data_ukf_enabled = load_trial_data(ukf_enabled_dir)
    
    print("Loading UKF disabled data...")
    data_ukf_disabled = load_trial_data(ukf_disabled_dir)
    
    # Create a figure with 3x1 grid of subplots (3 rows, 1 column)
    figure, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=FIGURE_SIZE)
    
    # Add subplot labels (a), (b), (c) for academic papers
    ax1.text(-0.1, 1.02, '(a)', transform=ax1.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax2.text(-0.1, 1.02, '(b)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax3.text(-0.1, 1.02, '(c)', transform=ax3.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')

    def plot_height_comparison(ax, data_ukf, data_no_ukf, label_ref, label_ukf, label_no_ukf):
        """Helper function to plot height comparison data for both datasets"""
        # Get trajectory data if available (use from UKF data as reference)
        trajectory_height = None
        if data_ukf['trajectory'] is not None:
            traj = data_ukf['trajectory'].T  # Transpose to match expected format
            if len(traj) >= 3:
                # Match trajectory length to state history if available
                if data_ukf['observed_state_history'] is not None:
                    trajectory_height = traj[2, :min(len(data_ukf['observed_state_history']), MAX_TIMESTEPS)] if len(traj[2]) >= min(len(data_ukf['observed_state_history']), MAX_TIMESTEPS) else traj[2][:MAX_TIMESTEPS]
                else:
                    trajectory_height = traj[2][:MAX_TIMESTEPS]

        # Plot desired trajectory height
        if trajectory_height is not None:
            ax.plot(trajectory_height, label=label_ref, color=DESIRED_HEIGHT_COLOR, 
                   linestyle=DESIRED_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        # Plot UKF state estimation height
        if data_ukf['UKF_state_estimation_history'] is not None and data_ukf['UKF_state_estimation_history'].shape[1] >= 3:
            ukf_height = data_ukf['UKF_state_estimation_history'][:MAX_TIMESTEPS, 2]
            ax.plot(ukf_height, label=label_ukf, color=UKF_HEIGHT_COLOR, 
                   linestyle=UKF_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        # Plot non-UKF height estimation (observed state)
        if data_no_ukf['observed_state_history'] is not None and data_no_ukf['observed_state_history'].shape[1] >= 3:
            no_ukf_height = data_no_ukf['observed_state_history'][:MAX_TIMESTEPS, 2]
            ax.plot(no_ukf_height, label=label_no_ukf, color=NO_UKF_HEIGHT_COLOR, 
                   linestyle=NO_UKF_STYLE, linewidth=LINE_WIDTH)
        
        ax.set_ylabel(HEIGHT_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    def plot_thrust_ratio(ax, data_ukf, data_no_ukf, label_ukf, label_no_ukf):
        """Helper function to plot estimated thrust ratio (parameter estimation) for both datasets"""
        # Plot UKF thrust ratio
        if data_ukf['parameter_estimation_history'] is not None:
            parameter_estimation = data_ukf['parameter_estimation_history'].flatten()[:MAX_TIMESTEPS]
            ax.plot(parameter_estimation, label=label_ukf, color=THRUST_RATIO_COLOR, 
                   linestyle=THRUST_RATIO_STYLE, linewidth=LINE_WIDTH)
        
        # Plot non-UKF thrust ratio
        if data_no_ukf['parameter_estimation_history'] is not None:
            parameter_estimation_no_ukf = data_no_ukf['parameter_estimation_history'].flatten()[:MAX_TIMESTEPS]
            ax.plot(parameter_estimation_no_ukf, label=label_no_ukf, color=NO_UKF_THRUST_COLOR, 
                   linestyle=NO_UKF_STYLE, linewidth=LINE_WIDTH)
        
        # If no data available for either
        if (data_ukf['parameter_estimation_history'] is None and 
            data_no_ukf['parameter_estimation_history'] is None):
            ax.text(0.5, 0.5, NO_THRUST_DATA_MSG, 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel(THRUST_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    def plot_speed_estimation(ax, data_ukf, data_no_ukf, label_ukf, label_no_ukf):
        """Helper function to plot speed estimation comparison for both datasets"""
        # Calculate speed from velocity components for UKF data
        if data_ukf['UKF_state_estimation_history'] is not None and data_ukf['UKF_state_estimation_history'].shape[1] >= 6:
            # Assuming columns 3,4,5 are vx, vy, vz
            ukf_velocities = data_ukf['UKF_state_estimation_history'][:MAX_TIMESTEPS, 3:6]
            ukf_speed = np.linalg.norm(ukf_velocities, axis=1)
            ax.plot(ukf_speed, label=label_ukf, color=UKF_HEIGHT_COLOR, 
                   linestyle=UKF_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        # Calculate speed from velocity components for non-UKF data
        if data_no_ukf['observed_state_history'] is not None and data_no_ukf['observed_state_history'].shape[1] >= 6:
            # Assuming columns 3,4,5 are vx, vy, vz
            no_ukf_velocities = data_no_ukf['observed_state_history'][:MAX_TIMESTEPS, 3:6]
            no_ukf_speed = np.linalg.norm(no_ukf_velocities, axis=1)
            ax.plot(no_ukf_speed, label=label_no_ukf, color=NO_UKF_HEIGHT_COLOR, 
                   linestyle=NO_UKF_STYLE, linewidth=LINE_WIDTH)
        
        # If no data available for either
        if ((data_ukf['UKF_state_estimation_history'] is None or data_ukf['UKF_state_estimation_history'].shape[1] < 6) and 
            (data_no_ukf['observed_state_history'] is None or data_no_ukf['observed_state_history'].shape[1] < 6)):
            ax.text(0.5, 0.5, 'No velocity data available', 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel('Speed (m/s)', fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    # Plot all comparisons on the same axes for direct comparison
    plot_height_comparison(ax1, data_ukf_enabled, data_ukf_disabled, LABEL_HEIGHT_REFERENCE, LABEL_HEIGHT_ESTIMATED_UKF, LABEL_HEIGHT_ESTIMATED_NO_UKF)
    plot_thrust_ratio(ax2, data_ukf_enabled, data_ukf_disabled, LABEL_THRUST_UKF, LABEL_THRUST_NO_UKF)
    plot_speed_estimation(ax3, data_ukf_enabled, data_ukf_disabled, LABEL_SPEED_UKF, LABEL_SPEED_NO_UKF)
    
    # Adjust layout to prevent overlapping
    plt.tight_layout()
    
    # Save the figure as a PDF with high DPI for journal quality
    plt.savefig('UKF_comparison.pdf', dpi=DPI, bbox_inches='tight', format='pdf')
    plt.show()



if __name__ == "__main__":
    main()
