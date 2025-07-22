
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

# Configuration settings
MAX_TIMESTEPS = 600

# Plot settings
FIGURE_SIZE = (1, 3)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 38
AXIS_LABEL_SIZE = 30
TICK_LABEL_SIZE = 26
LEGEND_SIZE = 26
LINE_WIDTH = 5
GRID_ALPHA = 0.3

# Colors - Custom palette (converted from D3.js)
# Palette: ['#00429d', '#415395', '#59668a', '#657b7d', '#68926c', '#5dab55', '#31c52f']
DESIRED_HEIGHT_COLOR = '#657b7d'        # Gray-green - reference line
UKF_HEIGHT_COLOR = '#00429d'            # Dark blue - with UKF
THRUST_RATIO_COLOR = '#59668a'          # Blue-gray - thrust ratio
NO_LATENCY_UKF_COLOR = '#31c52f'        # Bright green - without UKF
NO_LATENCY_THRUST_COLOR = '#5dab55'     # Medium green - thrust ratio comparison
NO_DATA_TEXT_SIZE = 48

# Line styles - Different patterns for black/white distinction
DESIRED_HEIGHT_STYLE = ':'              # Dotted for reference
UKF_HEIGHT_STYLE = '-'                  # Solid for primary data
THRUST_RATIO_STYLE = '-'                # Solid for primary data
NO_LATENCY_STYLE = '-'                 # Solid for comparison data

# Labels - All text labels used in the plots
DESIRED_HEIGHT_LABEL = 'Desired Height'
UKF_HEIGHT_LABEL = 'UKF State Height'
THRUST_RATIO_LABEL = 'Estimated Thrust Ratio'

# Plot-specific labels
UKF_HEIGHT_ENABLED_LABEL = 'Big Initial Conditions'
UKF_HEIGHT_DISABLED_LABEL = 'Small Initial Conditions'
THRUST_RATIO_ENABLED_LABEL = 'Big Initial Conditions'
THRUST_RATIO_DISABLED_LABEL = 'Small Initial Conditions'

# Axis labels
HEIGHT_YLABEL = 'Height (m)'
THRUST_YLABEL = 'Est. Thrust Ratio'
XLABEL = 'Time Steps'

# Error messages
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
    no_latency_estimation_dir = 'BIGQUAD_XYZ_SINE_1_BIG_INIT'
    latency_estimation_dir = 'BIGQUAD_XYZ_SINE_2_SMALL_INIT'
    
    # Load data for both trials
    print("Loading latency compensation enabled data...")
    data_latency_enabled = load_trial_data(latency_estimation_dir)
    
    print("Loading no latency compensation data...")
    data_no_latency = load_trial_data(no_latency_estimation_dir)
    
    # Create a figure with 2x1 grid of subplots (2 rows, 1 column)
    figure, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
    
    # Add subplot labels (a), (b) for academic papers
    ax1.text(-0.1, 1.02, '(a)', transform=ax1.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
    ax2.text(-0.1, 1.02, '(b)', transform=ax2.transAxes, fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')

    def plot_height_comparison(ax, data_enabled, data_disabled):
        """Helper function to plot height comparison data for both conditions"""
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
            ax.plot(trajectory_height_enabled, label=DESIRED_HEIGHT_LABEL, color=DESIRED_HEIGHT_COLOR, 
                   linestyle=DESIRED_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        if data_enabled['UKF_state_estimation_history'] is not None and data_enabled['UKF_state_estimation_history'].shape[1] >= 3:
            ukf_height_enabled = data_enabled['UKF_state_estimation_history'][:MAX_TIMESTEPS, 2]
            ax.plot(ukf_height_enabled, label=UKF_HEIGHT_ENABLED_LABEL, color=UKF_HEIGHT_COLOR, 
                   linestyle=UKF_HEIGHT_STYLE, linewidth=LINE_WIDTH)
        
        # Plot data for no latency compensation
        if data_disabled['UKF_state_estimation_history'] is not None and data_disabled['UKF_state_estimation_history'].shape[1] >= 3:
            ukf_height_disabled = data_disabled['UKF_state_estimation_history'][:MAX_TIMESTEPS, 2]
            ax.plot(ukf_height_disabled, label=UKF_HEIGHT_DISABLED_LABEL, color=NO_LATENCY_UKF_COLOR, 
                   linestyle=NO_LATENCY_STYLE, linewidth=LINE_WIDTH)
        
        ax.set_ylabel(HEIGHT_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    def plot_thrust_ratio(ax, data_enabled, data_disabled):
        """Helper function to plot estimated thrust ratio for both conditions"""
        if data_enabled['parameter_estimation_history'] is not None:
            parameter_estimation_enabled = data_enabled['parameter_estimation_history'].flatten()[:MAX_TIMESTEPS]
            ax.plot(parameter_estimation_enabled, label=THRUST_RATIO_ENABLED_LABEL, color=THRUST_RATIO_COLOR, 
                   linestyle=THRUST_RATIO_STYLE, linewidth=LINE_WIDTH)
        
        if data_disabled['parameter_estimation_history'] is not None:
            parameter_estimation_disabled = data_disabled['parameter_estimation_history'].flatten()[:MAX_TIMESTEPS]
            ax.plot(parameter_estimation_disabled, label=THRUST_RATIO_DISABLED_LABEL, color=NO_LATENCY_THRUST_COLOR, 
                   linestyle=NO_LATENCY_STYLE, linewidth=LINE_WIDTH)
        
        if data_enabled['parameter_estimation_history'] is None and data_disabled['parameter_estimation_history'] is None:
            ax.text(0.5, 0.5, NO_THRUST_DATA_MSG, 
                   horizontalalignment='center', verticalalignment='center', 
                   transform=ax.transAxes, fontsize=NO_DATA_TEXT_SIZE)
        
        ax.set_ylabel(THRUST_YLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.set_xlabel(XLABEL, fontsize=AXIS_LABEL_SIZE)
        ax.tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)
        ax.legend(fontsize=LEGEND_SIZE)

    # Plot all comparisons
    plot_height_comparison(ax1, data_latency_enabled, data_no_latency)
    plot_thrust_ratio(ax2, data_latency_enabled, data_no_latency)

    # Adjust layout to prevent overlapping
    plt.tight_layout()
    
    # Save the plot as a PDF
    plt.savefig('plot_init_condition_comparison.pdf', format='pdf', dpi=400, bbox_inches='tight')
    print("Plot saved as 'latency_compensation_comparison.pdf'")
    
    # Show the plot
    plt.show()



if __name__ == "__main__":
    main()
