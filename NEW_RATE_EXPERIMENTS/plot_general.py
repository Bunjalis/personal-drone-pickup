import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import argparse

def load_trial_data(trial_directory):
    """Load all CSV files from a trial directory"""
    data = {}
    
    # Define the expected CSV files
    csv_files = {
        'control_history': 'control_history.csv',
        'observed_state_history': 'observed_state_history.csv',
        'motion_capture_history': 'observed_state_history.csv',
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

def plot_system_response(data, trial_name):
    """Plot system response similar to the original plotSystemResponse method"""
    
    # Convert control history to a numpy array and remove the first 10 commands
    if data['control_history'] is not None and len(data['control_history']) > 10:
        control_history = data['control_history'][10:]
    else:
        print("Warning: Insufficient control history data")
        return
    
    # Create a figure with six subplots
    figure, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(12, 24))
    figure.suptitle(f'System Response Analysis - {trial_name}', fontsize=16)

    # Plot all control actions on the first subplot
    if control_history.shape[1] >= 4:
        ax1.plot(control_history[:, 0], label='Roll', color='blue')
        ax1.plot(control_history[:, 1], label='Pitch', color='orange')
        ax1.plot(control_history[:, 2], label='Throttle', color='green')
        ax1.plot(control_history[:, 3], label='Yaw', color='red')
    ax1.set_title('Control Actions over Time')
    ax1.set_ylabel('Control Values')
    ax1.set_xlabel('Time Steps')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Extract state histories
    if data['observed_state_history'] is not None:
        state_history = data['observed_state_history']
        
        # Get trajectory data if available
        trajectory_height = None
        trajectory_y = None
        trajectory_x = None
        if data['trajectory'] is not None:
            traj = data['trajectory'].T  # Transpose to match expected format
            if len(traj) >= 3:
                trajectory_height = traj[2, :len(state_history)] if len(traj[2]) >= len(state_history) else traj[2]
                trajectory_y = traj[1, :len(state_history)] if len(traj[1]) >= len(state_history) else traj[1]
                trajectory_x = traj[0, :len(state_history)] if len(traj[0]) >= len(state_history) else traj[0]

        # Plot height comparison on the second subplot
        if state_history.shape[1] >= 3:
            ax2.plot(state_history[:, 2], label='Observed State Height', color='purple')
        if trajectory_height is not None:
            ax2.plot(trajectory_height, label='Desired Trajectory Height', color='cyan', linestyle='dashed')
        if data['motion_capture_history'] is not None and data['motion_capture_history'].shape[1] >= 3:
            motion_capture_height = data['motion_capture_history'][:, 2]
            ax2.plot(motion_capture_height, label='Motion Capture Height', color='magenta', linestyle='dotted')
        if data['estimated_state_history'] is not None and data['estimated_state_history'].shape[1] >= 3:
            estimated_state_height = data['estimated_state_history'][:, 2]
            ax2.plot(estimated_state_height, label='Estimated State Height', color='green', linestyle='dashdot')
        if data['UKF_state_estimation_history'] is not None and data['UKF_state_estimation_history'].shape[1] >= 3:
            UKF_state_height = data['UKF_state_estimation_history'][:, 2]
            ax2.plot(UKF_state_height, label='UKF State Height', color='orange', linestyle='solid')
        ax2.set_title('Height Comparison over Time')
        ax2.set_ylabel('Height (m)')
        ax2.set_xlabel('Time Steps')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot the estimated parameter over time on the third subplot
        if data['parameter_estimation_history'] is not None:
            parameter_estimation = data['parameter_estimation_history'].flatten()
            ax3.plot(parameter_estimation, label='Estimated Parameter', color='brown')
        ax3.set_title('Estimated Parameter over Time')
        ax3.set_ylabel('Parameter Value')
        ax3.set_xlabel('Time Steps')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot the delay state estimation history on the fourth subplot
        if data['delay_state_estimation_history'] is not None:
            delay_state_estimation = data['delay_state_estimation_history'].flatten()
            ax4.plot(delay_state_estimation, label='Delay State Estimation', color='blue')
        ax4.set_title('Delay State Estimation over Time')
        ax4.set_ylabel('Delay States')
        ax4.set_xlabel('Time Steps')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        # Plot the y-axis values on the fifth subplot
        if state_history.shape[1] >= 2:
            ax5.plot(state_history[:, 1], label='Observed State Y', color='purple')
        if trajectory_y is not None:
            ax5.plot(trajectory_y, label='Desired Trajectory Y', color='cyan', linestyle='dashed')
        if data['motion_capture_history'] is not None and data['motion_capture_history'].shape[1] >= 2:
            motion_capture_y = data['motion_capture_history'][:, 1]
            ax5.plot(motion_capture_y, label='Motion Capture Y', color='magenta', linestyle='dotted')
        if data['estimated_state_history'] is not None and data['estimated_state_history'].shape[1] >= 2:
            estimated_state_y = data['estimated_state_history'][:, 1]
            ax5.plot(estimated_state_y, label='Estimated State Y', color='green', linestyle='dashdot')
        if data['UKF_state_estimation_history'] is not None and data['UKF_state_estimation_history'].shape[1] >= 2:
            UKF_state_y = data['UKF_state_estimation_history'][:, 1]
            ax5.plot(UKF_state_y, label='UKF State Y', color='orange', linestyle='solid')
        ax5.set_title('Y-Axis Comparison over Time')
        ax5.set_ylabel('Y-Axis (m)')
        ax5.set_xlabel('Time Steps')
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        # Plot the x-axis values on the sixth subplot
        if state_history.shape[1] >= 1:
            ax6.plot(state_history[:, 0], label='Observed State X', color='purple')
        if trajectory_x is not None:
            ax6.plot(trajectory_x, label='Desired Trajectory X', color='cyan', linestyle='dashed')
        if data['motion_capture_history'] is not None and data['motion_capture_history'].shape[1] >= 1:
            motion_capture_x = data['motion_capture_history'][:, 0]
            ax6.plot(motion_capture_x, label='Motion Capture X', color='magenta', linestyle='dotted')
        if data['estimated_state_history'] is not None and data['estimated_state_history'].shape[1] >= 1:
            estimated_state_x = data['estimated_state_history'][:, 0]
            ax6.plot(estimated_state_x, label='Estimated State X', color='green', linestyle='dashdot')
        if data['UKF_state_estimation_history'] is not None and data['UKF_state_estimation_history'].shape[1] >= 1:
            UKF_state_x = data['UKF_state_estimation_history'][:, 0]
            ax6.plot(UKF_state_x, label='UKF State X', color='orange', linestyle='solid')
        ax6.set_title('X-Axis Comparison over Time')
        ax6.set_ylabel('X-Axis (m)')
        ax6.set_xlabel('Time Steps')
        ax6.legend()
        ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    return figure

def main():
    parser = argparse.ArgumentParser(description='Plot system response from trial data')
    parser.add_argument('trial_directory', help='Path to the trial directory containing CSV files')
    parser.add_argument('--save', action='store_true', help='Save plots instead of showing them')
    parser.add_argument('--output', default='system_response.png', help='Output filename for saved plot')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.trial_directory):
        print(f"Error: Trial directory '{args.trial_directory}' does not exist")
        sys.exit(1)
    
    # Load trial data
    print(f"Loading trial data from: {args.trial_directory}")
    data = load_trial_data(args.trial_directory)
    
    # Get trial name from directory
    trial_name = os.path.basename(args.trial_directory.rstrip('/'))
    
    # Plot system response
    print("Generating plots...")
    figure = plot_system_response(data, trial_name)
    
    if args.save:
        output_path = os.path.join(args.trial_directory, args.output)
        figure.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
