import csv
import os
import shutil
from datetime import datetime


class DataLogger:
    """Handles all CSV logging functionality for the drone controller experiments."""
    
    def __init__(self, trial_name, base_experiment_folder="/home/mitchell/Documents/PhD/drone_cage_control/NEW_RATE_EXPERIMENTS"):
        """
        Initialize the data logger with experiment folder and CSV writers.
        
        Args:
            trial_name (str): Name of the current trial/experiment
            base_experiment_folder (str): Base folder for all experiments
        """
        self.trial_name = trial_name
        self.base_experiment_folder = base_experiment_folder
        
        # Create experiment folder
        self.output_folder = os.path.join(base_experiment_folder, trial_name)
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Initialize data storage lists
        self.control_history = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]] * 10 
        self.observed_state_history = []
        self.motion_capture_history = []
        self.parameter_estimation_history = []
        self.estimated_state_history = []
        self.delay_state_estimation_history = []
        self.UKF_state_estimation_history = []
        self.control_loop_timing_history = []
        self.battery_voltage_history = []
        
        # Initialize CSV files and writers
        self._initialize_csv_files()
        
        # Flag to prevent multiple closes
        self.closed = False
    
    def _initialize_csv_files(self):
        """Initialize all CSV files and writers."""
        self.control_history_file = open(os.path.join(self.output_folder, 'control_history.csv'), mode='w', newline='')
        self.control_history_writer = csv.writer(self.control_history_file)

        self.observed_state_history_file = open(os.path.join(self.output_folder, 'observed_state_history.csv'), mode='w', newline='')
        self.observed_state_history_writer = csv.writer(self.observed_state_history_file)

        self.motion_capture_history_file = open(os.path.join(self.output_folder, 'motion_capture_history.csv'), mode='w', newline='')
        self.motion_capture_history_writer = csv.writer(self.motion_capture_history_file)

        self.parameter_estimation_history_file = open(os.path.join(self.output_folder, 'parameter_estimation_history.csv'), mode='w', newline='')
        self.parameter_estimation_history_writer = csv.writer(self.parameter_estimation_history_file)

        self.estimated_state_history_file = open(os.path.join(self.output_folder, 'estimated_state_history.csv'), mode='w', newline='')
        self.estimated_state_history_writer = csv.writer(self.estimated_state_history_file)

        self.delay_state_estimation_history_file = open(os.path.join(self.output_folder, 'delay_state_estimation_history.csv'), mode='w', newline='')
        self.delay_state_estimation_history_writer = csv.writer(self.delay_state_estimation_history_file)

        self.UKF_state_estimation_history_file = open(os.path.join(self.output_folder, 'UKF_state_estimation_history.csv'), mode='w', newline='')
        self.UKF_state_estimation_history_writer = csv.writer(self.UKF_state_estimation_history_file)

        self.control_loop_timing_file = open(os.path.join(self.output_folder, 'control_loop_timing.csv'), mode='w', newline='')
        self.control_loop_timing_writer = csv.writer(self.control_loop_timing_file)

        self.battery_voltage_file = open(os.path.join(self.output_folder, 'battery_voltage_history.csv'), mode='w', newline='')
        self.battery_voltage_writer = csv.writer(self.battery_voltage_file)

        self.trajectory_file = open(os.path.join(self.output_folder, 'trajectory.csv'), mode='w', newline='')
        self.trajectory_writer = csv.writer(self.trajectory_file)
    
    def save_trajectory(self, trajectory):
        """Save the trajectory data to CSV file."""
        self.trajectory_writer.writerows(trajectory.T)  # Save trajectory as rows
    
    def log_control_data(self, u, u_rate):
        """Log control inputs and rates."""
        self.control_history.append(u.tolist() + u_rate.tolist())
    
    def log_observed_state(self, state):
        """Log observed state data."""
        self.observed_state_history.append(state.tolist())
    
    def log_motion_capture_state(self, state):
        """Log motion capture state data."""
        if state is not None:
            self.motion_capture_history.append(state.tolist())
    
    def log_parameter_estimation(self, params):
        """Log parameter estimation data."""
        self.parameter_estimation_history.append(params.tolist())
    
    def log_estimated_state(self, state):
        """Log estimated state data."""
        self.estimated_state_history.append(state.tolist())
    
    def log_delay_estimation(self, delay):
        """Log delay estimation data."""
        self.delay_state_estimation_history.append(delay)
    
    def log_ukf_state(self, state):
        """Log UKF state estimation data."""
        self.UKF_state_estimation_history.append(state[:19].tolist())
    
    def log_control_timing(self, timing):
        """Log control loop timing data."""
        self.control_loop_timing_history.append(timing)
    
    def log_battery_voltage(self, voltage):
        """Log battery voltage data."""
        self.battery_voltage_history.append(voltage)
    
    def copy_source_files_to_output(self, source_files_info):
        """
        Copy source code files to the experiment folder for reproducibility.
        
        Args:
            source_files_info (dict): Dictionary with file names as keys and file paths as values
        """
        try:
            source_code_dir = os.path.join(self.output_folder, 'source_code')
            os.makedirs(source_code_dir, exist_ok=True)
            
            for filename, filepath in source_files_info.items():
                if os.path.exists(filepath):
                    shutil.copy2(filepath, os.path.join(source_code_dir, filename))
                    print(f"Copied {filename} to {source_code_dir}")
            
            print(f"Source code files copied to: {source_code_dir}")
            
        except Exception as e:
            print(f"Warning: Could not copy source files: {e}")
    
    def close_and_save(self):
        """Save all data to CSV files and close file handles."""
        if self.closed:
            return
        
        print("Saving data to CSV files...")
        
        # Write all accumulated data to CSV files
        self.control_history_writer.writerows(self.control_history)
        self.observed_state_history_writer.writerows(self.observed_state_history)
        self.motion_capture_history_writer.writerows(self.motion_capture_history)
        self.parameter_estimation_history_writer.writerows(self.parameter_estimation_history)
        self.estimated_state_history_writer.writerows(self.estimated_state_history)
        self.delay_state_estimation_history_writer.writerows([[d] for d in self.delay_state_estimation_history])
        self.UKF_state_estimation_history_writer.writerows(self.UKF_state_estimation_history)
        self.control_loop_timing_writer.writerows([[t] for t in self.control_loop_timing_history])
        self.battery_voltage_writer.writerows([[v] for v in self.battery_voltage_history])

        # Close all files
        self.control_history_file.close()
        self.observed_state_history_file.close()
        self.motion_capture_history_file.close()
        self.parameter_estimation_history_file.close()
        self.estimated_state_history_file.close()
        self.delay_state_estimation_history_file.close()
        self.UKF_state_estimation_history_file.close()
        self.trajectory_file.close()
        self.control_loop_timing_file.close()
        self.battery_voltage_file.close()
        
        self.closed = True
        print(f"Data saved to: {self.output_folder}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures files are closed."""
        self.close_and_save()