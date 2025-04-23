import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_trajectory_data(file_path):
    simulated_states = []
    recorded_states = []

    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            # Skip empty rows and header rows
            if len(row) == 0 or row[0].startswith("//") or row[0] in ["Initial Solve State", "Step"]:
                continue

            # Extract the state (x, y, z) and recorded state (x, y, z) from the CSV file
            try:
                state = [float(row[1]), float(row[2]), float(row[3])]  # Simulated state (x, y, z)
                recorded_state = [float(row[14]), float(row[15]), float(row[16])]  # Recorded state (x, y, z)
                simulated_states.append(state)
                recorded_states.append(recorded_state)
            except ValueError:
                # Skip rows that cannot be parsed
                continue

    return np.array(simulated_states), np.array(recorded_states)

def plot_trajectories(simulated_states, recorded_states):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the simulated trajectory
    ax.plot(simulated_states[:, 0], simulated_states[:, 1], simulated_states[:, 2], label='Simulated Trajectory', color='blue')

    # Plot the recorded trajectory
    ax.plot(recorded_states[:, 0], recorded_states[:, 1], recorded_states[:, 2], label='Recorded Trajectory', color='red')

    # Labels and legend
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Simulated vs Recorded Trajectory in 3D Space')
    ax.legend()

    plt.show()

if __name__ == "__main__":
    file_path = "/home/mitchell/Documents/PhD/drone_cage_control/trajectory_data.csv"
    simulated_states, recorded_states = load_trajectory_data(file_path)
    plot_trajectories(simulated_states, recorded_states)