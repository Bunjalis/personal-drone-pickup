import os
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Path to the planned_trajectories folder
trajectories_folder = 'planned_trajectories'

# Iterate through all CSV files in the folder
for file_name in sorted(os.listdir(trajectories_folder)):
    if file_name.endswith('.csv'):
        file_path = os.path.join(trajectories_folder, file_name)
        data = pd.read_csv(file_path)
        x_positions = data['Planned_Position_X']
        y_positions = data['Planned_Position_Y']
        z_positions = data['Planned_Position_Z']

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_positions, y_positions, z_positions, label=f'Trajectory: {file_name}')
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title(f'3D Plot of {file_name}')
        ax.legend()

        # Set all axes to the same scale
        max_range = max(
            max(x_positions) - min(x_positions),
            max(y_positions) - min(y_positions),
            max(z_positions) - min(z_positions)
        ) / 2.0
        mid_x = (max(x_positions) + min(x_positions)) / 2.0
        mid_y = (max(y_positions) + min(y_positions)) / 2.0
        mid_z = (max(z_positions) + min(z_positions)) / 2.0

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        # Show the plot and wait for the window to close before proceeding
        plt.show()