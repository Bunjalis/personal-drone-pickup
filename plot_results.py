import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Load the CSV file
data = pd.read_csv('30Hz_test_5.csv')

# Extract the measured angular velocity columns
actual_angular_velocity_x = data['Actual_Angular_Velocity_X']
actual_angular_velocity_y = data['Actual_Angular_Velocity_Y']
actual_angular_velocity_z = data['Actual_Angular_Velocity_Z']
steps = data['Step']

# Extract the pose columns
actual_position_x = data['Actual_Position_X']
actual_position_y = data['Actual_Position_Y']
actual_position_z = data['Actual_Position_Z']

# Extract the control variables
control_0 = data['Last_Control_0']
control_1 = data['Last_Control_1']
control_2 = data['Last_Control_2']
control_3 = data['Last_Control_3']

def quaternion_to_euler(w, x, y, z):
    """Convert a quaternion into euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

# Convert quaternion to roll, pitch, yaw
roll, pitch, yaw = zip(*[quaternion_to_euler(w, x, y, z) for w, x, y, z in zip(data['Actual_Orientation_W'], data['Actual_Orientation_X'], data['Actual_Orientation_Y'], data['Actual_Orientation_Z'])])

# Calculate the error between predicted and actual angular velocity
delta_predicted_angular_velocity_x = data['Predicted_Angular_Velocity_X'] - data['Last_Angular_Velocity_X']
delta_predicted_angular_velocity_y = data['Predicted_Angular_Velocity_Y'] - data['Last_Angular_Velocity_Y']
delta_predicted_angular_velocity_z = data['Predicted_Angular_Velocity_Z'] - data['Last_Angular_Velocity_Z']

# Calculate the difference between the change in last angular velocity and actual angular velocity
delta_actual_angular_velocity_x = data['Actual_Angular_Velocity_X'] - data['Last_Angular_Velocity_X']
delta_actual_angular_velocity_y = data['Actual_Angular_Velocity_Y'] - data['Last_Angular_Velocity_Y']
delta_actual_angular_velocity_z = data['Actual_Angular_Velocity_Z'] - data['Last_Angular_Velocity_Z']


# Create a figure with four subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 24))

# Plot the control variables on the first subplot
axs[0].plot(steps, control_0, label='Control 0', color='r')
axs[0].plot(steps, control_1, label='Control 1', color='g')
axs[0].plot(steps, control_2, label='Control 2', color='b')
axs[0].plot(steps, control_3, label='Control 3', color='m')
axs[0].set_title('Control Variables (u)')
axs[0].set_xlabel('Step')
axs[0].set_ylabel('Control Value')
axs[0].legend()
axs[0].grid(True)

# Plot the XYZ pose of the drone and RPY on the second subplot
axs[1].plot(steps, actual_position_x, label='Position X', color='r')
axs[1].plot(steps, actual_position_y, label='Position Y', color='g')
axs[1].plot(steps, actual_position_z, label='Position Z', color='b')
axs[1].plot(steps, roll, label='Roll', linestyle='--', color='c')
axs[1].plot(steps, pitch, label='Pitch', linestyle='--', color='m')
axs[1].plot(steps, yaw, label='Yaw', linestyle='--', color='y')
axs[1].set_title('Drone XYZ Pose and Orientation (RPY)')
axs[1].set_xlabel('Step')
axs[1].set_ylabel('Position (m) / Orientation (rad)')
axs[1].legend()
axs[1].grid(True)

# Plot the measured angular velocity on the third subplot
axs[2].plot(steps, actual_angular_velocity_x, label='Angular Velocity X', color='r')
axs[2].plot(steps, actual_angular_velocity_y, label='Angular Velocity Y', color='g')
axs[2].plot(steps, actual_angular_velocity_z, label='Angular Velocity Z', color='b')
axs[2].set_title('Measured Angular Velocity')
axs[2].set_xlabel('Step')
axs[2].set_ylabel('Angular Velocity (rad/s)')
axs[2].legend()
axs[2].grid(True)

# Plot the delta predicted and actual angular velocity on the fourth subplot
axs[3].plot(steps, delta_predicted_angular_velocity_x, label='Delta Predicted Angular Velocity X', linestyle='--', color='r')
axs[3].plot(steps, delta_predicted_angular_velocity_y, label='Delta Predicted Angular Velocity Y', linestyle='--', color='g')
axs[3].plot(steps, delta_predicted_angular_velocity_z, label='Delta Predicted Angular Velocity Z', linestyle='--', color='b')
axs[3].plot(steps, delta_actual_angular_velocity_x, label='Delta Actual Angular Velocity X', color='r')
axs[3].plot(steps, delta_actual_angular_velocity_y, label='Delta Actual Angular Velocity Y', color='g')
axs[3].plot(steps, delta_actual_angular_velocity_z, label='Delta Actual Angular Velocity Z', color='b')
axs[3].set_title('Delta Angular Velocity (Predicted vs Actual)')
axs[3].set_xlabel('Step')
axs[3].set_ylabel('Delta Angular Velocity (rad/s)')
axs[3].legend()
axs[3].grid(True)

# Iterate through the data and print the required information
for index, row in data.iterrows():
    print(f"Step: {row['Step']}")
    print(f"Control Actions: {row['Last_Control_0']}, {row['Last_Control_1']}, {row['Last_Control_2']}, {row['Last_Control_3']}")
    print(f"Previous Angular Velocity: {row['Last_Angular_Velocity_X']}, {row['Last_Angular_Velocity_Y']}, {row['Last_Angular_Velocity_Z']}")
    print(f"Predicted Angular Velocity: {row['Predicted_Angular_Velocity_X']}, {row['Predicted_Angular_Velocity_Y']}, {row['Predicted_Angular_Velocity_Z']}")
    print(f"Actual Angular Velocity: {row['Actual_Angular_Velocity_X']}, {row['Actual_Angular_Velocity_Y']}, {row['Actual_Angular_Velocity_Z']}")
    print("-")

# Adjust layout and show the plot
plt.tight_layout()
plt.show()