import matplotlib.pyplot as plt
import pandas as pd
import numpy as np



# 30Hz_test_6 - with new first order system for motors
# 50Hz_test_7 - tried running controller at 50Hz
# 50Hz_test_8 - started at zero and decreased mass and inertia
# 50Hz_test_9 - incrased mass
# 50Hz_test_10 - saved each nlmpc solved/planned trajectory at each step
# 50Hz_test_11 - hoizon 1.0s
# 50Hz_test_12 - horizon 0.5s
# 50Hz_test_13 - horizon 2.0s
# 15Hz_test_14 - horizon 2.0s 
# 50Hz_test_15 - horizon 1.0 20 samples/nodes
# 50Hz_test_16 - horizon 1.0 20 samples/nodes but in simulation 
# 50Hz_test_17 - horizon 1.0 20 samples/nodes
# 50Hz_test_18 - horizon 1.0 20 samples/nodes with inverted yaw
# 50Hz_test_19 - horizon 1.0 20 samples/nodes full data collection incase simulation presents data differently
# 50Hz_test_20 - horizon 1.0 20 samples/nodes full data collection incase simulation presents data differently but in simulation
# 50Hz_test_21 - horizon 1.0 20 samples/nodes full data collection incase simulation presents data differently motion capture at 60Hz
# 50Hz_test_22 - first simulation test with a 0.01 g point mass offset from the drones body -0.03, -0.03
# 50Hz_test_23 - first simulation test with a 0.02 g point mass offset from the drones body -0.03, -0.03
# 50Hz_test_24 - first simulation test with a 0.05 g point mass offset from the drones body -0.03, -0.03

# Load the CSV file
data = pd.read_csv('50Hz_test_21.csv')

# Drop the last 3 data points from the dataset
#data = data[:-50]

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
fig, axs = plt.subplots(3, 1, figsize=(12, 6))

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

'''
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
'''

# Iterate through the data and print the required information
for index, row in data.iterrows():
    print("----------------------------------------------------")
    print(f"Step: {row['Step']}")

    print("Position")
    print(f" - CTRL Act: {row['Last_Control_0']}, {row['Last_Control_1']}, {row['Last_Control_2']}, {row['Last_Control_3']}")
    print(f" - Prev Pos: {row['Last_Position_X']}, {row['Last_Position_Y']}, {row['Last_Position_Z']}")
    print(f" - Pred Pos: {row['Predicted_Position_X']}, {row['Predicted_Position_Y']}, {row['Predicted_Position_Z']}")
    print(f" - Actu Pos: {row['Actual_Position_X']}, {row['Actual_Position_Y']}, {row['Actual_Position_Z']}")
    position_error = [
        row['Predicted_Position_X'] - row['Actual_Position_X'],
        row['Predicted_Position_Y'] - row['Actual_Position_Y'],
        row['Predicted_Position_Z'] - row['Actual_Position_Z']
    ]
    total_position_error = sum(abs(e) for e in position_error)
    print(f" - Pos Error: {position_error[0]}, {position_error[1]}, {position_error[2]}")
    print(f" - Total Pos Error: {total_position_error}")

    print("Orientation")
    print(f" - CTRL Act: {row['Last_Control_0']}, {row['Last_Control_1']}, {row['Last_Control_2']}, {row['Last_Control_3']}")
    print(f" - Prev Ori: {row['Last_Orientation_W']}, {row['Last_Orientation_X']}, {row['Last_Orientation_Y']}, {row['Last_Orientation_Z']}")
    print(f" - Pred Ori: {row['Predicted_Orientation_W']}, {row['Predicted_Orientation_X']}, {row['Predicted_Orientation_Y']}, {row['Predicted_Orientation_Z']}")
    print(f" - Actu Ori: {row['Actual_Orientation_W']}, {row['Actual_Orientation_X']}, {row['Actual_Orientation_Y']}, {row['Actual_Orientation_Z']}")
    orientation_error = [
        row['Predicted_Orientation_W'] - row['Actual_Orientation_W'],
        row['Predicted_Orientation_X'] - row['Actual_Orientation_X'],
        row['Predicted_Orientation_Y'] - row['Actual_Orientation_Y'],
        row['Predicted_Orientation_Z'] - row['Actual_Orientation_Z']
    ]
    total_orientation_error = sum(abs(e) for e in orientation_error)
    print(f" - Ori Error: {orientation_error[0]}, {orientation_error[1]}, {orientation_error[2]}, {orientation_error[3]}")
    print(f" - Total Ori Error: {total_orientation_error}")

    print("Linear Velocity")
    print(f" - CTRL Act: {row['Last_Control_0']}, {row['Last_Control_1']}, {row['Last_Control_2']}, {row['Last_Control_3']}")
    print(f" - Prev L_V: {row['Last_Linear_Velocity_X']}, {row['Last_Linear_Velocity_Y']}, {row['Last_Linear_Velocity_Z']}")
    print(f" - Pred L_V: {row['Predicted_Linear_Velocity_X']}, {row['Predicted_Linear_Velocity_Y']}, {row['Predicted_Linear_Velocity_Z']}")
    print(f" - Actu L_V: {row['Actual_Linear_Velocity_X']}, {row['Actual_Linear_Velocity_Y']}, {row['Actual_Linear_Velocity_Z']}")
    linear_velocity_error = [
        row['Predicted_Linear_Velocity_X'] - row['Actual_Linear_Velocity_X'],
        row['Predicted_Linear_Velocity_Y'] - row['Actual_Linear_Velocity_Y'],
        row['Predicted_Linear_Velocity_Z'] - row['Actual_Linear_Velocity_Z']
    ]
    total_linear_velocity_error = sum(abs(e) for e in linear_velocity_error)
    print(f" - L_V Error: {linear_velocity_error[0]}, {linear_velocity_error[1]}, {linear_velocity_error[2]}")
    print(f" - Total L_V Error: {total_linear_velocity_error}")

    print("Angular Velocity")
    print(f" - CTRL Act: {row['Last_Control_0']}, {row['Last_Control_1']}, {row['Last_Control_2']}, {row['Last_Control_3']}")
    print(f" - Prev w_V: {row['Last_Angular_Velocity_X']}, {row['Last_Angular_Velocity_Y']}, {row['Last_Angular_Velocity_Z']}")
    print(f" - Pred w_V: {row['Predicted_Angular_Velocity_X']}, {row['Predicted_Angular_Velocity_Y']}, {row['Predicted_Angular_Velocity_Z']}")
    print(f" - Actu w_V: {row['Actual_Angular_Velocity_X']}, {row['Actual_Angular_Velocity_Y']}, {row['Actual_Angular_Velocity_Z']}")
    angular_velocity_error = [
        row['Predicted_Angular_Velocity_X'] - row['Actual_Angular_Velocity_X'],
        row['Predicted_Angular_Velocity_Y'] - row['Actual_Angular_Velocity_Y'],
        row['Predicted_Angular_Velocity_Z'] - row['Actual_Angular_Velocity_Z']
    ]
    total_angular_velocity_error = sum(abs(e) for e in angular_velocity_error)
    print(f" - w_V Error: {angular_velocity_error[0]}, {angular_velocity_error[1]}, {angular_velocity_error[2]}")
    print(f" - Total w_V Error: {total_angular_velocity_error}")
    


# Adjust layout and show the plot
plt.tight_layout()
plt.show()