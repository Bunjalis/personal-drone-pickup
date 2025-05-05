import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import matplotlib.ticker as ticker





# thrust_data_step_1 first test with previous FC and controller
# thrust_data_step_2 second test with updated ardupilot parameters 
# thrust_data_step_3 updated ELRS parameters
# thrust_data_step_4 reduced elrs command buffer size
# thrust_data_step_5 turned off ardupilot EKF
# thrust_data_step_6 Tried to reduce arduino polling time
# thrust_data_step_7 moved throttle command message before thrust recieve message
# thrust_data_step_8 increased arduino resolution to 5 digits
# thrust_data_step_9 changed ardupilot output from 50Hz to 400Hz - no prop
# thrust_data_step_10 changed ardupilot output from 50Hz to 400Hz - prop
# thrust_data_step_11 switched to betaflight - prop
# thrust_data_step_12 Back to ardupilot but with 100Hz full instead of 333Hz full - prop


filename = 'thrust_data_step_12.csv'
# Load the thrust data CSV file
data_thrust = pd.read_csv(filename, names=['Throttle', 'Thrust'])

# Generate a time array assuming uniform time steps
time = np.arange(len(data_thrust))

# Add a condition to apply the scaled throttle mapping only for thrust_data_step_11
if filename == 'thrust_data_step_11.csv':
    print("Data for thrust_data_step_11.csv loaded.")
    scaled_throttle = np.zeros_like(data_thrust['Throttle'])
    scaled_throttle = np.where(data_thrust['Throttle'] == -1.0, 0.007, np.where(data_thrust['Throttle'] == -0.8, 0.007, np.where(data_thrust['Throttle'] == -0.4, 0.09, scaled_throttle)))

else:
    scaled_throttle = np.where(data_thrust['Throttle'] == 0, 0, (data_thrust['Throttle'] - 0.1) / (0.5 - 0.1) * (0.12415 - 0.0075) + 0.0075)

# Scale time to seconds
time_seconds = time / 80

# Create a new figure for thrust plot
plt.figure(figsize=(10, 6))

# Plot thrust against time in seconds
plt.plot(time_seconds, data_thrust['Thrust'], label='Thrust', color='b')

# Plot scaled throttle against time in seconds
plt.plot(time_seconds, scaled_throttle, label='Scaled Throttle', color='r', linestyle='--')

# Fix the range step for 30Hz intervals
for t in np.arange(0, max(time_seconds), 1/30):
    plt.axvline(x=t, color='g', linestyle=':', linewidth=0.5, label='30Hz Interval' if t == 0 else "")

# Add labels, title, and legend
plt.title('Thrust vs Time (Seconds)')
plt.xlabel('Time (seconds)')
plt.ylabel('Thrust / Scaled Throttle')
plt.legend()
plt.grid(True)

# Adjust y-axis ticks for better readability
ax = plt.gca()
ax.yaxis.set_major_locator(ticker.MaxNLocator(nbins=10))

# Show the plot
plt.tight_layout()
plt.show()

# Load all thrust data CSV files
data_files = [
    'thrust_data_step_1.csv',
    'thrust_data_step_2.csv',
    'thrust_data_step_3.csv',
    'thrust_data_step_4.csv',
    'thrust_data_step_5.csv',
    'thrust_data_step_6.csv',
    'thrust_data_step_7.csv',
    'thrust_data_step_8.csv'
]

# Create a new figure for all thrust data
plt.figure(figsize=(12, 8))

# Iterate through each file and plot the data
for i, file in enumerate(data_files, start=1):
    data = pd.read_csv(file, names=['Throttle', 'Thrust'])
    time = np.arange(len(data)) / 80  # Scale time to seconds
    plt.plot(time, data['Thrust'], label=f'Thrust Data Step {i}')

    # Plot scaled throttle for each data set
    scaled_throttle = np.where(data['Throttle'] == 0, 0, (data['Throttle'] - 0.1) / (0.5 - 0.1) * (0.12415 - 0.0075) + 0.0075)
    plt.plot(time, scaled_throttle, linestyle='--', label=f'Scaled Throttle Step {i}')

# Add 30Hz divisions
for t in np.arange(0, max(time), 1/30):
    plt.axvline(x=t, color='g', linestyle=':', linewidth=0.5, label='30Hz Interval' if t == 0 else "")

# Add labels, title, and legend
plt.title('Thrust vs Time for All Data Sets')
plt.xlabel('Time (seconds)')
plt.ylabel('Thrust')
plt.legend()
plt.grid(True)

# Show the plot
plt.tight_layout()
plt.show()

