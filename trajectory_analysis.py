import pandas as pd
import matplotlib.pyplot as plt

# Load the state errors CSV file
data = pd.read_csv('state_errors.csv')

# Create a figure with 4 vertical subplots
fig, axes = plt.subplots(4, 1, figsize=(10, 15), sharex=True)

# Plot position (X, Y, Z) against setpoints
axes[0].plot(data['Step'], data['Last_Position_X'], label='Measured X', color='blue')
axes[0].plot(data['Step'], data['Setpoint_X'], label='Setpoint X', color='red', linestyle='dashed')
axes[0].plot(data['Step'], data['Last_Position_Y'], label='Measured Y', color='green')
axes[0].plot(data['Step'], data['Setpoint_Y'], label='Setpoint Y', color='orange', linestyle='dashed')
axes[0].plot(data['Step'], data['Last_Position_Z'], label='Measured Z', color='purple')
axes[0].plot(data['Step'], data['Setpoint_Z'], label='Setpoint Z', color='brown', linestyle='dashed')
axes[0].set_title('Position (X, Y, Z)')
axes[0].legend()
axes[0].grid(True)

# Plot orientation (W, X, Y, Z) against setpoints
axes[1].plot(data['Step'], data['Last_Orientation_W'], label='Measured W', color='blue')
axes[1].plot(data['Step'], data['Setpoint_Orientation_W'], label='Setpoint W', color='red', linestyle='dashed')
axes[1].plot(data['Step'], data['Last_Orientation_X'], label='Measured X', color='green')
axes[1].plot(data['Step'], data['Setpoint_Orientation_X'], label='Setpoint X', color='orange', linestyle='dashed')
axes[1].plot(data['Step'], data['Last_Orientation_Y'], label='Measured Y', color='purple')
axes[1].plot(data['Step'], data['Setpoint_Orientation_Y'], label='Setpoint Y', color='brown', linestyle='dashed')
axes[1].plot(data['Step'], data['Last_Orientation_Z'], label='Measured Z', color='pink')
axes[1].plot(data['Step'], data['Setpoint_Orientation_Z'], label='Setpoint Z', color='gray', linestyle='dashed')
axes[1].set_title('Orientation (W, X, Y, Z)')
axes[1].legend()
axes[1].grid(True)

# Plot linear velocity (X, Y, Z) against setpoints
axes[2].plot(data['Step'], data['Last_Linear_Velocity_X'], label='Measured X', color='blue')
axes[2].plot(data['Step'], data['Setpoint_VX'], label='Setpoint X', color='red', linestyle='dashed')
axes[2].plot(data['Step'], data['Last_Linear_Velocity_Y'], label='Measured Y', color='green')
axes[2].plot(data['Step'], data['Setpoint_VY'], label='Setpoint Y', color='orange', linestyle='dashed')
axes[2].plot(data['Step'], data['Last_Linear_Velocity_Z'], label='Measured Z', color='purple')
axes[2].plot(data['Step'], data['Setpoint_VZ'], label='Setpoint Z', color='brown', linestyle='dashed')
axes[2].set_title('Linear Velocity (X, Y, Z)')
axes[2].legend()
axes[2].grid(True)

# Plot angular velocity (X, Y, Z) against setpoints
axes[3].plot(data['Step'], data['Last_Angular_Velocity_X'], label='Measured X', color='blue')
axes[3].plot(data['Step'], data['Setpoint_AX'], label='Setpoint X', color='red', linestyle='dashed')
axes[3].plot(data['Step'], data['Last_Angular_Velocity_Y'], label='Measured Y', color='green')
axes[3].plot(data['Step'], data['Setpoint_AY'], label='Setpoint Y', color='orange', linestyle='dashed')
axes[3].plot(data['Step'], data['Last_Angular_Velocity_Z'], label='Measured Z', color='purple')
axes[3].plot(data['Step'], data['Setpoint_AZ'], label='Setpoint Z', color='brown', linestyle='dashed')
axes[3].set_title('Angular Velocity (X, Y, Z)')
axes[3].legend()
axes[3].grid(True)

# Set common labels
plt.xlabel('Step')
plt.tight_layout()

# Show the plot
plt.show()