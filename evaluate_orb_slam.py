import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the CSV file
data = pd.read_csv('motion_comparison_test3.csv')

# Extract m_px, m_py, and m_pz columns
m_px = data['m_px']
m_py = data['m_py']
m_pz = data['m_pz']

# Normalize the initial offset between motion capture and ORB SLAM data
initial_m = data[['m_px', 'm_py', 'm_pz']].iloc[0]
data['m_px'] -= initial_m['m_px']
data['m_py'] -= initial_m['m_py']
data['m_pz'] -= initial_m['m_pz']

initial_o = data[['o_px', 'o_py', 'o_pz']].iloc[0]
data['o_px'] -= initial_o['o_px']
data['o_py'] -= initial_o['o_py']
data['o_pz'] -= initial_o['o_pz']

# Convert ORB SLAM data from centimeters to meters
data['o_px'] *= 9
data['o_py'] *= 9
data['o_pz'] *= 9

# Extract ORB SLAM data
o_px = data['o_px']
o_py = data['o_py']
o_pz = data['o_pz']

data['o_vz'] *= 9

# Apply a moving average filter of width 5 to the ORB SLAM velocity data
#data['o_vx'] = data['o_vx'].rolling(window=5, center=True).mean()
#data['o_vy'] = data['o_vy'].rolling(window=5, center=True).mean()
#data['o_vz'] = data['o_vz'].rolling(window=5, center=True).mean()

# Plot z-velocity comparison
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(data['m_vz'], label='Measured Z Velocity', color='blue')
ax.plot(data['o_vz'], label='ORB SLAM Z Velocity', color='red')
ax.set_title('Z Velocity Comparison')
ax.set_xlabel('Time Step')
ax.set_ylabel('Z Velocity (m/s)')
ax.legend()
plt.show()

# Ensure 3D plot has equal axis scaling
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(m_px, m_py, m_pz, label='Motion Capture', color='blue')
ax.plot(o_px, o_py, o_pz, label='ORB SLAM', color='red')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

# Set equal scaling for 3D plot
max_range = max(m_px.max() - m_px.min(), m_py.max() - m_py.min(), m_pz.max() - m_pz.min())
mid_x = (m_px.max() + m_px.min()) * 0.5
mid_y = (m_py.max() + m_py.min()) * 0.5
mid_z = (m_pz.max() + m_pz.min()) * 0.5
ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

plt.show()