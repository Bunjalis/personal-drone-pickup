import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.spatial.transform import Rotation

#csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/hover_with_orb.csv"
csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/xyz_mot_with_orb.csv"
#csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/hover_inside_house.csv"
#csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/LogsToKeep/zsin_inside_house.csv"

# Read CSV
df = pd.read_csv(csv_path)

# Function to convert quaternions to RPY angles
def quat_to_rpy(qw, qx, qy, qz):
    """Convert quaternion (qw, qx, qy, qz) to roll, pitch, yaw in degrees"""
    quats = np.array([qx, qy, qz, qw])  # scipy uses xyzw order
    r = Rotation.from_quat(quats)
    rpy_rad = r.as_euler('xyz')  # roll, pitch, yaw in radians
    return np.degrees(rpy_rad)  # convert to degrees

# Extract RPY from quaternions
mot_rpy = np.array([quat_to_rpy(row['mot_pose_qw'], row['mot_pose_qx'], row['mot_pose_qy'], row['mot_pose_qz']) 
                     for _, row in df.iterrows()])
est_rpy = np.array([quat_to_rpy(row['est_pose_qw'], row['est_pose_qx'], row['est_pose_qy'], row['est_pose_qz']) 
                     for _, row in df.iterrows()])
orb_rpy = np.array([quat_to_rpy(row['orb_pose_qw'], row['orb_pose_qx'], row['orb_pose_qy'], row['orb_pose_qz']) 
                     for _, row in df.iterrows()])

# Add RPY columns to dataframe
df['mot_roll'] = mot_rpy[:, 0]
df['mot_pitch'] = mot_rpy[:, 1]
df['mot_yaw'] = mot_rpy[:, 2]
df['est_roll'] = est_rpy[:, 0]
df['est_pitch'] = est_rpy[:, 1]
df['est_yaw'] = est_rpy[:, 2]
df['orb_roll'] = orb_rpy[:, 0]
df['orb_pitch'] = orb_rpy[:, 1]
df['orb_yaw'] = orb_rpy[:, 2]

# Extract angular velocities (already in CSV as mot_pose_av* and est_pose_av*)
# These are the angular velocities around x, y, z axes

# Confirm required columns exist and pick velocity column
required_cols = ["timestamp", "mot_pose_z", "est_pose_z", "mot_pose_vx", "est_pose_vx"]
for c in required_cols:
    if c not in df.columns:
        raise KeyError(f"Required column '{c}' not found in CSV. Available columns: {list(df.columns)}")

# Choose x-velocity columns for comparison
vel_cols = ["mot_pose_vx", "est_pose_vx"]
for vel_col in vel_cols:
    if vel_col not in df.columns:
        raise KeyError(f"Velocity column '{vel_col}' not found. Available columns: {list(df.columns)}")

# Convert timestamp to seconds relative to start (timestamps are Unix seconds with fractional part)
t0 = df["timestamp"].iloc[0]
time_s = df["timestamp"] - t0

# Offset mot_pose_z by -0.1 m
df["mot_pose_z"] = df["mot_pose_z"] - 0.1

# Create figure with 12 stacked axes sharing X
# 1: x position, 2: y position, 3: z position, 4: vx velocity, 5: vy velocity, 6: vz velocity, 7: roll, 8: pitch, 9: yaw, 10: avx, 11: avy, 12: avz
fig, axes = plt.subplots(12, 1, sharex=True, figsize=(12, 20))

# Axis 0: x position
axes[0].plot(time_s, df["mot_pose_x"], label="mot_pose_x", color="tab:blue", linewidth=1)
axes[0].plot(time_s, df["orb_pose_x"], label="orb_pose_x", color="tab:green", linewidth=1, linestyle=":")
if "est_pose_x" in df.columns:
    axes[0].plot(time_s, df["est_pose_x"], label="est_pose_x", color="tab:purple", linewidth=1, linestyle="--")
axes[0].set_ylabel("x (m)")
axes[0].legend(loc="best")
axes[0].grid(True)

# Axis 1: y position
axes[1].plot(time_s, df["mot_pose_y"], label="mot_pose_y", color="tab:blue", linewidth=1)
axes[1].plot(time_s, df["orb_pose_y"], label="orb_pose_y", color="tab:green", linewidth=1, linestyle=":")
if "est_pose_y" in df.columns:
    axes[1].plot(time_s, df["est_pose_y"], label="est_pose_y", color="tab:purple", linewidth=1, linestyle="--")
axes[1].set_ylabel("y (m)")
axes[1].legend(loc="best")
axes[1].grid(True)

# Axis 2: z position
axes[2].plot(time_s, df["mot_pose_z"], label="mot_pose_z", color="tab:blue", linewidth=1)
axes[2].plot(time_s, df["orb_pose_z"], label="orb_pose_z", color="tab:green", linewidth=1, linestyle=":")
if "est_pose_z" in df.columns:
    axes[2].plot(time_s, df["est_pose_z"], label="est_pose_z", color="tab:purple", linewidth=1, linestyle="--")
axes[2].set_ylabel("z (m)")
axes[2].legend(loc="best")
axes[2].grid(True)

# Axis 3: x velocity
axes[3].plot(time_s, df["mot_pose_vx"], label="mot_pose_vx", color="tab:blue", linewidth=1)
axes[3].plot(time_s, df["orb_pose_vx"], label="orb_pose_vx", color="tab:green", linewidth=1, linestyle=":")
axes[3].plot(time_s, df["est_pose_vx"], label="est_pose_vx", color="tab:purple", linewidth=1, linestyle="--")
axes[3].set_ylabel("vx (m/s)")
axes[3].legend(loc="best")
axes[3].grid(True)

# Axis 4: y velocity
axes[4].plot(time_s, df["mot_pose_vy"], label="mot_pose_vy", color="tab:blue", linewidth=1)
axes[4].plot(time_s, df["orb_pose_vy"], label="orb_pose_vy", color="tab:green", linewidth=1, linestyle=":")
axes[4].plot(time_s, df["est_pose_vy"], label="est_pose_vy", color="tab:purple", linewidth=1, linestyle="--")
axes[4].set_ylabel("vy (m/s)")
axes[4].legend(loc="best")
axes[4].grid(True)

# Axis 5: z velocity
axes[5].plot(time_s, df["mot_pose_vz"], label="mot_pose_vz", color="tab:blue", linewidth=1)
axes[5].plot(time_s, df["orb_pose_vz"], label="orb_pose_vz", color="tab:green", linewidth=1, linestyle=":")
axes[5].plot(time_s, df["est_pose_vz"], label="est_pose_vz", color="tab:purple", linewidth=1, linestyle="--")
axes[5].set_ylabel("vz (m/s)")
axes[5].legend(loc="best")
axes[5].grid(True)

# Axis 6: Roll
axes[6].plot(time_s, df["mot_roll"], label="mot_roll", color="tab:red", linewidth=1)
axes[6].plot(time_s, df["orb_roll"], label="orb_roll", color="tab:green", linewidth=1, linestyle=":")
axes[6].plot(time_s, df["est_roll"], label="est_roll", color="tab:orange", linewidth=1, linestyle="--")
if "u1" in df.columns:
    # Assuming u1 is normalized roll command, scale by 55 degrees
    axes[6].plot(time_s, df["u0"] * 55.0, label="u1_roll_des", color="tab:pink", linewidth=1, linestyle="-.")
axes[6].set_ylabel("roll (deg)")
axes[6].legend(loc="best")
axes[6].grid(True)

# Axis 7: Pitch
axes[7].plot(time_s, df["mot_pitch"], label="mot_pitch", color="tab:red", linewidth=1)
axes[7].plot(time_s, df["orb_pitch"], label="orb_pitch", color="tab:green", linewidth=1, linestyle=":")
axes[7].plot(time_s, df["est_pitch"], label="est_pitch", color="tab:orange", linewidth=1, linestyle="--")
if "u2" in df.columns:
    # Assuming u2 is normalized pitch command, scale by 55 degrees
    axes[7].plot(time_s, df["u1"] * 55.0, label="u2_pitch_des", color="tab:pink", linewidth=1, linestyle="-.")
axes[7].set_ylabel("pitch (deg)")
axes[7].legend(loc="best")
axes[7].grid(True)

# Axis 8: Yaw
axes[8].plot(time_s, df["mot_yaw"], label="mot_yaw", color="tab:red", linewidth=1)
axes[8].plot(time_s, df["orb_yaw"], label="orb_yaw", color="tab:green", linewidth=1, linestyle=":")
axes[8].plot(time_s, df["est_yaw"], label="est_yaw", color="tab:orange", linewidth=1, linestyle="--")
axes[8].set_ylabel("yaw (deg)")
axes[8].legend(loc="best")
axes[8].grid(True)

# Axis 9: Angular velocity around x (roll rate)
axes[9].plot(time_s, df["mot_pose_avx"], label="mot_pose_avx", color="tab:blue", linewidth=1)
axes[9].plot(time_s, df["orb_pose_avx"], label="orb_pose_avx", color="tab:green", linewidth=1, linestyle=":")
axes[9].plot(time_s, df["est_pose_avx"], label="est_pose_avx", color="tab:purple", linewidth=1, linestyle="--")
axes[9].set_ylabel("avx (rad/s)")
axes[9].legend(loc="best")
axes[9].grid(True)

# Axis 10: Angular velocity around y (pitch rate)
axes[10].plot(time_s, df["mot_pose_avy"], label="mot_pose_avy", color="tab:blue", linewidth=1)
axes[10].plot(time_s, df["orb_pose_avy"], label="orb_pose_avy", color="tab:green", linewidth=1, linestyle=":")
axes[10].plot(time_s, df["est_pose_avy"], label="est_pose_avy", color="tab:purple", linewidth=1, linestyle="--")
axes[10].set_ylabel("avy (rad/s)")
axes[10].legend(loc="best")
axes[10].grid(True)

# Axis 11: Angular velocity around z (yaw rate)
axes[11].plot(time_s, df["mot_pose_avz"], label="mot_pose_avz", color="tab:cyan", linewidth=1)
axes[11].plot(time_s, df["orb_pose_avz"], label="orb_pose_avz", color="tab:green", linewidth=1, linestyle=":")
axes[11].plot(time_s, df["est_pose_avz"], label="est_pose_avz", color="tab:brown", linewidth=1, linestyle="--")
axes[11].set_xlabel("time (s) (relative)")
axes[11].set_ylabel("avz (rad/s)")
axes[11].legend(loc="best")
axes[11].grid(True)

plt.tight_layout()

# Save and show
out_path = os.path.join(os.path.dirname(csv_path), "z_and_vz_plot.png")
plt.savefig(out_path, dpi=200)
print(f"Saved plot to {out_path}")
plt.show()