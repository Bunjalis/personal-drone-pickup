import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.spatial.transform import Rotation

csv_path = "/home/benji/drone_cage_control/logs/controller_ukf/u_pickup_20260309_142357/log.csv"

# Read CSV
df = pd.read_csv(csv_path)

# Function to convert quaternions to RPY angles
def quat_to_rpy(qw, qx, qy, qz):
    quats = np.array([qx, qy, qz, qw])  # scipy uses xyzw order
    r = Rotation.from_quat(quats)
    rpy_rad = r.as_euler('xyz')
    return np.degrees(rpy_rad)

# Convert timestamp to seconds relative to start
t0 = df["timestamp"].iloc[0]
time_s = df["timestamp"] - t0

# Extract RPY from pose quaternions
pose_rpy = np.array([quat_to_rpy(row['pose_qw'], row['pose_qx'], row['pose_qy'], row['pose_qz'])
                     for _, row in df.iterrows()])
df['pose_roll']  = pose_rpy[:, 0]
df['pose_pitch'] = pose_rpy[:, 1]
df['pose_yaw']   = pose_rpy[:, 2]

# Extract RPY from estimated pose quaternions
est_rpy = np.array([quat_to_rpy(row['est_pose_qw'], row['est_pose_qx'], row['est_pose_qy'], row['est_pose_qz'])
                    for _, row in df.iterrows()])
df['est_roll']  = est_rpy[:, 0]
df['est_pitch'] = est_rpy[:, 1]
df['est_yaw']   = est_rpy[:, 2]

# Check if trajectory reference data exists
has_traj_ref = all(col in df.columns for col in [
    'traj_x_ref', 'traj_y_ref', 'traj_z_ref',
    'traj_qw_ref', 'traj_qx_ref', 'traj_qy_ref', 'traj_qz_ref'
])
if has_traj_ref:
    traj_rpy = np.array([quat_to_rpy(row['traj_qw_ref'], row['traj_qx_ref'], row['traj_qy_ref'], row['traj_qz_ref'])
                         for _, row in df.iterrows()])
    df['traj_roll_ref']  = traj_rpy[:, 0]
    df['traj_pitch_ref'] = traj_rpy[:, 1]
    df['traj_yaw_ref']   = traj_rpy[:, 2]

# Create figure: 11 rows x 2 cols (left: signals, right: error pose - est)
fig, axes = plt.subplots(11, 2, sharex=True, figsize=(20, 30))

def plot_row(ax_left, ax_right, time_s, pose_col, est_col, traj_col, ylabel, error_ylabel):
    ax_left.plot(time_s, df[pose_col], label=pose_col, color="tab:blue", linewidth=1)
    ax_left.plot(time_s, df[est_col],  label=est_col,  color="tab:purple", linewidth=1, linestyle="--")
    if traj_col and traj_col in df.columns:
        ax_left.plot(time_s, df[traj_col], label=traj_col, color="black", linewidth=1.5, linestyle="-")
    ax_left.set_ylabel(ylabel)
    ax_left.legend(loc="best", fontsize=7)
    ax_left.grid(True)

    if traj_col and traj_col in df.columns:
        ax_right.plot(time_s, df[est_col] - df[traj_col], label=f"{est_col} - {traj_col}", color="tab:red", linewidth=1)
    else:
        ax_right.plot(time_s, df[est_col], label=est_col, color="tab:red", linewidth=1)
    ax_right.axhline(0, color='black', linestyle='--', linewidth=0.5)
    ax_right.set_ylabel(error_ylabel)
    ax_right.legend(loc="best", fontsize=7)
    ax_right.grid(True)

# Row 0: X position
plot_row(axes[0,0], axes[0,1], time_s, "pose_x", "est_pose_x", "traj_x_ref", "x (m)", "x error (m)")
axes[0,0].set_title("Pose vs Estimated (and reference)")
axes[0,1].set_title("Error (est - traj)")

# Row 1: Y position
plot_row(axes[1,0], axes[1,1], time_s, "pose_y", "est_pose_y", "traj_y_ref", "y (m)", "y error (m)")

# Row 2: Z position
plot_row(axes[2,0], axes[2,1], time_s, "pose_z", "est_pose_z", "traj_z_ref", "z (m)", "z error (m)")

# Row 3: vx
plot_row(axes[3,0], axes[3,1], time_s, "pose_vx", "est_pose_vx", None, "vx (m/s)", "vx error (m/s)")

# Row 4: vy
plot_row(axes[4,0], axes[4,1], time_s, "pose_vy", "est_pose_vy", None, "vy (m/s)", "vy error (m/s)")

# Row 5: vz
plot_row(axes[5,0], axes[5,1], time_s, "pose_vz", "est_pose_vz", None, "vz (m/s)", "vz error (m/s)")

# Row 6: Roll
plot_row(axes[6,0], axes[6,1], time_s, "pose_roll", "est_roll",
         "traj_roll_ref" if has_traj_ref else None, "roll (deg)", "roll error (deg) est-traj")
if "u0" in df.columns:
    axes[6,0].plot(time_s, df["u0"] * 55.0, label="u0*55 (roll des)", color="tab:pink", linewidth=1, linestyle="-.")
    axes[6,0].legend(loc="best", fontsize=7)

# Row 7: Pitch
plot_row(axes[7,0], axes[7,1], time_s, "pose_pitch", "est_pitch",
         "traj_pitch_ref" if has_traj_ref else None, "pitch (deg)", "pitch error (deg)")
if "u1" in df.columns:
    axes[7,0].plot(time_s, df["u1"] * 55.0, label="u1*55 (pitch des)", color="tab:pink", linewidth=1, linestyle="-.")
    axes[7,0].legend(loc="best", fontsize=7)

# Row 8: Yaw
plot_row(axes[8,0], axes[8,1], time_s, "pose_yaw", "est_yaw",
         "traj_yaw_ref" if has_traj_ref else None, "yaw (deg)", "yaw error (deg)")

# Row 9: Thrust ratio
if "est_param_thrust_ratio" in df.columns:
    axes[9,0].plot(time_s, df["est_param_thrust_ratio"], label="thrust_ratio", color="tab:blue", linewidth=1.5)
    axes[9,0].set_ylabel("kT")
    axes[9,0].legend(loc="best", fontsize=7)
    axes[9,0].grid(True)
    axes[9,0].set_title("Estimated Thrust Ratio")

# Row 9 right: tau_rate
if "est_param_tau_rate" in df.columns:
    axes[9,1].plot(time_s, df["est_param_tau_rate"], label="tau_rate", color="tab:orange", linewidth=1.5)
    axes[9,1].set_ylabel("tau_rate")
    axes[9,1].legend(loc="best", fontsize=7)
    axes[9,1].grid(True)
    axes[9,1].set_title("Estimated Tau Rate")

# Row 10: Drag coefficient
if "est_param_drag_coeff_z" in df.columns:
    axes[10,0].plot(time_s, df["est_param_drag_coeff_z"], label="drag_coeff_z", color="tab:green", linewidth=1.5)
    axes[10,0].set_xlabel("time (s)")
    axes[10,0].set_ylabel("drag coeff")
    axes[10,0].legend(loc="best", fontsize=7)
    axes[10,0].grid(True)
    axes[10,0].set_title("Estimated Drag Coefficient Z")

# Row 10 right: Computation times
timing_cols = [c for c in ["MPC_setup_time", "MPC_solve_time", "UKF_update_time"] if c in df.columns]
for col in timing_cols:
    axes[10,1].plot(time_s, df[col], label=col, linewidth=1)
axes[10,1].set_xlabel("time (s)")
axes[10,1].set_ylabel("time (s)")
axes[10,1].legend(loc="best", fontsize=7)
axes[10,1].grid(True)
axes[10,1].set_title("Computation Times")

plt.tight_layout()

out_path = os.path.join(os.path.dirname(csv_path), "comparison_and_error_plot.png")
plt.savefig(out_path, dpi=200)
print(f"Saved plot to {out_path}")
plt.show()