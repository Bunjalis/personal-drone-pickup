import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_ukf/hover_20251103_142812/log.csv"
csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_ukf/hover_20251103_143835/log.csv"
csv_path = "/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_ukf/hover_20251103_165335/log.csv"

# Read CSV
df = pd.read_csv(csv_path)

# Confirm required columns exist and pick velocity column
required_cols = ["timestamp", "pose_z","est_pose_z", "mot_cap_z"]
for c in required_cols:
    if c not in df.columns:
        raise KeyError(f"Required column '{c}' not found in CSV. Available columns: {list(df.columns)}")

# Choose z-velocity column (assumption: pose_vz). Change to 'mot_cap_vz' if you prefer.
vel_col = "pose_vz"
if vel_col not in df.columns:
    raise KeyError(f"Velocity column '{vel_col}' not found. Available columns: {list(df.columns)}")

# Convert timestamp to seconds relative to start (timestamps are Unix seconds with fractional part)
t0 = df["timestamp"].iloc[0]
time_s = df["timestamp"] - t0

# Limit to first N seconds for plotting
max_seconds = 10.0  # show only the first 10 seconds
mask = time_s <= max_seconds
if not mask.any():
    raise ValueError(f"No data within the first {max_seconds} seconds (relative time)")
# apply mask to both time and dataframe (reset index for safety)
time_s = time_s[mask].reset_index(drop=True)
df = df.loc[mask].reset_index(drop=True)

# Create figure with three stacked axes sharing X
# top: pose z, mid: z velocities, bottom: control u_2
fig, (ax_top, ax_mid, ax_bot) = plt.subplots(3, 1, sharex=True, figsize=(10, 8), gridspec_kw={"height_ratios": [2, 1, 1]})

# Top axis: pose_z, estimated pose, and mot_cap_z
ax_top.plot(time_s, df["pose_z"], label="pose_z", color="tab:blue", linewidth=1)
if "est_pose_z" in df.columns:
    ax_top.plot(time_s, df["est_pose_z"], label="est_pose_z", color="tab:purple", linewidth=1, linestyle="--")
ax_top.plot(time_s, df["mot_cap_z"], label="mot_cap_z", color="tab:orange", linewidth=1)
ax_top.set_ylabel("z (m)")
ax_top.legend(loc="best")
ax_top.grid(True)

# Mid axis: z velocity (plot both pose_vz and mot_cap_vz if available)
vels_to_plot = []
if "pose_vz" in df.columns:
    vels_to_plot.append("pose_vz")
if "mot_cap_vz" in df.columns:
    vels_to_plot.append("mot_cap_vz")
if "est_pose_vz" in df.columns:
    vels_to_plot.append("est_pose_vz")

if not vels_to_plot:
    raise KeyError("No z-velocity columns found ('pose_vz' or 'mot_cap_vz').")

# Plot velocities on the mid axis
colors = {"pose_vz": "tab:green", "mot_cap_vz": "tab:red", "est_pose_vz": "tab:purple"}
for col in vels_to_plot:
    ls = "--" if col == "est_pose_vz" else "-"
    ax_mid.plot(time_s, df[col], label=col, color=colors.get(col, None), linewidth=1, linestyle=ls)

ax_mid.set_ylabel("z velocity (m/s)")
ax_mid.legend(loc="best")
ax_mid.grid(True)

# Bottom axis: control value u_2 vs time
if "u2" not in df.columns:
    raise KeyError("Required control column 'u2' not found in CSV. Available columns: {list(df.columns)}")
ax_bot.plot(time_s, df["u2"], label="u2", color="tab:cyan", linewidth=1)
ax_bot.set_xlabel("time (s) (relative)")
ax_bot.set_ylabel("u_2 (control)")
ax_bot.legend(loc="best")
ax_bot.grid(True)

plt.tight_layout()

# Save and show
out_path = os.path.join(os.path.dirname(csv_path), "z_and_vz_plot.png")
plt.savefig(out_path, dpi=200)
print(f"Saved plot to {out_path}")
plt.show()