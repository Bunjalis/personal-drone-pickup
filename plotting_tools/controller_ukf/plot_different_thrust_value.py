import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Plot settings
FIGURE_SIZE = (1, 3)
MAIN_TITLE_SIZE = 24
SUBPLOT_TITLE_SIZE = 26
AXIS_LABEL_SIZE = 24
TICK_LABEL_SIZE = 24
LEGEND_SIZE = 22
LINE_WIDTH = 2
GRID_ALPHA = 0.3

# Define the color palette
colors = ['#e97d00', '#008e00', '#0049bd', '#911515', '#000000', '#97c6d1', '#b697ff']

# Define the log file paths
log_files = [
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/forward_z_sin_20251202_123653/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/forward_z_sin_20251202_123529/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/forward_z_sin_20251202_123359/log.csv',
    '/home/mitchell/Documents/PhD/drone_cage_control/logs/controller_angle_ukf/forward_z_sin_20251202_123233/log.csv',
]

# Labels for the legend
labels = [r'$k_{t,0} = 15$', r'$k_{t,0} = 20$', r'$k_{t,0} = 25$', r'$k_{t,0} = 30$']

# Create figure with 2 subplots
fig, axes = plt.subplots(2, 1, figsize=(FIGURE_SIZE[0] * 10, FIGURE_SIZE[1] * 2))
fig.subplots_adjust(hspace=0.3)

# Create separate figure for legend
fig_legend = plt.figure(figsize=(10, 0.5))

# Store handles and labels for legend
legend_handles = []
legend_labels = []

# Read and plot data from each log file
for idx, (log_file, label) in enumerate(zip(log_files, labels)):
    try:
        # Read the CSV file
        df = pd.read_csv(log_file)
        
        # Calculate time from timestamp
        time = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Filter data to first 48 seconds
        mask = time <= 60
        time = time[mask]
        
        # Extract data
        z_position = df['ukf_pose_z'][mask]
        z_desired = df['traj_z_ref'][mask]
        thrust_estimate = df['est_param_thrust_ratio'][mask]
        
        color = colors[idx]
        
        # Plot 1: Z position and desired position
        line1, = axes[0].plot(time, z_position, color=color, linewidth=LINE_WIDTH, label=label)
        axes[0].plot(time, z_desired, color='#000000', linewidth=LINE_WIDTH, linestyle='--', alpha=0.7)
        
        # Plot 2: Estimated thrust value
        axes[1].plot(time, thrust_estimate, color=color, linewidth=LINE_WIDTH, label=label)
        
        # Store for legend
        legend_handles.append(line1)
        legend_labels.append(label)
        
    except FileNotFoundError:
        print(f"Warning: File not found: {log_file}")
    except Exception as e:
        print(f"Error processing {log_file}: {e}")

# Add desired trajectory to legend last
from matplotlib.lines import Line2D
desired_line = Line2D([0], [0], color='#000000', linewidth=LINE_WIDTH, linestyle='--', alpha=0.7)

# Configure axis 0: Z Position
axes[0].set_ylabel('Altitude (m)', fontsize=AXIS_LABEL_SIZE)
axes[0].grid(True, alpha=GRID_ALPHA)
axes[0].set_title('(a) Altitude Tracking Performance', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[0].set_xlim(0, 60)
axes[0].set_xticks(np.arange(0, 40, 10))
axes[0].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Configure axis 1: Thrust Estimate
axes[1].set_ylabel(r'Thrust Constant $k_t$', fontsize=AXIS_LABEL_SIZE)
axes[1].set_xlabel('Time (s)', fontsize=AXIS_LABEL_SIZE)
axes[1].grid(True, alpha=GRID_ALPHA)
axes[1].set_title(r'(b) Estimated Thrust Constant $k_t$', fontsize=SUBPLOT_TITLE_SIZE, fontweight='bold')
axes[1].set_xlim(0, 60)
axes[1].set_xticks(np.arange(0, 40, 10))
axes[1].tick_params(axis='both', which='major', labelsize=TICK_LABEL_SIZE)

# Create legend in separate figure
# Manually reorder to go across first (left to right) instead of down
# With ncol=3, we want: [15, 20, 25] on row 1, [30, Desired] on row 2
# Matplotlib fills columns first, so for 3 cols and 5 items it would naturally do:
# [item0, item3] [item1, item4] [item2, empty]
# We want: [item0, item1, item2] [item3, item4, empty]
# So reorder as: [15(0), 30(3), 20(1), Desired(4), 25(2)]
all_handles = legend_handles + [desired_line]
all_labels = legend_labels + ['Desired Height']
# Reorder for row-first layout with ncol=3
row_first_handles = [all_handles[0], all_handles[3], all_handles[1], all_handles[4], all_handles[2]]
row_first_labels = [all_labels[0], all_labels[3], all_labels[1], all_labels[4], all_labels[2]]

fig_legend.legend(row_first_handles, row_first_labels, 
                  loc='center', ncol=3, frameon=True, fontsize=LEGEND_SIZE, borderaxespad=0)
fig_legend.tight_layout()

# Adjust main figure layout
fig.tight_layout()

# Save figures as PDFs
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
fig.savefig(os.path.join(script_dir, 'thrust_constant_comparison.pdf'), format='pdf', bbox_inches='tight')
fig_legend.savefig(os.path.join(script_dir, 'thrust_constant_comparison_legend.pdf'), format='pdf', bbox_inches='tight')
print(f"Saved figures to {script_dir}")

# Show both figures
plt.show()
