import pandas as pd
import matplotlib.pyplot as plt



'''
        self.motion_capture_csv_writer.writerow([
                        'r_px', 'r_py', 'r_pz',
                        'r_rw', 'r_rx', 'r_ry', 'r_rz',
                        'r_vx', 'r_vy', 'r_vz',
                        'r_wx', 'r_wy', 'r_wz',
                        'rolling_vx', 'rolling_vy', 'rolling_vz',
                        'low_pass_vx', 'low_pass_vy', 'low_pass_vz',
                        'butter_vx', 'butter_vy', 'butter_vz',
                        'rolling_wx', 'rolling_wy', 'rolling_wz',
                        'low_pass_wx', 'low_pass_wy', 'low_pass_wz',
                        'butter_wx', 'butter_wy', 'butter_wz',
        ])
        '''

# Sampling frequency
sampling_frequency = 240  # Hz
time_step = 1 / sampling_frequency

# Load motion capture results
motion_capture_results = pd.read_csv('motion_capture_raw.csv')
motion_capture_results = motion_capture_results.iloc[60:].reset_index(drop=True)
time_motion = motion_capture_results.index * time_step

# Plot motion capture results
fig1, axs1 = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
fig1.suptitle('Motion Capture Results')


# X linear velocity
axs1[0].plot(time_motion, motion_capture_results['r_vx'], label='raw_vx')
axs1[0].plot(time_motion, motion_capture_results['rolling_vx'], label='rol_vx')
axs1[0].plot(time_motion, motion_capture_results['low_pass_vx'], label='low_vx')
axs1[0].plot(time_motion, motion_capture_results['butter_vx'], label='but_vx')
axs1[0].set_ylabel('X Linear Velocity')
axs1[0].legend()

# Y linear velocity
axs1[1].plot(time_motion, motion_capture_results['r_vy'], label='raw_vy')
axs1[1].plot(time_motion, motion_capture_results['rolling_vy'], label='rol_vy')
axs1[1].plot(time_motion, motion_capture_results['low_pass_vy'], label='low_vy')
axs1[1].plot(time_motion, motion_capture_results['butter_vy'], label='but_vy')
axs1[1].set_ylabel('Y Linear Velocity')
axs1[1].legend()

# Z linear velocity
axs1[2].plot(time_motion, motion_capture_results['r_vz'], label='raw_vz')
axs1[2].plot(time_motion, motion_capture_results['rolling_vz'], label='rol_vz')
axs1[2].plot(time_motion, motion_capture_results['low_pass_vz'], label='low_vz')
axs1[2].plot(time_motion, motion_capture_results['butter_vz'], label='but_vz')
axs1[2].set_ylabel('Z Linear Velocity')
axs1[2].legend()

# X angular velocity
axs1[3].plot(time_motion, motion_capture_results['r_wx'],label='raw_wx')
axs1[3].plot(time_motion, motion_capture_results['rolling_wx'], label='rol_wx')
axs1[3].plot(time_motion, motion_capture_results['low_pass_wx'], label='low_wx')
axs1[3].plot(time_motion, motion_capture_results['butter_wx'], label='but_wx')
axs1[3].set_ylabel('X Angular Velocity')
axs1[3].legend()

# Y angular velocity
axs1[4].plot(time_motion, motion_capture_results['r_wy'],label='raw_wy')
axs1[4].plot(time_motion, motion_capture_results['rolling_wy'], label='rol_wy')
axs1[4].plot(time_motion, motion_capture_results['low_pass_wy'], label='low_wy')
axs1[4].plot(time_motion, motion_capture_results['butter_wy'], label='but_wy')
axs1[4].set_ylabel('Y Angular Velocity')
axs1[4].legend()

# Z angular velocity
axs1[5].plot(time_motion, motion_capture_results['r_wz'],label='raw_wz')
axs1[5].plot(time_motion, motion_capture_results['rolling_wz'], label='rol_wz')
axs1[5].plot(time_motion, motion_capture_results['low_pass_wz'], label='low_wz')
axs1[5].plot(time_motion, motion_capture_results['butter_wz'], label='but_wz')
axs1[5].set_ylabel('Z Angular Velocity')
axs1[5].legend()





axs1[5].set_xlabel('Time (s)')
# Show plots
plt.show()