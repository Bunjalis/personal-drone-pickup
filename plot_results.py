import pandas as pd
import matplotlib.pyplot as plt

# Sampling frequency
sampling_frequency = 240  # Hz
time_step = 1 / sampling_frequency



#motion_capture_file_path = 'misc/trajectory_results/15_motion_capture_results.csv'
#control_file_path = 'misc/trajectory_results/15_control_results.csv'




#motion_capture_file_path = 'motion_capture_results.csv'
control_file_path = '8_without_l1_50g_control_results.csv'


'''
# Load motion capture results
motion_capture_results = pd.read_csv(motion_capture_file_path)
time_motion = motion_capture_results.index * time_step

# Plot motion capture results
fig1, axs1 = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
fig1.suptitle('Motion Capture Results')

# Position
axs1[0].plot(time_motion, motion_capture_results['px'], label='px')
axs1[0].plot(time_motion, motion_capture_results['py'], label='py')
axs1[0].plot(time_motion, motion_capture_results['pz'], label='pz')
axs1[0].set_ylabel('Position')
axs1[0].legend()

# Orientation
axs1[1].plot(time_motion, motion_capture_results['rw'], label='rw')
axs1[1].plot(time_motion, motion_capture_results['rx'], label='rx')
axs1[1].plot(time_motion, motion_capture_results['ry'], label='ry')
axs1[1].plot(time_motion, motion_capture_results['rz'], label='rz')
axs1[1].set_ylabel('Orientation')
axs1[1].legend()

# Linear Velocity
axs1[2].plot(time_motion, motion_capture_results['vx'], label='vx')
axs1[2].plot(time_motion, motion_capture_results['vy'], label='vy')
axs1[2].plot(time_motion, motion_capture_results['vz'], label='vz')
axs1[2].set_ylabel('Linear Velocity')
axs1[2].legend()

# Angular Velocity
axs1[3].plot(time_motion, motion_capture_results['wx'], label='wx')
axs1[3].plot(time_motion, motion_capture_results['wy'], label='wy')
axs1[3].plot(time_motion, motion_capture_results['wz'], label='wz')
axs1[3].set_ylabel('Angular Velocity')
axs1[3].set_xlabel('Time (s)')
axs1[3].legend()
'''


# Load control results

sampling_frequency = 30  # Hz
time_step = 1 / sampling_frequency
control_results = pd.read_csv(control_file_path)
time_control = control_results.index * time_step

# Plot control results
fig2, axs2 = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
fig2.suptitle('Control Results')

# Position
axs2[0].plot(time_control, control_results['px'], label='Actual px')
axs2[0].plot(time_control, control_results['py'], label='Actual py')
axs2[0].plot(time_control, control_results['pz'], label='Actual pz')
axs2[0].plot(time_control, control_results['sp_px'], linestyle='dotted', label='Desired px')
axs2[0].plot(time_control, control_results['sp_py'], linestyle='dotted', label='Desired py')
axs2[0].plot(time_control, control_results['sp_pz'], linestyle='dotted', label='Desired pz')
axs2[0].set_ylabel('Position')
axs2[0].legend()

# Orientation
axs2[1].plot(time_control, control_results['rw'], label='Actual rw')
axs2[1].plot(time_control, control_results['rx'], label='Actual rx')
axs2[1].plot(time_control, control_results['ry'], label='Actual ry')
axs2[1].plot(time_control, control_results['rz'], label='Actual rz')
axs2[1].plot(time_control, control_results['sp_rw'], linestyle='dotted', label='Desired rw')
axs2[1].plot(time_control, control_results['sp_rx'], linestyle='dotted', label='Desired rx')
axs2[1].plot(time_control, control_results['sp_ry'], linestyle='dotted', label='Desired ry')
axs2[1].plot(time_control, control_results['sp_rz'], linestyle='dotted', label='Desired rz')
axs2[1].set_ylabel('Orientation')
axs2[1].legend()

# Linear Velocity
axs2[2].plot(time_control, control_results['vx'], label='Actual vx')
axs2[2].plot(time_control, control_results['vy'], label='Actual vy')
axs2[2].plot(time_control, control_results['vz'], label='Actual vz')
axs2[2].plot(time_control, control_results['sp_vx'], linestyle='dotted', label='Desired vx')
axs2[2].plot(time_control, control_results['sp_vy'], linestyle='dotted', label='Desired vy')
axs2[2].plot(time_control, control_results['sp_vz'], linestyle='dotted', label='Desired vz')
axs2[2].set_ylabel('Linear Velocity')
axs2[2].legend()

# Angular Velocity
axs2[3].plot(time_control, control_results['wx'], label='Actual wx')
axs2[3].plot(time_control, control_results['wy'], label='Actual wy')
axs2[3].plot(time_control, control_results['wz'], label='Actual wz')
axs2[3].plot(time_control, control_results['sp_wx'], linestyle='dotted', label='Desired wx')
axs2[3].plot(time_control, control_results['sp_wy'], linestyle='dotted', label='Desired wy')
axs2[3].plot(time_control, control_results['sp_wz'], linestyle='dotted', label='Desired wz')
axs2[3].set_ylabel('Angular Velocity')
axs2[3].set_xlabel('Time (s)')
axs2[3].legend()

# Show plots
plt.show()