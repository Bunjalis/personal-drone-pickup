import numpy as np
from scipy.spatial.transform import Rotation as R




def takeoff_trajectory(dt):
    steps_takeoff = 1 * 30  # 1 second of takeoff at 30 Hz
    steps_hover = 2 * 30    # 2 seconds of hover at 30 Hz

    # Takeoff phase
    time_space_takeoff = np.linspace(0, steps_takeoff * dt, steps_takeoff)
    x_traj_takeoff = np.zeros_like(time_space_takeoff)
    y_traj_takeoff = np.zeros_like(time_space_takeoff)
    z_traj_takeoff = np.linspace(0.0, 1.0, steps_takeoff)

    # Hover phase
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = np.ones_like(time_space_hover) * 1.0

    # Concatenate phases
    x_traj = np.concatenate((x_traj_takeoff, x_traj_hover))
    y_traj = np.concatenate((y_traj_takeoff, y_traj_hover))
    z_traj = np.concatenate((z_traj_takeoff, z_traj_hover))
    time_space_total = np.concatenate((time_space_takeoff, time_space_hover))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 30  # 5 seconds of takeoff at 30 Hz
    steps_descend = 3 * 30  # 5 seconds of takeoff at 30 Hz
    time_space_move_back = np.linspace(0, steps_move_back * dt, steps_move_back)
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    time_space_total = np.concatenate((time_space_move_back, time_space_descend))

    x_traj_move_back = np.linspace(init_pose[0], 0, steps_move_back)  # Move back 1 meter
    x_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    x_traj = np.concatenate((x_traj_move_back, x_traj_descend))

    y_traj_move_back = np.linspace(init_pose[1], 0, steps_move_back)  # Move back 1 meter
    y_traj_descend = np.linspace(0, 0, steps_descend)  # Move back 1 meter
    y_traj = np.concatenate((y_traj_move_back, y_traj_descend))

    z_traj_move_back = np.linspace(init_pose[2], 1.0, steps_move_back)  # Move back 1 meter
    z_traj_descend  = np.linspace(1.0, 0.1, steps_descend)  # Move back 1 meter
    z_traj = np.concatenate((z_traj_move_back, z_traj_descend))

    roll_traj = np.zeros_like(time_space_total)
    pitch_traj = np.zeros_like(time_space_total)
    yaw_traj = np.zeros_like(time_space_total)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_total)
    ay_traj = np.zeros_like(time_space_total)
    az_traj = np.zeros_like(time_space_total)
    u1 = np.zeros_like(time_space_total)
    u2 = np.zeros_like(time_space_total)
    u3 = np.zeros_like(time_space_total)
    u4 = np.zeros_like(time_space_total)
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])



def hover_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    steps = 20 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space) #np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 * np.ones_like(time_space)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)




def z_sin_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    # Phase 1: Hover for 20 seconds
    steps_hover = 20 * 30  # 20 seconds of hover at 30 Hz
    time_space_hover = np.linspace(0, steps_hover * dt, steps_hover)
    x_traj_hover = np.zeros_like(time_space_hover)
    y_traj_hover = np.zeros_like(time_space_hover)
    z_traj_hover = 1.5 * np.ones_like(time_space_hover)  # Constant hover at 1.5m
    
    # Phase 2: Variable frequency sinusoidal motion for 40 seconds
    steps_sin = 40 * 30  # 40 seconds of sinusoidal motion at 30 Hz
    time_space_sin = np.linspace(0, steps_sin * dt, steps_sin)
    
    # Frequency increases linearly from 1.0 to 3.0 Hz over 40 seconds
    freq_start = 0.1  # Hz
    freq_end = 0.5    # Hz
    frequencies = np.linspace(freq_start, freq_end, steps_sin)
    
    # Calculate the instantaneous phase by integrating frequency
    # phase(t) = 2π * ∫frequency(τ)dτ from 0 to t
    phase = 2 * np.pi * np.cumsum(frequencies) * dt
    
    x_traj_sin = np.zeros_like(time_space_sin)
    y_traj_sin = np.zeros_like(time_space_sin)
    z_traj_sin = 1.5 + 1.0 * np.sin(phase)  # 1m amplitude around 1.5m center
    
    # Combine both phases
    x_traj = np.concatenate((x_traj_hover, x_traj_sin))
    y_traj = np.concatenate((y_traj_hover, y_traj_sin))
    z_traj = np.concatenate((z_traj_hover, z_traj_sin))
    time_space = np.concatenate((time_space_hover, time_space_sin))

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)





def xyz_sine_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    steps = 20 * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = 0.0 + 1.0 * np.sin(9.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    y_traj = 0.0 + 1.0 * np.sin(9.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 + 0.5 * np.sin(3.0 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj =  np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                 vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj = land_trajectory(dt, take_off_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj,hover_traj, land_traj,zeros), axis=1)





def circle_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    # Transition from hover to circle start (3 seconds)
    steps_transition = 3 * 30  # 3 seconds at 30 Hz
    time_space_transition = np.linspace(0, steps_transition * dt, steps_transition)
    
    # Circle parameters
    radius = 1.25  # 1 meter
    period = 2.5  # seconds per rotation
    omega = 2 * np.pi / period  # angular velocity (rad/s)
    
    # Get the final position from takeoff (should be 0, 0, 1.0)
    takeoff_end_x = take_off_traj[0, -1]  # x position
    takeoff_end_y = take_off_traj[1, -1]  # y position
    takeoff_end_z = take_off_traj[2, -1]  # z position
    
    # Transition trajectory from takeoff end to circle start
    x_traj_transition = np.linspace(takeoff_end_x, radius, steps_transition)  # Move to circle start
    y_traj_transition = np.linspace(takeoff_end_y, 0.0, steps_transition)     # Move to y=0
    z_traj_transition = np.linspace(takeoff_end_z, 1.5, steps_transition)     # Move to circle altitude

    steps = 30 * 30  # 40 seconds at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)

    x_traj = radius * np.cos(omega * time_space)
    y_traj = radius * np.sin(omega * time_space)
    z_traj = 1.5 * np.ones_like(time_space)  # constant altitude

    # Combine transition and circle time spaces for trajectory calculations
    time_space_combined = np.concatenate((time_space_transition, time_space))
    x_traj_combined = np.concatenate((x_traj_transition, x_traj))
    y_traj_combined = np.concatenate((y_traj_transition, y_traj))
    z_traj_combined = np.concatenate((z_traj_transition, z_traj))

    roll_traj = np.zeros_like(time_space_combined)
    pitch_traj = np.zeros_like(time_space_combined)
    yaw_traj = np.zeros_like(time_space_combined)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)
    ax_traj = np.zeros_like(time_space_combined)
    ay_traj = np.zeros_like(time_space_combined)
    az_traj = np.zeros_like(time_space_combined)
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined)
    u4 = np.zeros_like(time_space_combined)
    hover_traj = np.array([x_traj_combined, y_traj_combined, z_traj_combined, qw_traj, qx_traj, qy_traj, qz_traj,
                           vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj, hover_traj, land_traj, zeros), axis=1)



def light_circle_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    # Circle parameters
    circles = [
        {'height': 1.0, 'radius': 0.75, 'period': 10.0},  # Circle 1
        {'height': 1.25, 'radius': 0.5, 'period': 10.0},  # Circle 2
        {'height': 1.5, 'radius': 0.25, 'period': 10.0}   # Circle 3
    ]
    
    loops_per_circle = 5
    transition_time = 4.0  # seconds between circles
    
    # Initialize trajectory arrays
    x_traj_total = []
    y_traj_total = []
    z_traj_total = []
    
    for i, circle in enumerate(circles):
        # Execute 3 loops for this circle
        loop_duration = circle['period'] * loops_per_circle
        steps_loop = int(loop_duration * 30)  # 30 Hz
        time_space_loop = np.linspace(0, loop_duration, steps_loop)
        omega = 2 * np.pi / circle['period']
        
        # Circle trajectory
        x_circle = circle['radius'] * np.cos(omega * time_space_loop)
        y_circle = circle['radius'] * np.sin(omega * time_space_loop)
        z_circle = circle['height'] * np.ones_like(time_space_loop)
        
        x_traj_total.extend(x_circle)
        y_traj_total.extend(y_circle)
        z_traj_total.extend(z_circle)
        
        # Add transition to next circle (except after the last circle)
        if i < len(circles) - 1:
            steps_transition = int(transition_time * 30)  # 2 seconds at 30 Hz
            
            # Start and end positions for transition
            start_x = x_circle[-1]
            start_y = y_circle[-1]
            start_z = z_circle[-1]
            
            next_circle = circles[i + 1]
            end_x = next_circle['radius']  # Start next circle at (radius, 0)
            end_y = 0.0
            end_z = next_circle['height']
            
            # Linear interpolation for transition
            x_transition = np.linspace(start_x, end_x, steps_transition)
            y_transition = np.linspace(start_y, end_y, steps_transition)
            z_transition = np.linspace(start_z, end_z, steps_transition)
            
            x_traj_total.extend(x_transition)
            y_traj_total.extend(y_transition)
            z_traj_total.extend(z_transition)
    
    # Convert to numpy arrays
    x_traj = np.array(x_traj_total)
    y_traj = np.array(y_traj_total)
    z_traj = np.array(z_traj_total)
    
    # Create time array for the entire trajectory
    time_space = np.arange(len(x_traj)) * dt

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.zeros_like(time_space)
    u1 = np.zeros_like(time_space)
    u2 = np.zeros_like(time_space)
    u3 = np.zeros_like(time_space)
    u4 = np.zeros_like(time_space)
    hover_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                           vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    land_traj = land_trajectory(dt, hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))

    return np.concatenate((take_off_traj, hover_traj, land_traj, zeros), axis=1)


def backflip_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    
    # Backsault parameters
    backsault_duration = 2.0  # seconds to complete backsault
    steps_backsault = int(backsault_duration * 30)  # 30 Hz
    time_space_backsault = np.linspace(0, backsault_duration, steps_backsault)
    
    # Hover before backsault (2 seconds)
    steps_pre_hover = 2 * 30
    time_space_pre_hover = np.linspace(0, steps_pre_hover * dt, steps_pre_hover)
    
    # Hover after backsault (2 seconds)
    steps_post_hover = 2 * 30
    time_space_post_hover = np.linspace(0, steps_post_hover * dt, steps_post_hover)
    
    # Get takeoff end position
    takeoff_end_x = take_off_traj[0, -1]  # x position
    takeoff_end_y = take_off_traj[1, -1]  # y position
    takeoff_end_z = take_off_traj[2, -1]  # z position
    
    # Pre-hover phase - maintain position
    x_traj_pre_hover = np.full_like(time_space_pre_hover, takeoff_end_x)
    y_traj_pre_hover = np.full_like(time_space_pre_hover, takeoff_end_y)
    z_traj_pre_hover = np.full_like(time_space_pre_hover, 1.5)  # Higher altitude for safety
    
    # Backsault phase - arc trajectory with 0.5m radius
    radius = 0.5  # 0.5 meter radius for the backsault
    center_x = takeoff_end_x - radius  # Center of the arc is 0.5m behind starting position
    center_z = 1.5  # Same altitude as hover
    
    # Parametric arc from 0 to π (semicircle)
    theta = np.linspace(0, np.pi, steps_backsault)  # From 0 to π for backward arc
    x_traj_backsault = center_x + radius * np.cos(theta)  # Moves backward then forward
    y_traj_backsault = np.full_like(time_space_backsault, takeoff_end_y)  # No y movement
    z_traj_backsault = center_z + radius * np.sin(theta)  # Arc up and down
    
    # Post-hover phase - maintain final position
    final_x = x_traj_backsault[-1]  # Should be back to starting x position
    x_traj_post_hover = np.full_like(time_space_post_hover, final_x)
    y_traj_post_hover = np.full_like(time_space_post_hover, takeoff_end_y)
    z_traj_post_hover = np.full_like(time_space_post_hover, 1.5)
    
    # Combine position trajectories
    x_traj_combined = np.concatenate((x_traj_pre_hover, x_traj_backsault, x_traj_post_hover))
    y_traj_combined = np.concatenate((y_traj_pre_hover, y_traj_backsault, y_traj_post_hover))
    z_traj_combined = np.concatenate((z_traj_pre_hover, z_traj_backsault, z_traj_post_hover))
    time_space_combined = np.concatenate((time_space_pre_hover, time_space_backsault, time_space_post_hover))
    
    # Orientation trajectories
    roll_traj_pre = np.zeros_like(time_space_pre_hover)
    pitch_traj_pre = np.zeros_like(time_space_pre_hover)
    yaw_traj_pre = np.zeros_like(time_space_pre_hover)
    
    # Backsault: complete 360-degree rotation around pitch axis synchronized with arc
    roll_traj_backsault = np.zeros_like(time_space_backsault)
    pitch_traj_backsault = np.linspace(0, 2 * np.pi, steps_backsault)  # Full rotation
    yaw_traj_backsault = np.zeros_like(time_space_backsault)
    
    roll_traj_post = np.zeros_like(time_space_post_hover)
    pitch_traj_post = np.zeros_like(time_space_post_hover)
    yaw_traj_post = np.zeros_like(time_space_post_hover)
    
    # Combine orientation trajectories
    roll_traj = np.concatenate((roll_traj_pre, roll_traj_backsault, roll_traj_post))
    pitch_traj = np.concatenate((pitch_traj_pre, pitch_traj_backsault, pitch_traj_post))
    yaw_traj = np.concatenate((yaw_traj_pre, yaw_traj_backsault, yaw_traj_post))
    
    # Convert to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities and angular rates
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)
    
    # Calculate angular rates (derivatives of roll, pitch, yaw)
    roll_rate = np.gradient(roll_traj, dt)  # Angular velocity around x-axis
    pitch_rate = np.gradient(pitch_traj, dt)  # Angular velocity around y-axis
    yaw_rate = np.gradient(yaw_traj, dt)  # Angular velocity around z-axis
    
    # Desired angular velocities for the trajectory
    ax_traj = roll_rate   # Desired angular velocity around x-axis
    ay_traj = pitch_rate  # Desired angular velocity around y-axis
    az_traj = yaw_rate    # Desired angular velocity around z-axis
    
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined)
    u4 = np.zeros_like(time_space_combined)
    
    backsault_traj = np.array([x_traj_combined, y_traj_combined, z_traj_combined, qw_traj, qx_traj, qy_traj, qz_traj,
                              vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    
    land_traj = land_trajectory(dt, backsault_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, backsault_traj, land_traj, zeros), axis=1)
