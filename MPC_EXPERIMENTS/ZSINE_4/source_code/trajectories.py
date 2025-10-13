import numpy as np
from scipy.spatial.transform import Rotation as R



def takeoff_trajectory(dt):
    steps_takeoff = 4 * 30  # 1 second of takeoff at 30 Hz
    steps_hover = 4 * 30    # 2 seconds of hover at 30 Hz

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

    steps = 5 * 30  # 10 seconds of hover at 30 Hz
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
    x_traj = 0.0 + 1.0 * np.sin(1.0 * np.pi * time_space / 5.0) #np.zeros_like(time_space)
    y_traj = 0.0 + 1.0 * np.sin(0.5 * np.pi * time_space / 5.0) #np.zeros_like(time_space) # 0.0 + 0.5 * np.sin(1.0 * np.pi * time_space / 5.0)
    z_traj = 1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)#1.2 * np.ones_like(time_space) #1.5 + 0.5 * np.sin(1.5 * np.pi * time_space / 5.0)    #

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
    steps_transition = 5 * 30  # 3 seconds at 30 Hz
    time_space_transition = np.linspace(0, steps_transition * dt, steps_transition)
    
    # Circle parameters
    radius = 1.25  # 1 meter
    period = 4  # seconds per rotation
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


def yaw_trajectory(dt):
    """
    Trajectory that performs takeoff, hover, yaw rotation, then land.
    The drone yaws 90 degrees over 1 second while hovering.
    """
    take_off_traj = takeoff_trajectory(dt)
    
    # Pre-yaw hover (2 seconds)
    steps_pre_hover = 0 * 30  # 2 seconds at 30 Hz
    time_space_pre_hover = np.linspace(0, steps_pre_hover * dt, steps_pre_hover)
    
    # Yaw phase (3 seconds for full 360-degree rotation)
    steps_yaw = 3 * 30  # 3 seconds at 30 Hz
    time_space_yaw = np.linspace(0, steps_yaw * dt, steps_yaw)
    
    # Post-yaw hover (2 seconds)
    steps_post_hover = 5 * 30  # 2 seconds at 30 Hz  
    time_space_post_hover = np.linspace(0, steps_post_hover * dt, steps_post_hover)
    
    # Get takeoff end position
    takeoff_end_x = take_off_traj[0, -1]  # x position
    takeoff_end_y = take_off_traj[1, -1]  # y position
    takeoff_end_z = take_off_traj[2, -1]  # z position
    
    # Position trajectories - maintain hover position throughout
    hover_altitude = 1.0
    x_traj_pre_hover = np.full_like(time_space_pre_hover, takeoff_end_x)
    y_traj_pre_hover = np.full_like(time_space_pre_hover, takeoff_end_y)
    z_traj_pre_hover = np.full_like(time_space_pre_hover, hover_altitude)
    
    x_traj_yaw = np.full_like(time_space_yaw, takeoff_end_x)
    y_traj_yaw = np.full_like(time_space_yaw, takeoff_end_y)
    z_traj_yaw = np.full_like(time_space_yaw, hover_altitude)
    
    x_traj_post_hover = np.full_like(time_space_post_hover, takeoff_end_x)
    y_traj_post_hover = np.full_like(time_space_post_hover, takeoff_end_y)
    z_traj_post_hover = np.full_like(time_space_post_hover, hover_altitude)
    
    # Combine position trajectories
    x_traj_combined = np.concatenate((x_traj_pre_hover, x_traj_yaw, x_traj_post_hover))
    y_traj_combined = np.concatenate((y_traj_pre_hover, y_traj_yaw, y_traj_post_hover))
    z_traj_combined = np.concatenate((z_traj_pre_hover, z_traj_yaw, z_traj_post_hover))
    time_space_combined = np.concatenate((time_space_pre_hover, time_space_yaw, time_space_post_hover))
    
    # Orientation trajectories
    # Pre-yaw: maintain zero orientation
    roll_traj_pre = np.zeros_like(time_space_pre_hover)
    pitch_traj_pre = np.zeros_like(time_space_pre_hover)
    yaw_traj_pre = np.zeros_like(time_space_pre_hover)
    
    # Yaw phase: rotate 360 degrees (2π radians) over 3 seconds
    roll_traj_yaw = np.zeros_like(time_space_yaw)
    pitch_traj_yaw = np.zeros_like(time_space_yaw)
    yaw_traj_yaw = np.zeros_like(time_space_yaw)
    yaw_traj_yaw = np.linspace(0, 2 * np.pi, steps_yaw)  # 360 degree yaw rotation
    
    # Post-yaw: maintain final yaw angle (back to 0 degrees after full rotation)
    roll_traj_post = np.zeros_like(time_space_post_hover)
    pitch_traj_post = np.zeros_like(time_space_post_hover)
    yaw_traj_post = np.full_like(time_space_post_hover, 2 * np.pi)  # Maintain 360 degrees (equivalent to 0)
    
    # Combine orientation trajectories
    roll_traj = np.concatenate((roll_traj_pre, roll_traj_yaw, roll_traj_post))
    pitch_traj = np.concatenate((pitch_traj_pre, pitch_traj_yaw, pitch_traj_post))
    yaw_traj = np.concatenate((yaw_traj_pre, yaw_traj_yaw, yaw_traj_post))
    
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
    
    yaw_hover_traj = np.array([x_traj_combined, y_traj_combined, z_traj_combined, qw_traj, qx_traj, qy_traj, qz_traj,
                              vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    
    land_traj = land_trajectory(dt, yaw_hover_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * 30))
    
    return np.concatenate((take_off_traj, yaw_hover_traj, land_traj, zeros), axis=1)






def power_loop_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)

    # Phase lengths (Hz assumed 30 via your code)
    hz = 30
    steps_runup        = 1 * hz   # 1 s run-up (comment said 3 s previously; leaving as code dictates)
    steps_loop         = 3 * hz   # 3 s loop
    steps_rundown      = 3 * hz   # 3 s run-down
    steps_final_hover  = 2 * hz   # 2 s hover

    time_space_runup        = np.linspace(0, steps_runup * dt, steps_runup, endpoint=False)
    time_space_loop         = np.linspace(0, steps_loop * dt, steps_loop, endpoint=False)
    time_space_rundown      = np.linspace(0, steps_rundown * dt, steps_rundown, endpoint=False)
    time_space_final_hover  = np.linspace(0, steps_final_hover * dt, steps_final_hover, endpoint=False)

    # Takeoff end position
    takeoff_end_x = take_off_traj[0, -1]
    takeoff_end_y = take_off_traj[1, -1]
    takeoff_end_z = take_off_traj[2, -1]

    # Loop geometry
    loop_radius   = 1.1
    loop_center_x = 1.0
    loop_center_z = takeoff_end_z + loop_radius

    # --- KEYFRAMES (θ in DEGREES from bottom) ---
    # Feel free to tweak these. They aim to keep your original “feel”:
    # bottom ~ level, right ~ +30° nose up, top ~ -90° (center-pointing),
    # left ~ -30°, back to bottom ~ level.
    keyframes = [
        {'theta_deg':   0.0, 'pitch_deg':  30.0},
        {'theta_deg':  90.0, 'pitch_deg':  0.0},
        {'theta_deg': 180.0, 'pitch_deg': -100.0},
        {'theta_deg': 270.0, 'pitch_deg': -300.0},
        {'theta_deg': 360.0, 'pitch_deg':   -400.0},
    ]

    # --- Phase 1: Run-up to bottom entry point ---
    x_runup_start = takeoff_end_x
    x_runup_end   = loop_center_x
    z_runup_start = takeoff_end_z
    z_runup_end   = loop_center_z - loop_radius  # bottom of loop

    x_traj_runup = np.linspace(x_runup_start, x_runup_end, steps_runup, endpoint=False)
    y_traj_runup = np.full_like(time_space_runup, takeoff_end_y)
    z_traj_runup = np.linspace(z_runup_start, z_runup_end, steps_runup, endpoint=False)

    # --- Phase 2: Power loop (θ from 0° -> 360°) ---
    theta_deg_loop = np.linspace(0.0, 360.0, steps_loop, endpoint=False)

    # Convert bottom-based degrees (0° at bottom) to standard circle param φ (0° at +x/right).
    # Bottom corresponds to φ=270°, so φ = θ + 270 (mod 360).
    phi_deg = (theta_deg_loop + 270.0) % 360.0
    phi_rad = np.deg2rad(phi_deg)

    x_traj_loop = loop_center_x + loop_radius * np.cos(phi_rad)
    y_traj_loop = np.full_like(time_space_loop, takeoff_end_y)
    z_traj_loop = loop_center_z + loop_radius * np.sin(phi_rad)

    # --- Phase 3: Run-down back to hover position ---
    x_rundown_start = loop_center_x
    x_rundown_end   = takeoff_end_x
    z_rundown_start = loop_center_z - loop_radius
    z_rundown_end   = takeoff_end_z

    x_traj_rundown = np.linspace(x_rundown_start, x_rundown_end, steps_rundown, endpoint=False)
    y_traj_rundown = np.full_like(time_space_rundown, takeoff_end_y)
    z_traj_rundown = np.linspace(z_rundown_start, z_rundown_end, steps_rundown, endpoint=False)

    # --- Phase 4: Final hover ---
    x_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_x)
    y_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_y)
    z_traj_final_hover = np.full_like(time_space_final_hover, takeoff_end_z)

    # Combine position trajectories
    x_traj_combined = np.concatenate((x_traj_runup, x_traj_loop, x_traj_rundown, x_traj_final_hover))
    y_traj_combined = np.concatenate((y_traj_runup, y_traj_loop, y_traj_rundown, y_traj_final_hover))
    z_traj_combined = np.concatenate((z_traj_runup, z_traj_loop, z_traj_rundown, z_traj_final_hover))
    time_space_combined = np.concatenate((time_space_runup, time_space_loop, time_space_rundown, time_space_final_hover))

    # --- Orientation (roll=0, yaw=0; pitch from keyframes) ---
    roll_traj = np.zeros_like(time_space_combined)
    yaw_traj  = np.zeros_like(time_space_combined)
    pitch_traj = np.zeros_like(time_space_combined)

    # Phase 1: pitch up slightly for entry
    pitch_start = 0.0
    pitch_end   = np.deg2rad(20.0)
    pitch_traj[:steps_runup] = np.linspace(pitch_start, pitch_end, steps_runup, endpoint=False)

    # Phase 2: pitch from keyframes along θ
    loop_start_idx = steps_runup
    loop_end_idx   = steps_runup + steps_loop

    # Sort and unwrap keyframes in degrees [0,360], allow wrap-around interpolation
    kf_thetas = np.array([kf['theta_deg'] for kf in keyframes], dtype=float)
    kf_pitchs = np.deg2rad(np.array([kf['pitch_deg'] for kf in keyframes], dtype=float))
    sort_idx = np.argsort(kf_thetas)
    kf_thetas = kf_thetas[sort_idx]
    kf_pitchs = kf_pitchs[sort_idx]

    # For fast lookup, pre-build arrays for piecewise-linear interpolation with wrap:
    # Extend by one keyframe at +360° for continuous interpolation at the end.
    kf_thetas_ext = np.concatenate((kf_thetas, [kf_thetas[0] + 360.0]))
    kf_pitchs_ext = np.concatenate((kf_pitchs, [kf_pitchs[0]]))

    def interp_pitch_deg_from_bottom(theta_deg):
        """Linear interpolate pitch (radians) from keyframes at θ in degrees [0,360)."""
        # Find segment
        idx = np.searchsorted(kf_thetas_ext, theta_deg, side='right') - 1
        idx = np.clip(idx, 0, len(kf_thetas_ext) - 2)
        t1, p1 = kf_thetas_ext[idx], kf_pitchs_ext[idx]
        t2, p2 = kf_thetas_ext[idx + 1], kf_pitchs_ext[idx + 1]
        if t2 == t1:
            return p1
        w = (theta_deg - t1) / (t2 - t1)
        return p1 + w * (p2 - p1)

    for i, th in enumerate(theta_deg_loop):
        pitch_traj[loop_start_idx + i] = interp_pitch_deg_from_bottom(th)

    # Phase 3: pitch based on path slope (keeps a “natural” rundown)
    vx_runup_rundown = np.concatenate((
        np.gradient(x_traj_runup, dt),
        np.gradient(x_traj_rundown, dt),
        np.gradient(x_traj_final_hover, dt)
    ))
    vz_runup_rundown = np.concatenate((
        np.gradient(z_traj_runup, dt),
        np.gradient(z_traj_rundown, dt),
        np.gradient(z_traj_final_hover, dt)
    ))

    rundown_start_idx = loop_end_idx
    rundown_end_idx   = rundown_start_idx + steps_rundown
    eps = 1e-2
    pitch_traj[rundown_start_idx:rundown_end_idx] = np.arctan2(
        vz_runup_rundown[steps_runup:steps_runup + steps_rundown],
        np.clip(np.abs(vx_runup_rundown[steps_runup:steps_runup + steps_rundown]), eps, None)
    )

    # Phase 4: level
    pitch_traj[rundown_end_idx:] = 0.0

    # Convert RPY to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quats = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj, qy_traj, qz_traj, qw_traj = quats[:,0], quats[:,1], quats[:,2], quats[:,3]

    # Velocities
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)

    # Angular rates (time-derivatives of RPY)
    roll_rate  = np.gradient(roll_traj, dt)
    pitch_rate = np.gradient(pitch_traj, dt)
    yaw_rate   = np.gradient(yaw_traj, dt)

    ax_traj = roll_rate
    ay_traj = pitch_rate
    az_traj = yaw_rate

    # Placeholders for thrust/inputs
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined)
    u4 = np.zeros_like(time_space_combined)

    power_loop_traj = np.array([
        x_traj_combined, y_traj_combined, z_traj_combined,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])

    land_traj = land_trajectory(dt, power_loop_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * hz))

    return np.concatenate((take_off_traj, power_loop_traj, land_traj, zeros), axis=1)


def christmas_tree_spiral_trajectory(dt):
    """
    Christmas tree/cone spiral trajectory:
    - Starts at top center (2m height)
    - Spirals down to 1.5m radius at 1m height
    - Spirals back up to top center
    """
    take_off_traj = takeoff_trajectory(dt)
    
    # Phase durations
    hz = 30
    steps_ascent = 5 * hz      # 3s to reach top of tree
    steps_spiral_down = 30 * hz # 15s spiraling down
    steps_spiral_up = 30 * hz   # 15s spiraling back up
    steps_final_hover = 5 * hz  # 2s hover at top
    
    # Tree parameters
    tree_top_height = 2.0      # Top of tree at 2m
    tree_bottom_height = 1.0   # Bottom of spiral at 1m
    max_radius = 1.0           # Maximum radius at bottom
    
    # Takeoff end position
    takeoff_end_x = take_off_traj[0, -1]
    takeoff_end_y = take_off_traj[1, -1] 
    takeoff_end_z = take_off_traj[2, -1]
    
    # Phase 1: Ascent to tree top center
    time_space_ascent = np.linspace(0, steps_ascent * dt, steps_ascent)
    x_traj_ascent = np.linspace(takeoff_end_x, 0.0, steps_ascent)  # Move to center
    y_traj_ascent = np.linspace(takeoff_end_y, 0.0, steps_ascent)  # Move to center
    z_traj_ascent = np.linspace(takeoff_end_z, tree_top_height, steps_ascent)
    
    # Phase 2: Spiral down (cone shape)
    time_space_spiral_down = np.linspace(0, steps_spiral_down * dt, steps_spiral_down)
    
    # Height decreases linearly from top to bottom
    z_traj_spiral_down = np.linspace(tree_top_height, tree_bottom_height, steps_spiral_down)
    
    # Radius increases linearly as we go down (cone shape)
    # At top (z=2m): radius = 0
    # At bottom (z=1m): radius = max_radius
    height_progress = (tree_top_height - z_traj_spiral_down) / (tree_top_height - tree_bottom_height)
    radius_spiral_down = max_radius * height_progress
    
    # Angular position - multiple spirals on the way down
    num_spirals_down = 3  # 3 full rotations going down
    theta_spiral_down = np.linspace(0, num_spirals_down * 2 * np.pi, steps_spiral_down)
    
    x_traj_spiral_down = radius_spiral_down * np.cos(theta_spiral_down)
    y_traj_spiral_down = radius_spiral_down * np.sin(theta_spiral_down)
    
    # Phase 3: Spiral up (reverse cone)
    time_space_spiral_up = np.linspace(0, steps_spiral_up * dt, steps_spiral_up)
    
    # Height increases linearly from bottom to top
    z_traj_spiral_up = np.linspace(tree_bottom_height, tree_top_height, steps_spiral_up)
    
    # Radius decreases linearly as we go up
    height_progress_up = (z_traj_spiral_up - tree_bottom_height) / (tree_top_height - tree_bottom_height)
    radius_spiral_up = max_radius * (1 - height_progress_up)
    
    # Angular position - continue from where spiral down ended, add more rotations
    num_spirals_up = 3  # 3 full rotations going up
    theta_start_up = theta_spiral_down[-1]  # Continue from end of down spiral
    theta_spiral_up = np.linspace(theta_start_up, theta_start_up + num_spirals_up * 2 * np.pi, steps_spiral_up)
    
    x_traj_spiral_up = radius_spiral_up * np.cos(theta_spiral_up)
    y_traj_spiral_up = radius_spiral_up * np.sin(theta_spiral_up)
    
    # Phase 4: Final hover at top center
    time_space_final_hover = np.linspace(0, steps_final_hover * dt, steps_final_hover)
    x_traj_final_hover = np.zeros_like(time_space_final_hover)
    y_traj_final_hover = np.zeros_like(time_space_final_hover)
    z_traj_final_hover = np.full_like(time_space_final_hover, tree_top_height)
    
    # Combine all phases
    x_traj_combined = np.concatenate((x_traj_ascent, x_traj_spiral_down, x_traj_spiral_up, x_traj_final_hover))
    y_traj_combined = np.concatenate((y_traj_ascent, y_traj_spiral_down, y_traj_spiral_up, y_traj_final_hover))
    z_traj_combined = np.concatenate((z_traj_ascent, z_traj_spiral_down, z_traj_spiral_up, z_traj_final_hover))
    time_space_combined = np.concatenate((time_space_ascent, time_space_spiral_down, time_space_spiral_up, time_space_final_hover))
    
    # Orientation - maintain level flight with slight banking in turns
    roll_traj = np.zeros_like(time_space_combined)
    pitch_traj = np.zeros_like(time_space_combined)
    yaw_traj = np.zeros_like(time_space_combined)
    
    # Add banking during spiral phases
    bank_angle = np.deg2rad(15)  # 15 degree bank angle
    
    # Calculate angular velocity for banking
    ascent_end = steps_ascent
    spiral_down_end = ascent_end + steps_spiral_down
    spiral_up_end = spiral_down_end + steps_spiral_up
    
    # Spiral down banking
    omega_down = np.gradient(theta_spiral_down, dt)
    for i in range(steps_spiral_down):
        if abs(omega_down[i]) > 0.1:  # Only bank when turning
            roll_traj[ascent_end + i] = -bank_angle * np.sign(omega_down[i])
    
    # Spiral up banking  
    omega_up = np.gradient(theta_spiral_up, dt)
    for i in range(steps_spiral_up):
        if abs(omega_up[i]) > 0.1:  # Only bank when turning
            roll_traj[spiral_down_end + i] = -bank_angle * np.sign(omega_up[i])
    
    # Convert RPY to quaternions
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1] 
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    
    # Calculate velocities
    vx_traj = np.gradient(x_traj_combined, dt)
    vy_traj = np.gradient(y_traj_combined, dt)
    vz_traj = np.gradient(z_traj_combined, dt)
    
    # Calculate angular rates
    roll_rate = np.gradient(roll_traj, dt)
    pitch_rate = np.gradient(pitch_traj, dt)
    yaw_rate = np.gradient(yaw_traj, dt)
    
    ax_traj = roll_rate
    ay_traj = pitch_rate
    az_traj = yaw_rate
    
    # Control inputs (placeholders)
    u1 = np.zeros_like(time_space_combined)
    u2 = np.zeros_like(time_space_combined)
    u3 = np.zeros_like(time_space_combined) 
    u4 = np.zeros_like(time_space_combined)
    
    christmas_tree_traj = np.array([
        x_traj_combined, y_traj_combined, z_traj_combined,
        qw_traj, qx_traj, qy_traj, qz_traj,
        vx_traj, vy_traj, vz_traj,
        ax_traj, ay_traj, az_traj,
        u1, u2, u3, u4
    ])
    
    land_traj = land_trajectory(dt, christmas_tree_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 1 * hz))
    
    return np.concatenate((take_off_traj, christmas_tree_traj, land_traj, zeros), axis=1)
