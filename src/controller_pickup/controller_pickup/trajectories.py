import numpy as np
from scipy.spatial.transform import Rotation as R

def m_pickup_trajectory(dt):

    takeoff_x, takeoff_y = 0.0, 1.0
    landing_x, landing_y = 0.0, -1.0
    peak_height = 1.2
    pickup_height = 0.0
    wait_time = 10

    steps_per_segment = 80  # adjust for speed
    t = np.linspace(0, 1, steps_per_segment)

    # takeoff
    take_off_traj = takeoff_helper(dt, takeoff_x, takeoff_y, peak_height)

    # hover before starting
    wait_start_traj = hover_helper(dt, takeoff_x, takeoff_y, peak_height, wait_time)

    # Arc down
    arc_clearance = 0.3
    z_linear_desc = peak_height + (pickup_height - peak_height) * t
    z_arc_desc = arc_clearance * np.sin(np.pi * t)
    y1 = 1 - t
    x1 = np.full(steps_per_segment, 0.0)  # constant X
    z1 = z_linear_desc + z_arc_desc

    # Arc up
    z_linear_asc = pickup_height + (peak_height - pickup_height) * t
    z_arc_asc = arc_clearance * np.sin(np.pi * t)
    y2 = 0 - t
    x2 = np.full(steps_per_segment, 0.0)  # constant X
    z2 = z_linear_asc + z_arc_asc


    # Combine segments
    x_traj = np.concatenate((x1, x2))
    y_traj = np.concatenate((y1, y2))
    z_traj = np.concatenate((z1, z2))

    # Yaw: face along trajectory (optional: along Y)
    dx = np.gradient(x_traj)
    dy = np.gradient(y_traj)
    yaw_traj = np.arctan2(dy, dx)

    roll_traj = np.zeros_like(yaw_traj)
    pitch_traj = np.zeros_like(yaw_traj)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:,0]
    qy_traj = quaternions[:,1]
    qz_traj = quaternions[:,2]
    qw_traj = quaternions[:,3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(vx_traj)
    ay_traj = np.zeros_like(vy_traj)
    az_traj = np.zeros_like(vz_traj)
    u1 = np.zeros_like(vx_traj)
    u2 = np.zeros_like(vx_traj)
    u3 = np.zeros_like(vx_traj)
    u4 = np.zeros_like(vx_traj)

    # m shape
    hover_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    # hover after completing
    wait_end_traj = hover_helper(dt, landing_x, landing_y, peak_height, 5)
    
    # land
    land_traj = land_helper(dt, hover_traj[:, -1], landing_x, landing_y)
    #zeros = np.tile(land_traj[:, -1:], (1, 30))

    traj = np.concatenate((take_off_traj ,wait_start_traj, hover_traj, wait_end_traj, land_traj), axis=1)
    return traj, "m_pickup"

def u_pickup_trajectory(dt):

    takeoff_x, takeoff_y = 0.0, 1.5
    landing_x, landing_y = 0.0, -1.5
    pickup_x, pickup_y = 0.0, 0.0
    peak_height = 1.2
    pickup_height = -0.05
    wait_time = 5
    pickup_length = 0.8 / 2 # divide by 2 as it is calculated per side

    steps_per_segment = 80  # adjust for speed
    t = np.linspace(0, 1, steps_per_segment)

    # takeoff
    take_off_traj = takeoff_helper(dt, takeoff_x, takeoff_y, peak_height)

    # hover before starting
    wait_start_traj = hover_helper(dt, takeoff_x, takeoff_y, peak_height, wait_time)

    # swoop down
    x1 = np.linspace(takeoff_x, pickup_x, steps_per_segment)
    y1 = np.linspace(takeoff_y, pickup_y + pickup_length, steps_per_segment)
    z1 = peak_height + (pickup_height - peak_height) * (0.5 - 0.5 * np.cos(np.pi * t))

    # flyby and grab
    # multiplying steps_per_segment so speed roughly scales with pickup_length
    steps_pickup = int(np.floor(steps_per_segment * pickup_length * 1.5))
    x2 = np.full(steps_pickup, pickup_x) 
    y2 = np.linspace(pickup_y + pickup_length, pickup_y - pickup_length, steps_pickup)
    z2 = np.full(steps_pickup, pickup_height)

    # swoop up
    x3 = np.linspace(pickup_x, landing_x, steps_per_segment)
    y3 = np.linspace(pickup_y - pickup_length, landing_y, steps_per_segment)
    z3 = pickup_height + (peak_height - pickup_height) * (0.5 - 0.5 * np.cos(np.pi * t))


    # Combine segments
    x_traj = np.concatenate((x1, x2, x3))
    y_traj = np.concatenate((y1, y2, y3))
    z_traj = np.concatenate((z1, z2, z3))

    # Yaw: face along trajectory (optional: along Y)
    dx = np.gradient(x_traj)
    dy = np.gradient(y_traj)
    yaw_traj = np.arctan2(dy, dx)

    roll_traj = np.zeros_like(yaw_traj)
    pitch_traj = np.zeros_like(yaw_traj)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:,0]
    qy_traj = quaternions[:,1]
    qz_traj = quaternions[:,2]
    qw_traj = quaternions[:,3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(vx_traj)
    ay_traj = np.zeros_like(vy_traj)
    az_traj = np.zeros_like(vz_traj)
    u1 = np.zeros_like(vx_traj)
    u2 = np.zeros_like(vx_traj)
    u3 = np.zeros_like(vx_traj)
    u4 = np.zeros_like(vx_traj)

    # m shape
    hover_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    # hover after completing
    wait_end_traj = hover_helper(dt, landing_x, landing_y, peak_height, 5)
    
    # land
    land_traj = land_helper(dt, hover_traj[:, -1], landing_x, landing_y)
    #zeros = np.tile(land_traj[:, -1:], (1, 30))

    traj = np.concatenate((take_off_traj ,wait_start_traj, hover_traj, wait_end_traj, land_traj), axis=1)
    return traj, "u_pickup"

def cutoff_pickup_trajectory(dt):

    takeoff_x, takeoff_y = 0.0, 1.5
    landing_x, landing_y = 0.0, -1.5
    pickup_x, pickup_y = 0.0, 0.0
    
    wait_time = 10

    # use these to figure out when rotor cut off should be using s = ut + 0.5at^2, t = sqrt(2s/a), then s = vt
    pickup_speed = 1
    peak_height = 1.2
    pickup_height = 0.0

    # time to drop from peak height to pickup height
    time_drop = np.sqrt((2*(peak_height - pickup_height)) / 9.8)

    # the y location the drop should begin
    begin_drop = (pickup_speed * time_drop) - pickup_y

    steps_approach = int(30 * ((takeoff_y - begin_drop) / pickup_speed))
    steps_drop = int(30 * time_drop)
    steps_up = steps_drop


    # takeoff
    take_off_traj = takeoff_helper(dt, takeoff_x, takeoff_y, peak_height)

    # hover before starting
    wait_start_traj = hover_helper(dt, takeoff_x, takeoff_y, peak_height, wait_time)

    # approach
    x1 = np.linspace(takeoff_x, pickup_x, steps_approach)
    y1 = np.linspace(takeoff_y, begin_drop, steps_approach)
    z1 = np.full(steps_approach, peak_height)

    # drop and grab
    # multiplying steps_per_segment so speed roughly scales with pickup_length
    t = np.linspace(0, 1, steps_drop)
    x2 = np.full(steps_drop, pickup_x) 
    y2 = np.linspace(begin_drop, pickup_y , steps_drop)
    real_t = t * time_drop
    z2 = peak_height - pickup_height - 0.5 * 9.81 * (real_t**2)

    # up
    x3 = np.linspace(pickup_x, landing_x, steps_up)
    y3 = np.linspace(pickup_y , landing_y, steps_up)
    z3 = np.linspace(pickup_height, peak_height, steps_up)

    # Combine segments
    x_traj = np.concatenate((x1, x2, x3))
    y_traj = np.concatenate((y1, y2, y3))
    z_traj = np.concatenate((z1, z2, z3))

    # Yaw: face along trajectory (optional: along Y)
    dx = np.gradient(x_traj)
    dy = np.gradient(y_traj)
    yaw_traj = np.arctan2(dy, dx)

    roll_traj = np.zeros_like(yaw_traj)
    pitch_traj = np.zeros_like(yaw_traj)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:,0]
    qy_traj = quaternions[:,1]
    qz_traj = quaternions[:,2]
    qw_traj = quaternions[:,3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(vx_traj)
    ay_traj = np.zeros_like(vy_traj)
    az_traj = np.zeros_like(vz_traj)
    u1 = np.zeros_like(vx_traj)
    u2 = np.zeros_like(vx_traj)
    u3 = np.zeros_like(vx_traj)
    u4 = np.zeros_like(vx_traj)

    # m shape
    hover_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])

    # hover after completing
    wait_end_traj = hover_helper(dt, landing_x, landing_y, peak_height, 5)
    
    # land
    land_traj = land_helper(dt, hover_traj[:, -1], landing_x, landing_y)
    #zeros = np.tile(land_traj[:, -1:], (1, 30))

    traj = np.concatenate((take_off_traj ,wait_start_traj, hover_traj, wait_end_traj, land_traj), axis=1)
    return traj, "cutoff_pickup"

def takeoff_helper(dt, takeoff_x=0.0, takeoff_y=0.0, takeoff_z=1.0):

    steps_takeoff = 4 * 30  # 1 second of takeoff at 30 Hz

    # Takeoff phase
    time_space_takeoff = np.linspace(0, steps_takeoff * dt, steps_takeoff)
    x_traj = np.ones_like(time_space_takeoff) * takeoff_x
    y_traj = np.ones_like(time_space_takeoff) * takeoff_y
    z_traj = np.linspace(0.0, takeoff_z, steps_takeoff)


    roll_traj = np.zeros_like(time_space_takeoff)
    pitch_traj = np.zeros_like(time_space_takeoff)
    yaw_traj = np.zeros_like(time_space_takeoff)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_takeoff)
    ay_traj = np.zeros_like(time_space_takeoff)
    az_traj = np.zeros_like(time_space_takeoff)
    u1 = np.zeros_like(time_space_takeoff)
    u2 = np.zeros_like(time_space_takeoff)
    u3 = np.zeros_like(time_space_takeoff)
    u4 = np.zeros_like(time_space_takeoff)
    traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    return traj

def hover_helper(dt, hover_x=0.0, hover_y=0.0, hover_z=1.2, length=5):

    steps = length * 30  # 10 seconds of hover at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.ones_like(time_space) * hover_x
    y_traj = np.ones_like(time_space) * hover_y
    z_traj = np.ones_like(time_space) * hover_z 

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

    return hover_traj

def land_helper(dt, init_pose, landing_x=0.0, landing_y=0.0):

    steps_descend = 4 * 30
    time_space_descend = np.linspace(0, steps_descend * dt, steps_descend)

    x_traj = np.linspace(landing_x, landing_x, steps_descend)
    y_traj = np.linspace(landing_y, landing_y, steps_descend)
    z_traj = np.linspace(init_pose[2], 0.1, steps_descend)

    roll_traj = np.zeros_like(time_space_descend)
    pitch_traj = np.zeros_like(time_space_descend)
    yaw_traj = np.zeros_like(time_space_descend)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()  # Convert to quaternions
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space_descend)
    ay_traj = np.zeros_like(time_space_descend)
    az_traj = np.zeros_like(time_space_descend)
    u1 = np.zeros_like(time_space_descend)
    u2 = np.zeros_like(time_space_descend)
    u3 = np.zeros_like(time_space_descend)
    u4 = np.zeros_like(time_space_descend)
    land_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj, u1, u2, u3, u4])
    zeros = np.tile(land_traj[:, -1:], (1, steps_descend))

    traj = np.concatenate((land_traj, zeros), axis=1)

    return traj