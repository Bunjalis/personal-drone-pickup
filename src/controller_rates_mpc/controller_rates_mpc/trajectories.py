import numpy as np
from scipy.spatial.transform import Rotation as R




def takeoff_trajectory(dt):
    steps = 2 * 30  # 5 seconds of takeoff at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)
    x_traj = np.zeros_like(time_space)
    y_traj = np.zeros_like(time_space)
    z_traj = np.linspace(0, 1.0, steps)  # Ascend to 1 meter

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
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])
    

def land_trajectory(dt, init_pose):
    steps_move_back = 3 * 30  # 5 seconds of takeoff at 30 Hz
    steps_descend = 2 * 30  # 5 seconds of takeoff at 30 Hz
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
    z_traj_descend  = np.linspace(1.0, 0.0, steps_descend)  # Move back 1 meter
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
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])


def move_to_start_of_main_trajectory(dt, init_pose, final_pose):
    steps = 3 * 30  # 3 seconds at 30 Hz
    time_space = np.linspace(0, steps * dt, steps)

    x_traj = np.linspace(init_pose[0], final_pose[0], steps)
    y_traj = np.linspace(init_pose[1], final_pose[1], steps)
    z_traj = np.linspace(init_pose[2], final_pose[2], steps)
    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]
    vx_traj = np.gradient(x_traj, dt)
    vy_traj = np.gradient(y_traj, dt)
    vz_traj = np.gradient(z_traj, dt)
    ax_traj = np.zeros_like(time_space)
    ay_traj = np.zeros_like(time_space)
    az_traj = np.gradient(vz_traj, dt)
    return np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                     vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])





def hover_trajectory(dt):

        take_off_traj = takeoff_trajectory(dt)
        land_traj = land_trajectory(dt, take_off_traj[:, -1])
        zeros = np.zeros((take_off_traj.shape[0], 2 * 30))

        return np.concatenate((take_off_traj, land_traj,zeros), axis=1)



def circle_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], np.array([1.5, 0.0, 1.5]))


    radius = 1.5
    height = 1.5
    angular_velocity_start = 0.1  # m/s
    angular_velocity_end = 2.0  # m/s
    duration = 20  
    steps = int(duration / dt)


    time_space = np.linspace(0, duration, steps)
    angular_velocity = np.linspace(angular_velocity_start, angular_velocity_end, steps)

    theta = np.cumsum(angular_velocity * dt)  
    x_traj = radius * np.cos(theta)
    y_traj = radius * np.sin(theta)
    z_traj = np.full_like(x_traj, height)

    roll_traj = np.zeros_like(time_space)
    pitch_traj = np.zeros_like(time_space)
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    vx_traj = -radius * angular_velocity * np.sin(theta)
    vy_traj = radius * angular_velocity * np.cos(theta)
    vz_traj = np.zeros_like(vx_traj)
    ax_traj = np.zeros_like(vx_traj)
    ay_traj = np.zeros_like(vy_traj)
    az_traj = np.zeros_like(vx_traj)

    circular_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                              vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

    land_traj = land_trajectory(dt, circular_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 2 * 30))

    return np.concatenate((take_off_traj, move_to_start, circular_traj, land_traj, zeros), axis=1)






def power_loop_trajectory(dt):
    take_off_traj = takeoff_trajectory(dt)
    move_to_start = move_to_start_of_main_trajectory(dt, take_off_traj[:, -1], np.array([0.0, 0.0, 0.8]))

    # Define parameters for the power loop
    radius = 1.2
    origin = np.array([0.0, 0.0, 2.0])
    angular_speed = 3.0  # rad/s
    duration = 2 * np.pi / angular_speed  # Time to complete one loop
    steps = int(duration / dt)

    # Time array for the power loop
    time_space = np.linspace(0, duration, steps)

    # Circular trajectory in the XZ plane (loop)
    theta = angular_speed * time_space
    x_traj = radius * np.sin(theta) + origin[0]
    y_traj = np.full_like(x_traj, origin[1])  # Y remains constant
    z_traj = -radius * np.cos(theta) + origin[2]  # Start from origin - radius in Z

    # Orientation (quaternions) changes to pitch back during the loop
    roll_traj = np.zeros_like(time_space)
    pitch_traj = -theta  # Pitch back as the drone loops
    yaw_traj = np.zeros_like(time_space)
    rpy_traj = np.vstack((roll_traj, pitch_traj, yaw_traj)).T
    quaternions = R.from_euler('xyz', rpy_traj).as_quat()
    qx_traj = quaternions[:, 0]
    qy_traj = quaternions[:, 1]
    qz_traj = quaternions[:, 2]
    qw_traj = quaternions[:, 3]

    # Velocities and accelerations
    vx_traj = radius * angular_speed * np.cos(theta)
    vy_traj = np.zeros_like(vx_traj)
    vz_traj = -radius * angular_speed * np.sin(theta)
    ax_traj = -radius * angular_speed**2 * np.sin(theta)
    ay_traj = np.zeros_like(vx_traj)
    az_traj = np.zeros_like(vx_traj)

    power_loop_traj = np.array([x_traj, y_traj, z_traj, qw_traj, qx_traj, qy_traj, qz_traj,
                                vx_traj, vy_traj, vz_traj, ax_traj, ay_traj, az_traj])

    land_traj = land_trajectory(dt, power_loop_traj[:, -1])
    zeros = np.zeros((take_off_traj.shape[0], 2 * 30))

    return np.concatenate((take_off_traj, move_to_start, power_loop_traj, land_traj, zeros), axis=1)