# Requirements
 - Ros2 Humble
 - Ignition Gazebo Fortress (Needed for simulation)



# Building 
colcon build --symlink-install
source install/setup.bash


# Real world operation
Launch the motion capture node:
 - ros2 run drone_communication motion_capture_publisher_node 

Launch the ELRS communication node:
 - ros2 run drone_communication elrs_interface 

Launch your controller node:
 - ros2 run controller_demo main 


# Simulation operation
Launch the simulation node communication package:
 - ros2 launch simulation_communication simulation_launch.py 

Navigate to simulation assests and launch the simulation:
 - cd simulation_assets
 - ign gazebo world_large.sdf

Launch your controller node:
 - ros2 run controller_demo main 



# Drone configuration 

Standard x - frame configuration 

## Controller mapping
All control inputs are a float between [0.0,1.0]

Control 0 - Back Right - CW
Control 1 - Front Right - CCW
Control 2 - Back Left - CCW
Control 3 - Front Left - CW


Positive Roll = [-1, -1, 1, 1]
Positive Pitch = [1, -1, 1, -1]
Positive Yaw = [1, -1, -1, 1]

