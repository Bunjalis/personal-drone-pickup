### Launch the gazebo simulation 

ign gazebo quadcopter_example.sdf 


### ROS2 pose bridge 

ros2 run ros_ign_bridge parameter_bridge /world/quadcopter/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V


### ROS2 control bridge

ros2 run ros_ign_bridge parameter_bridge /X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators




### Simulation interfaces 
ros2 run simulation_communication ELRS_pass_through 
ros2 run simulation_communication motion_capture_emulator


ros2 launch simulation_communication simulation_launch.py 






### Launch the betaflight simulation

## Launch websockify

cd websockify-other/c
./websockify 127.0.0.1:6761 127.0.0.1:5761


## Launch betaflight SITL - https://betaflight.com/docs/development/SITL

./obj/main/betaflight_SITL.elf 

## Launch the ros2 betaflight communication bridge 

ros2 launch simulation_communication betaflight_simulation_launch.py 

## Launch gazebo 

ign gazebo world_large.sdf 

## Launch the controller

ros2 run controller_rates_mpc main