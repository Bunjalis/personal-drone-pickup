### Launch the gazebo simulation 

ign gazebo quadcopter_example.sdf 


### ROS2 pose bridge 

ros2 run ros_ign_bridge parameter_bridge /world/quadcopter/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V


### ROS2 control bridge

ros2 run ros_ign_bridge parameter_bridge /X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators




### Simulation interfaces 
ros2 run simulation_communication ELRS_pass_through 
ros2 run simulation_communication motion_capture_emulator
