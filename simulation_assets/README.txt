### Individual Motor Control 


## Launch the ros2 betaflight communication bridge 

ros2 launch simulation_communication simulation_launch.py 

## Launch gazebo 

ign gazebo world_large.sdf 

## Launch the controller

ros2 run controller_mpc main







### Launch the Betaflight Control


## Launch the ros2 betaflight communication bridge 

ros2 launch simulation_communication betaflight_simulation_launch.py 

## Launch gazebo 

gz sim world_large.sdf -v -r


## Launch the controller

ros2 run controller_rates_mpc main