from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS2 pose bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='pose_bridge',
            arguments=['/world/quadcopter/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']
        ),
        # ROS2 control bridge
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='control_bridge',
            arguments=['/X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']
        ),
        # ELRS Pass Through
        Node(
            package='simulation_communication',
            executable='ELRS_pass_through',
            name='elrs_pass_through'
        ),
        # Motion Capture Emulator
        Node(
            package='simulation_communication',
            executable='motion_capture_emulator',
            name='motion_capture_emulator'
        ),
    ])