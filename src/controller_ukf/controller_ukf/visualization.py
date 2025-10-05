#!/usr/bin/env python3

"""
Trajectory Visualization Module for RViz2

This module provides functions to visualize drone trajectories in RViz2 using various message types:
- Path messages for trajectory paths
- PoseArray messages for waypoints with orientation
- MarkerArray messages for custom trajectory visualization with colors and styles
- PointCloud2 messages for high-density trajectory points

Usage:
    from .visualization import TrajectoryVisualizer
    
    # In your ROS2 node
    visualizer = TrajectoryVisualizer(node)
    visualizer.publish_trajectory_path(trajectory_data)
    visualizer.publish_trajectory_poses(trajectory_data)
    visualizer.publish_trajectory_markers(trajectory_data)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Quaternion, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time
import struct


class TrajectoryVisualizer:
    
    def __init__(self, node: Node, frame_id: str = "map"):
        self.node = node
        self.frame_id = frame_id
        
        # Create publishers for different visualization types
        self.path_publisher = node.create_publisher(Path, '/trajectory_path', 10)
        self.motion_capture_pose_publisher = node.create_publisher(PoseStamped, '/motion_capture_pose_viz', 10)
        self.orb_slam_pose_publisher = node.create_publisher(PoseStamped, '/orb_slam_pose_viz', 10)
        
        self.node.get_logger().info("TrajectoryVisualizer initialized")
    
    def _create_header(self) -> Header:
        """Create a header with current timestamp and frame_id"""
        header = Header()
        header.frame_id = self.frame_id
        now = self.node.get_clock().now()
        header.stamp = now.to_msg()
        return header
    
    def _trajectory_to_poses(self, trajectory: np.ndarray) -> list:
        poses = []
        num_points = trajectory.shape[1]
        
        for i in range(num_points):
            pose = Pose()
            
            # Position
            pose.position.x = float(trajectory[0, i])
            pose.position.y = float(trajectory[1, i])
            pose.position.z = float(trajectory[2, i])
            
            # Orientation (quaternion)
            pose.orientation.w = float(trajectory[3, i])
            pose.orientation.x = float(trajectory[4, i])
            pose.orientation.y = float(trajectory[5, i])
            pose.orientation.z = float(trajectory[6, i])
            
            poses.append(pose)
        
        return poses
    
    def publish_trajectory_path(self, trajectory: np.ndarray, color: tuple = (1.0, 0.0, 0.0, 1.0)):
        try:
            path_msg = Path()
            path_msg.header = self._create_header()
            
            poses = self._trajectory_to_poses(trajectory)
            
            for pose in poses:
                pose_stamped = PoseStamped()
                pose_stamped.header = self._create_header()
                pose_stamped.pose = pose
                path_msg.poses.append(pose_stamped)
            
            self.path_publisher.publish(path_msg)
            self.node.get_logger().debug(f"Published trajectory path with {len(poses)} points")
            
        except Exception as e:
            self.node.get_logger().error(f"Error publishing trajectory path: {e}")
    
   
   
    def publish_pose_visualization(self, pose_data: np.ndarray, pose_type: str = "motion_capture"):
        """
        Publish a single pose for visualization in RViz2
        
        Args:
            pose_data: numpy array with [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
            pose_type: either "motion_capture" or "orb_slam"
        """
        try:
            if pose_data is None or len(pose_data) < 7:
                return
                
            pose_msg = PoseStamped()
            pose_msg.header = self._create_header()
            
            # Position
            pose_msg.pose.position.x = float(pose_data[0])
            pose_msg.pose.position.y = float(pose_data[1])
            pose_msg.pose.position.z = float(pose_data[2])
            
            # Orientation (quaternion)
            pose_msg.pose.orientation.w = float(pose_data[3])
            pose_msg.pose.orientation.x = float(pose_data[4])
            pose_msg.pose.orientation.y = float(pose_data[5])
            pose_msg.pose.orientation.z = float(pose_data[6])
            
            # Select the appropriate publisher based on pose type
            if pose_type == "motion_capture":
                self.motion_capture_pose_publisher.publish(pose_msg)
            elif pose_type == "orb_slam":
                self.orb_slam_pose_publisher.publish(pose_msg)
                
        except Exception as e:
            self.node.get_logger().error(f"Error publishing pose visualization for {pose_type}: {e}")
    
    def publish_all_visualizations(self, trajectory: np.ndarray, **kwargs):
        self.publish_trajectory_path(trajectory)