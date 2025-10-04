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
    """
    A class to handle trajectory visualization in RViz2
    """
    
    def __init__(self, node: Node, frame_id: str = "map"):
        """
        Initialize the trajectory visualizer
        
        Args:
            node: ROS2 node instance
            frame_id: Frame ID for visualization messages (default: "map")
        """
        self.node = node
        self.frame_id = frame_id
        
        # Create publishers for different visualization types
        self.path_publisher = node.create_publisher(Path, '/trajectory_path', 10)
        self.poses_publisher = node.create_publisher(PoseArray, '/trajectory_poses', 10)
        self.markers_publisher = node.create_publisher(MarkerArray, '/trajectory_markers', 10)
        self.pointcloud_publisher = node.create_publisher(PointCloud2, '/trajectory_pointcloud', 10)
        
        self.node.get_logger().info("TrajectoryVisualizer initialized")
    
    def _create_header(self) -> Header:
        """Create a header with current timestamp and frame_id"""
        header = Header()
        header.frame_id = self.frame_id
        now = self.node.get_clock().now()
        header.stamp = now.to_msg()
        return header
    
    def _trajectory_to_poses(self, trajectory: np.ndarray) -> list:
        """
        Convert trajectory array to list of Pose messages
        
        Args:
            trajectory: numpy array of shape (17, N) where N is number of waypoints
                       Format: [x, y, z, qw, qx, qy, qz, vx, vy, vz, ax, ay, az, u1, u2, u3, u4]
        
        Returns:
            List of Pose messages
        """
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
        """
        Publish trajectory as a Path message (shows as connected line in RViz2)
        
        Args:
            trajectory: numpy array of shape (17, N)
            color: RGBA color tuple (not used by Path, but kept for consistency)
        """
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
    
    def publish_trajectory_poses(self, trajectory: np.ndarray, subsample: int = 1):
        """
        Publish trajectory as PoseArray message (shows individual poses with arrows in RViz2)
        
        Args:
            trajectory: numpy array of shape (17, N)
            subsample: Only publish every Nth pose to reduce clutter (default: 1)
        """
        try:
            pose_array = PoseArray()
            pose_array.header = self._create_header()
            
            poses = self._trajectory_to_poses(trajectory)
            
            # Subsample poses if requested
            if subsample > 1:
                poses = poses[::subsample]
            
            pose_array.poses = poses
            
            self.poses_publisher.publish(pose_array)
            self.node.get_logger().debug(f"Published trajectory poses with {len(poses)} points")
            
        except Exception as e:
            self.node.get_logger().error(f"Error publishing trajectory poses: {e}")
    
    def publish_trajectory_markers(self, trajectory: np.ndarray, 
                                 line_color: tuple = (0.0, 1.0, 0.0, 1.0),
                                 point_color: tuple = (1.0, 0.0, 0.0, 1.0),
                                 show_velocity: bool = False,
                                 velocity_scale: float = 0.1):
        """
        Publish trajectory as MarkerArray with customizable visualization
        
        Args:
            trajectory: numpy array of shape (17, N)
            line_color: RGBA color for trajectory line
            point_color: RGBA color for waypoints
            show_velocity: Whether to show velocity vectors
            velocity_scale: Scale factor for velocity vectors
        """
        try:
            marker_array = MarkerArray()
            
            # Create trajectory line
            line_marker = Marker()
            line_marker.header = self._create_header()
            line_marker.ns = "trajectory_line"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02  # Line width
            line_marker.color = ColorRGBA(r=line_color[0], g=line_color[1], 
                                        b=line_color[2], a=line_color[3])
            
            # Add points to line
            for i in range(trajectory.shape[1]):
                point = Point()
                point.x = float(trajectory[0, i])
                point.y = float(trajectory[1, i])
                point.z = float(trajectory[2, i])
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
            
            # Create waypoint markers
            points_marker = Marker()
            points_marker.header = self._create_header()
            points_marker.ns = "trajectory_points"
            points_marker.id = 1
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.scale.x = 0.05  # Sphere diameter
            points_marker.scale.y = 0.05
            points_marker.scale.z = 0.05
            points_marker.color = ColorRGBA(r=point_color[0], g=point_color[1], 
                                          b=point_color[2], a=point_color[3])
            
            # Add waypoints (subsample to avoid clutter)
            subsample = max(1, trajectory.shape[1] // 50)  # Show at most 50 points
            for i in range(0, trajectory.shape[1], subsample):
                point = Point()
                point.x = float(trajectory[0, i])
                point.y = float(trajectory[1, i])
                point.z = float(trajectory[2, i])
                points_marker.points.append(point)
            
            marker_array.markers.append(points_marker)
            
            # Add velocity vectors if requested
            if show_velocity:
                for i in range(0, trajectory.shape[1], max(1, trajectory.shape[1] // 20)):
                    vel_marker = Marker()
                    vel_marker.header = self._create_header()
                    vel_marker.ns = "velocity_vectors"
                    vel_marker.id = i + 100
                    vel_marker.type = Marker.ARROW
                    vel_marker.action = Marker.ADD
                    
                    # Start point
                    vel_marker.points.append(Point(
                        x=float(trajectory[0, i]),
                        y=float(trajectory[1, i]),
                        z=float(trajectory[2, i])
                    ))
                    
                    # End point (start + velocity * scale)
                    vel_marker.points.append(Point(
                        x=float(trajectory[0, i] + trajectory[7, i] * velocity_scale),
                        y=float(trajectory[1, i] + trajectory[8, i] * velocity_scale),
                        z=float(trajectory[2, i] + trajectory[9, i] * velocity_scale)
                    ))
                    
                    vel_marker.scale.x = 0.01  # Arrow shaft diameter
                    vel_marker.scale.y = 0.02  # Arrow head diameter
                    vel_marker.scale.z = 0.03  # Arrow head length
                    vel_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)  # Blue
                    
                    marker_array.markers.append(vel_marker)
            
            self.markers_publisher.publish(marker_array)
            self.node.get_logger().debug(f"Published trajectory markers with {len(marker_array.markers)} markers")
            
        except Exception as e:
            self.node.get_logger().error(f"Error publishing trajectory markers: {e}")
    
    def publish_trajectory_pointcloud(self, trajectory: np.ndarray, 
                                    color_by_time: bool = True,
                                    color_by_velocity: bool = False):
        """
        Publish trajectory as PointCloud2 message (good for dense trajectories)
        
        Args:
            trajectory: numpy array of shape (17, N)
            color_by_time: Color points by time progression
            color_by_velocity: Color points by velocity magnitude
        """
        try:
            cloud_msg = PointCloud2()
            cloud_msg.header = self._create_header()
            cloud_msg.height = 1
            cloud_msg.width = trajectory.shape[1]
            cloud_msg.is_dense = True
            cloud_msg.is_bigendian = False
            
            # Define point fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            cloud_msg.fields = fields
            cloud_msg.point_step = 16  # 4 bytes * 4 fields
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            
            # Pack point data
            points_data = []
            for i in range(trajectory.shape[1]):
                x = float(trajectory[0, i])
                y = float(trajectory[1, i])
                z = float(trajectory[2, i])
                
                # Calculate color
                if color_by_time:
                    # Color from red to green based on time progression
                    t = i / (trajectory.shape[1] - 1)
                    r = int((1.0 - t) * 255)
                    g = int(t * 255)
                    b = 0
                elif color_by_velocity:
                    # Color by velocity magnitude
                    vel_mag = np.sqrt(trajectory[7, i]**2 + trajectory[8, i]**2 + trajectory[9, i]**2)
                    vel_max = np.max(np.sqrt(trajectory[7, :]**2 + trajectory[8, :]**2 + trajectory[9, :]**2))
                    if vel_max > 0:
                        vel_norm = vel_mag / vel_max
                    else:
                        vel_norm = 0
                    r = int(vel_norm * 255)
                    g = int((1.0 - vel_norm) * 255)
                    b = 0
                else:
                    r, g, b = 255, 0, 0  # Default red
                
                # Pack RGB as uint32
                rgb = (r << 16) | (g << 8) | b
                
                # Pack point data
                point_data = struct.pack('fffl', x, y, z, rgb)
                points_data.append(point_data)
            
            cloud_msg.data = b''.join(points_data)
            
            self.pointcloud_publisher.publish(cloud_msg)
            self.node.get_logger().debug(f"Published trajectory pointcloud with {trajectory.shape[1]} points")
            
        except Exception as e:
            self.node.get_logger().error(f"Error publishing trajectory pointcloud: {e}")
    
    def publish_all_visualizations(self, trajectory: np.ndarray, **kwargs):
        """
        Publish all visualization types for a trajectory
        
        Args:
            trajectory: numpy array of shape (17, N)
            **kwargs: Additional arguments passed to individual visualization functions
        """
        self.publish_trajectory_path(trajectory)
        self.publish_trajectory_poses(trajectory, subsample=kwargs.get('pose_subsample', 10))
        self.publish_trajectory_markers(trajectory, 
                                      show_velocity=kwargs.get('show_velocity', False),
                                      velocity_scale=kwargs.get('velocity_scale', 0.1))
        self.publish_trajectory_pointcloud(trajectory, 
                                         color_by_time=kwargs.get('color_by_time', True))
        
        self.node.get_logger().info("Published all trajectory visualizations")


def visualize_trajectory_standalone(trajectory: np.ndarray, 
                                  node_name: str = "trajectory_visualizer",
                                  frame_id: str = "world",
                                  duration: float = 10.0):
    """
    Standalone function to visualize a trajectory (creates its own ROS2 node)
    
    Args:
        trajectory: numpy array of shape (17, N)
        node_name: Name for the ROS2 node
        frame_id: Frame ID for visualization
        duration: How long to publish the trajectory (seconds)
    """
    rclpy.init()
    
    try:
        node = Node(node_name)
        visualizer = TrajectoryVisualizer(node, frame_id)
        
        # Create timer to publish trajectory periodically
        def timer_callback():
            visualizer.publish_all_visualizations(trajectory)
        
        timer = node.create_timer(1.0, timer_callback)  # Publish at 1 Hz
        
        node.get_logger().info(f"Publishing trajectory visualization for {duration} seconds...")
        
        # Spin for the specified duration
        start_time = node.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            current_time = node.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > duration:
                break
        
        node.get_logger().info("Trajectory visualization complete")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Example usage
    from .trajectories import power_loop_trajectory
    
    dt = 1.0 / 30.0
    trajectory = power_loop_trajectory(dt)
    
    visualize_trajectory_standalone(trajectory, duration=30.0)