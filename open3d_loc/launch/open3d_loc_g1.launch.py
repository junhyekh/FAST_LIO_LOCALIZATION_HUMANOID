#!/usr/bin/env python3
"""
ROS2 Launch file for FAST-LIO Global Localization System

This launch file sets up the complete localization system for humanoid robots:
1. Static transform publishers for robot frame hierarchy
2. Global localization node with Open3D point cloud registration
3. Configurable parameters for different environments

The system provides robust localization using:
- FAST-LIO for LiDAR-inertial odometry
- Open3D for point cloud registration against pre-built maps
- Kalman filtering for pose smoothing
- Multi-resolution registration (coarse + fine) for efficiency

Author: FAST-LIO Localization Team
"""

import os.path
from re import T
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the global localization system.
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # Get package paths for configuration files
    package_path = get_package_share_directory('open3d_loc')
    default_config_path = os.path.join(package_path, 'config', 'loc_param_g1.yaml')
    default_map_path = os.path.join(package_path, '..', 'data', 'map.ply')
    default_rviz_config_path = os.path.join(package_path, 'rviz_cfg', 'loc_map_cur_humble.rviz')

    # ===== Launch Arguments =====
    
    # Configuration for simulation vs real-time operation
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true, real-time clock if false'
    )
    
    # Configuration file path
    config_path = LaunchConfiguration('config_path')
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Path to YAML configuration file containing localization parameters'
    )
    
    # Map file path
    map_path = LaunchConfiguration('map_path')
    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path', default_value=default_map_path,
        description='Path to the point cloud map file (.ply format) for localization'
    )

    # RViz2 toggle
    rviz = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 visualization if true'
    )

    # RViz2 config
    rviz_config = LaunchConfiguration('rviz_config')
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config_path,
        description='Path to RViz2 config file (.rviz)'
    )

    # ===== Static Transform Publishers =====
    # These establish the robot's frame hierarchy and coordinate system
    
    # Transform from camera_init to odom frame
    # This is typically used to establish the initial coordinate system
    camera_init2odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    # Transform from base_link to imu_link frame
    # This represents the IMU position relative to the robot's base
    imulink2baselink_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    # Transform from motion_link to base_link frame
    # This represents the robot's motion center relative to its base
    base_center_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'motion_link', 'base_link']
    )

    # ===== RViz2 Visualization =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    # ===== Global Localization Node =====
    # This is the main localization system that performs point cloud registration
    
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',  # Show output in terminal for debugging
        parameters=[
            # Load configuration from YAML file
            PathJoinSubstitution([config_path]),
            
            # System configuration
            {'use_sim_time': use_sim_time},
            {'path_map': map_path},
            
            # Point cloud processing parameters
            {'pcd_queue_maxsize': 10},        # Maximum number of scans in processing queue
            {'voxelsize_coarse': 0.15},       # Voxel size for coarse registration (meters)
            {'voxelsize_fine': 0.1},          # Voxel size for fine registration (meters)
            
            # Registration quality thresholds
            {'threshold_fitness': 0.9},       # Minimum fitness score for regular updates
            {'threshold_fitness_init': 0.9},  # Minimum fitness score for initialization
            
            # System timing
            {'loc_frequence': 10.0},           # Localization frequency (Hz)
            
            # Data processing options
            {'save_scan': False},             # Whether to save scan data to files
            {'hidden_removal': False},        # Whether to remove hidden points
            
            # Point cloud size limits for performance
            {'maxpoints_source': 80000},      # Maximum points in source (scan) for registration
            {'maxpoints_target': 400000},     # Maximum points in target (map) for registration
            
            # Kalman filtering configuration
            {'filter_odom2map': True},       # Enable/disable Kalman filtering
            {'kalman_processVar2': 0.001},    # Process noise variance for Kalman filter
            {'kalman_estimatedMeasVar2': 0.02}, # Measurement noise variance for Kalman filter
            
            # Quality control
            {'confidence_loc_th': 0.7},       # Confidence threshold for localization
            {'dis_updatemap': 3.5}            # Distance threshold for map updates (meters)
        ]
    )

    # ===== Launch Description Assembly =====
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_map_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)

    # Optionally launch RViz2
    ld.add_action(rviz_node)

    # Add static transform publishers
    ld.add_action(camera_init2odom_node)
    ld.add_action(imulink2baselink_node)
    ld.add_action(base_center_broadcaster_node)
    
    # Add the main localization node
    ld.add_action(global_localization_node)

    return ld
