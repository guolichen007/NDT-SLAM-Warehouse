#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_dir = os.path.expanduser('~/lidarslam_ws/src/AutoCraneSlam/lidar_slam2')
    
    rviz_config_file = os.path.join(pkg_dir, 'launch', 'rviz.rviz')
    config_file_path = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    lidar_slam_node = Node(
        package='lidar_slam2',
        executable='lidar_slam2_node',
        output='screen',
        arguments=[config_file_path],
        parameters=[{'use_sim_time': True}],
    )
    
    rviz_node = Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    return LaunchDescription([
        lidar_slam_node,
        rviz_node,
    ])
