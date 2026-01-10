#!/usr/bin/env python3
"""
Mission to Station Launch File

Launches navigation and ArUco detection for specified station, with a certain number of bags.


Usage:
    ros2 launch ros2_fra2mo mission_to_station.launch.py station:=3 num_bags:=5

Author: Franco
Date: 2026-01-07
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    
    station_arg = DeclareLaunchArgument(
        'station',
        default_value='1',
        description='Station ID (1-6 corresponding to aircraft 1-6 and ArUco 1-6)'
    )
    
    # Argomento numero bagagli
    num_bags_arg = DeclareLaunchArgument(
        'num_bags',
        default_value='1',
        description='Number of bags to handle at this station (default: 1)'
    )
    
    station = LaunchConfiguration('station')
    num_bags = LaunchConfiguration('num_bags')
    
    # ============================================
    # ARUCO DETECTION
    # ============================================
    
    # Include ArUco detection launch file
    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_fra2mo'),
                'launch',
                'aruco_detection.launch.py'
            ])
        ),
        launch_arguments={
            'marker_id': station  # Direct mapping: Station 3 → ArUco 3
        }.items()
    )
    
    # ============================================
    # NAVIGATION TO STATION
    # ============================================
    
    navigate_to_station_node = Node(
        package='ros2_fra2mo',
        executable='navigate_to_station.py',
        name='navigate_to_station',
        output='screen',
        arguments=['--station', station, '--num-bags', num_bags],  # numero bagagli num_bags
        parameters=[{'use_sim_time': True}]
    )
    
    # ============================================
    # LAUNCH DESCRIPTION
    # ============================================
    
    return LaunchDescription([
        station_arg,
        num_bags_arg,  
        aruco_detection_launch,
        navigate_to_station_node,
    ])