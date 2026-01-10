#!/usr/bin/env python3
"""
Launch file per avviare tutti e 6 gli Armando controller

Ogni controller rimane in ascolto su /target_station e /num_bags.
Quando riceve trigger per la propria station, esegue sequenza per il numero di bagagli indicato.

Usage:
    ros2 launch final_project armando_controllers.launch.py

Author: Franco
Date: 2026-01-07
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Lancia 6 Armando controller, uno per ogni station.
    Tutti rimangono attivi in background, in ascolto.
    """
    
    nodes = []
    
    # Spawn controller per ogni Armando (1-6)
    for station_id in range(1, 7):
        node = Node(
            package='final_project',
            executable='armando_controller.py',
            name=f'armando_controller_{station_id}',
            output='screen',
            parameters=[
                {'station': station_id},
                {'use_sim_time': True}
            ],
            
        )
        nodes.append(node)
    
    return LaunchDescription(nodes)