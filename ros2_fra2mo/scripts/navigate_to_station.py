#!/usr/bin/env python3
"""
Navigate to Station Node

This node navigates Fra2mo to a specified station (1-6) corresponding to aircraft.
Fra2mo positions itself 1m away from the ArUco marker to enable visual servoing.

Also publishes /num_bags for Armando controller!

Usage:
    ros2 run ros2_fra2mo navigate_to_station.py --station 3 --num-bags 5

Author: Franco
Date: 2026-01-07
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
import argparse
import math
import sys


# ============================================
# WAYPOINT CONFIGURATION
# ============================================

STATION_WAYPOINTS = {
    # Aerei DISPARI - Fila DX (Y Gazebo negativo)
    1: {'x':  3.8, 'y': -6.4, 'yaw': 0.0},   # Aereo 1, ArUco 1
    3: {'x': 9.8, 'y': -6.4, 'yaw': 0.0},   # Aereo 3, ArUco 3
    5: {'x': 15.8, 'y': -6.4, 'yaw': 0.0},   # Aereo 5, ArUco 5
    
    # Aerei PARI - Fila SX (Y Gazebo positivo)
    2: {'x':  3.8, 'y':  6.4, 'yaw': 0.0},   # Aereo 2, ArUco 2
    4: {'x': 9.8, 'y':  6.4, 'yaw': 0.0},   # Aereo 4, ArUco 4
    6: {'x': 15.8, 'y':  6.4, 'yaw': 0.0},   # Aereo 6, ArUco 6
}

STATION_TO_ARUCO = {
    1: 10,
    2: 20,
    3: 30,
    4: 40,
    5: 50,
    6: 60,
}

ARUCO_TO_STATION = {v: k for k, v in STATION_TO_ARUCO.items()}


# ============================================
# HELPER FUNCTIONS
# ============================================

def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion.
    
    Args:
        yaw: Rotation angle in radians around Z axis
        
    Returns:
        List [qx, qy, qz, qw]
    """
    return [
        0.0,                    # qx
        0.0,                    # qy
        math.sin(yaw / 2.0),   # qz
        math.cos(yaw / 2.0)    # qw
    ]


def create_goal_pose(navigator, x, y, yaw):
    """
    Create a PoseStamped goal pose in map frame.
    
    Args:
        navigator: BasicNavigator instance
        x: X coordinate in map frame
        y: Y coordinate in map frame
        yaw: Orientation in radians
        
    Returns:
        PoseStamped message
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Orientation (from yaw)
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    
    return pose


# ============================================
# NAVIGATE TO STATION NODE
# ============================================

class NavigateToStationNode(Node):
    """
    Node for autonomous navigation to specified aircraft station.
    """
    
    def __init__(self, station_id, num_bags):
        super().__init__('navigate_to_station')
        
        # Validate station ID
        if station_id not in STATION_WAYPOINTS:
            self.get_logger().error(f'Invalid station ID: {station_id}. Must be 1-6.')
            raise ValueError(f'Invalid station ID: {station_id}')
        
        # Validate num_bags
        if num_bags < 1:
            self.get_logger().error(f'Invalid num_bags: {num_bags}. Must be >= 1.')
            raise ValueError(f'Invalid num_bags: {num_bags}')
        
        self.station_id = station_id
        self.num_bags = num_bags
        self.aruco_id = STATION_TO_ARUCO[station_id]
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Publishers
        self.target_station_pub = self.create_publisher(
            Int32, 
            '/target_station', 
            10
        )
        
        # Publisher per numero bagagli
        self.num_bags_pub = self.create_publisher(
            Int32,
            '/num_bags',
            10
        )
        
        self.navigation_status_pub = self.create_publisher(
            String, 
            '/navigation_status', 
            10
        )
        
        self.get_logger().info(f'Navigate to Station Node initialized')
        self.get_logger().info(f'Target: Station {station_id} (ArUco {self.aruco_id})')
        self.get_logger().info(f'Number of bags: {num_bags}')
    
    def navigate(self):
        """
        Execute navigation to target station.
        """
        # Get waypoint for this station
        waypoint = STATION_WAYPOINTS[self.station_id]
        
        self.get_logger().info(
            f'Target waypoint: x={waypoint["x"]:.2f}, '
            f'y={waypoint["y"]:.2f}, yaw={waypoint["yaw"]:.2f} rad'
        )
        
        # Create goal pose
        goal_pose = create_goal_pose(
            self.navigator,
            waypoint['x'],
            waypoint['y'],
            waypoint['yaw']
        )
        
        # Wait for Nav2 to be active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('🚙 Nav2 is active!')
        
        # Publish navigation status: NAVIGATING
        self.navigation_status_pub.publish(String(data='NAVIGATING'))
        
        # Start navigation
        self.get_logger().info(f'Starting navigation to station {self.station_id}...')
        nav_start = self.navigator.get_clock().now()
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation progress
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            
            # Log progress every 5 iterations (to avoid spam)
            if feedback and i % 5 == 0:
                current_pose = feedback.current_pose.pose
                distance_remaining = math.sqrt(
                    (waypoint['x'] - current_pose.position.x)**2 +
                    (waypoint['y'] - current_pose.position.y)**2
                )
                
                self.get_logger().info(
                    f'​​​🏃 Navigating... Distance remaining: {distance_remaining:.2f}m'
                )
            
            # Safety timeout (5 minutes)
            now = self.navigator.get_clock().now()
            if now - nav_start > Duration(seconds=300):
                self.get_logger().warn('Navigation timeout (5 min). Canceling...')
                self.navigator.cancelTask()
                break
        
        # Check result
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'⛳ Successfully reached station {self.station_id}!')
            
            # Pubblica ENTRAMBI i messaggi per Armando controller
            self.get_logger().info(
                f'Publishing: /target_station={self.station_id}, /num_bags={self.num_bags}'
            )
            
            # Pubblica station ID
            self.target_station_pub.publish(Int32(data=self.station_id))
            
            # Pubblica numero bagagli
            self.num_bags_pub.publish(Int32(data=self.num_bags))
            
            # Publish navigation status: ARRIVED
            self.navigation_status_pub.publish(String(data='ARRIVED'))
            
            self.get_logger().info('🦾​ Armando should activate now!')
            
            return True
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation was canceled!')
            self.navigation_status_pub.publish(String(data='CANCELED'))
            return False
            
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed!')
            self.navigation_status_pub.publish(String(data='FAILED'))
            return False
            
        else:
            self.get_logger().error(f'Navigation returned invalid status: {result}')
            self.navigation_status_pub.publish(String(data='UNKNOWN_ERROR'))
            return False


# ============================================
# MAIN
# ============================================

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Navigate Fra2mo to specified aircraft station'
    )
    parser.add_argument(
        '--station',
        type=int,
        required=True,
        choices=range(1, 7),
        help='Station ID (1-6 corresponding to aircraft 1-6)'
    )
    
    # Argomento num-bags
    parser.add_argument(
        '--num-bags',
        type=int,
        default=1,
        help='Number of bags to handle (default: 1)'
    )
    
    # Remove ROS args and parse
    filtered_args = []
    skip_next = False
    for arg in sys.argv[1:]:
        if skip_next:
            skip_next = False
            continue
        if arg in ['--ros-args', '-r', '--params-file', '--log-level']:
            skip_next = True
            continue
        if arg.startswith('__') or arg.startswith('/tmp/'):
            continue
        filtered_args.append(arg)
    
    args = parser.parse_args(args=filtered_args)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create node
        node = NavigateToStationNode(args.station, args.num_bags)
        
        # Execute navigation
        success = node.navigate()
        
        # Keep node alive for a moment to ensure publishers send
        if success:
            node.get_logger().info('Navigation complete. Node will shutdown in 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
        
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        return 1
    finally:
        # Cleanup
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())