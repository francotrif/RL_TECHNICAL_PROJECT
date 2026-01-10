#!/usr/bin/env python3
"""
Vision-Based Controller for Fra2mo
Uses dual independent PID controllers to position the robot in front of ArUco marker.

Target: z=0.5m distance, x=0 (centered)

(Non più usato nel progetto, ma testato) 

Author: Franco
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        
        # ============ SUBSCRIBERS & PUBLISHERS ============
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # ============ PID GAINS ============
        # Linear controller (z -> v_x)
        self.Kp_linear = 0.4
        self.Ki_linear = 0.05
        self.Kd_linear = 0.15
        
        # Angular controller (x -> omega_z)
        self.Kp_angular = 1.2
        self.Ki_angular = 0.02
        self.Kd_angular = 0.25
        
        # ============ TARGETS ============
        self.target_z = 0.5  # 50cm distance from marker
        self.target_x = 0.0  # centered
        
        # ============ LIMITS ============
        self.v_max = 0.3     # m/s
        self.omega_max = 0.6  # rad/s
        
        # ============ TOLERANCES ============
        self.tol_z = 0.05    # ±5cm
        self.tol_x = 0.03    # ±3cm
        
        # Dead zones
        self.dead_zone_z = 0.03  # ±3cm
        self.dead_zone_x = 0.02  # ±2cm
        
        # ============ PID STATE ============
        self.integral_z = 0.0
        self.integral_x = 0.0
        self.prev_error_z = 0.0
        self.prev_error_x = 0.0
        self.prev_time = None
        
        # Anti-windup limits
        self.max_integral_z = 0.5
        self.max_integral_x = 0.3
        
        # ============ CONTROL STATE ============
        self.aruco_detected = False
        self.target_reached = False
        self.stable_count = 0
        self.stable_threshold = 15  # iterations (0.75s @ 20Hz)
        
        # ============ SAFETY ============
        self.min_distance = 0.3  # Don't go closer than 30cm
        self.control_active = False
        self.start_time = None
        self.timeout = 30.0  # seconds
        
        # ============ TIMER ============
        self.control_rate = 20  # Hz
        self.timer = self.create_timer(1.0/self.control_rate, self.control_loop)
        
        # ============ LOGGING ============
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎯 Vision Controller Initialized!')
        self.get_logger().info(f'   Target: z={self.target_z}m, x={self.target_x}m')
        self.get_logger().info(f'   PID Linear:  Kp={self.Kp_linear}, Ki={self.Ki_linear}, Kd={self.Kd_linear}')
        self.get_logger().info(f'   PID Angular: Kp={self.Kp_angular}, Ki={self.Ki_angular}, Kd={self.Kd_angular}')
        self.get_logger().info('   Waiting for ArUco marker detection...')
        self.get_logger().info('=' * 70)
        
        # Storage for latest pose
        self.latest_x = None
        self.latest_y = None
        self.latest_z = None
    
    def aruco_callback(self, msg):
        """Callback when ArUco marker is detected"""
        self.aruco_detected = True
        
        # Extract position
        self.latest_x = msg.pose.position.x
        self.latest_y = msg.pose.position.y
        self.latest_z = msg.pose.position.z
        
        # Activate control if not already
        if not self.control_active and not self.target_reached:
            self.control_active = True
            self.start_time = time.time()
            self.get_logger().info('🚀 ArUco detected! Starting vision control...')
    
    def control_loop(self):
        """Main control loop - runs at fixed rate"""
        
        # Check if ArUco detected
        if not self.aruco_detected:
            return
        
        # Check if control active
        if not self.control_active:
            return
        
        # Check if target already reached
        if self.target_reached:
            return
        
        # Check timeout
        if self.start_time and (time.time() - self.start_time > self.timeout):
            self.get_logger().warn('⏱️  Timeout! Could not reach target.')
            self.stop_robot()
            self.control_active = False
            return
        
        # Check if we have fresh data
        if self.latest_z is None:
            return
        
        # Get current pose
        x = self.latest_x
        y = self.latest_y
        z = self.latest_z
        
        # Check if target reached
        if self.check_target_reached(x, z):
            self.get_logger().info('=' * 70)
            self.get_logger().info('🎯 TARGET REACHED!')
            self.get_logger().info(f'   Final position: z={z:.3f}m, x={x:.3f}m')
            self.get_logger().info('   Robot stopped.')
            self.get_logger().info('=' * 70)
            self.stop_robot()
            self.target_reached = True
            self.control_active = False
            return
        
        # Calculate control commands
        cmd = self.calculate_control(x, z)
        
        # Publish command
        self.cmd_pub.publish(cmd)
    
    def calculate_control(self, x, z):
        """Calculate control commands using dual PID"""
        
        # Get current time
        current_time = time.time()
        if self.prev_time is None:
            dt = 1.0 / self.control_rate
        else:
            dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # ========== LINEAR CONTROLLER (z -> v_x) ==========
        error_z = z - self.target_z
        
        # Dead zone
        if abs(error_z) < self.dead_zone_z:
            error_z = 0.0
        
        # PID components
        P_linear = self.Kp_linear * error_z
        
        self.integral_z += error_z * dt
        # Anti-windup
        self.integral_z = max(-self.max_integral_z, min(self.max_integral_z, self.integral_z))
        I_linear = self.Ki_linear * self.integral_z
        
        derivative_z = (error_z - self.prev_error_z) / dt
        D_linear = self.Kd_linear * derivative_z
        
        # Control output
        v_x = P_linear + I_linear + D_linear
        
        # Saturation
        v_x = max(-self.v_max, min(self.v_max, v_x))
        
        # Safety: don't get too close
        if z < self.min_distance:
            v_x = min(v_x, -0.1)  # Force backward
        
        # ========== ANGULAR CONTROLLER (x -> omega_z) ==========
        error_x = -x  # Negative because: x>0 (right) -> rotate right (negative omega)
        
        # Dead zone
        if abs(error_x) < self.dead_zone_x:
            error_x = 0.0
        
        # PID components
        P_angular = self.Kp_angular * error_x
        
        self.integral_x += error_x * dt
        # Anti-windup
        self.integral_x = max(-self.max_integral_x, min(self.max_integral_x, self.integral_x))
        I_angular = self.Ki_angular * self.integral_x
        
        derivative_x = (error_x - self.prev_error_x) / dt
        D_angular = self.Kd_angular * derivative_x
        
        # Control output
        omega_z = P_angular + I_angular + D_angular
        
        # Saturation
        omega_z = max(-self.omega_max, min(self.omega_max, omega_z))
        
        # ========== UPDATE STATE ==========
        self.prev_error_z = error_z
        self.prev_error_x = error_x
        
        # ========== CREATE COMMAND MESSAGE ==========
        cmd = Twist()
        cmd.linear.x = v_x
        cmd.angular.z = omega_z
        
        # ========== LOGGING (every 10 iterations) ==========
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        
        self.log_counter += 1
        if self.log_counter >= 10:
            distance = math.sqrt(x**2 + z**2)
            self.get_logger().info(
                f'📍 z={z:.3f}m (e={error_z:+.3f}) | '
                f'x={x:+.3f}m (e={error_x:+.3f}) | '
                f'v={v_x:+.2f} ω={omega_z:+.2f} | '
                f'd={distance:.3f}m'
            )
            self.log_counter = 0
        
        return cmd
    
    def check_target_reached(self, x, z):
        """Check if target position is reached and stable"""
        
        error_z = abs(z - self.target_z)
        error_x = abs(x - self.target_x)
        
        # Check if within tolerance
        if error_z < self.tol_z and error_x < self.tol_x:
            self.stable_count += 1
            
            # Must be stable for multiple iterations
            if self.stable_count >= self.stable_threshold:
                return True
        else:
            self.stable_count = 0
        
        return False
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()  # All zeros
        self.cmd_pub.publish(cmd)
        
        # Reset PID state
        self.integral_z = 0.0
        self.integral_x = 0.0
        self.prev_error_z = 0.0
        self.prev_error_x = 0.0

def main(args=None):
    rclpy.init(args=args)
    
    controller = VisionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Vision Controller...')
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
