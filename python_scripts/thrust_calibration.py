#!/usr/bin/env python3
"""
Thrust Calibration Script for Finding Optimal Thrust Values

This script helps you systematically test different thrust values to:
1. Find hover thrust (maintains altitude)
2. Find takeoff thrust (climbs steadily)
3. Find forward flight thrust (maintains altitude while pitched)

SAFETY: Always test in a safe environment with emergency stop ready!
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (in radians) to a quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class ThrustCalibrationNode(Node):
    """
    Node for systematically testing thrust values.
    Monitors altitude changes to help determine optimal thrust.
    """
    def __init__(self):
        super().__init__('thrust_calibration_node')
        
        # Publisher
        self.attitude_pub = self.create_publisher(
            AttitudeTarget, 
            '/mavros/setpoint_raw/attitude', 
            10
        )
        
        # Subscribers for monitoring
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(0.05, self.publish_attitude)  # 20 Hz
        
        # State variables
        self.current_altitude = 0.0
        self.initial_altitude = None
        self.vertical_velocity = 0.0
        
        # Test parameters (modify these)
        self.test_mode = 'hover'  # Options: 'hover', 'takeoff', 'forward'
        self.thrust_value = 0.5
        self.pitch_angle_deg = 0.0  # For forward flight testing
        
        # Logging
        self.last_log_time = self.get_clock().now()
        self.log_interval = 1.0  # Log every 1 second
        
        self.get_logger().info('=== Thrust Calibration Node Started ===')
        self.get_logger().info(f'Test Mode: {self.test_mode}')
        self.get_logger().info(f'Initial Thrust: {self.thrust_value}')
        self.get_logger().info('Monitoring altitude and velocity...')
        
    def pose_callback(self, msg):
        """Update current altitude."""
        self.current_altitude = msg.pose.position.z
        if self.initial_altitude is None:
            self.initial_altitude = self.current_altitude
            
    def velocity_callback(self, msg):
        """Update vertical velocity."""
        self.vertical_velocity = msg.twist.linear.z
        
    def publish_attitude(self):
        """Publish attitude command with current thrust."""
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Set attitude based on test mode
        roll_rad = 0.0
        pitch_rad = math.radians(self.pitch_angle_deg)
        yaw_rad = 0.0
        
        msg.orientation = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = self.thrust_value
        msg.type_mask = 7  # Ignore body rates
        
        self.attitude_pub.publish(msg)
        
        # Periodic logging
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_interval:
            self.log_status()
            self.last_log_time = now
            
    def log_status(self):
        """Log current altitude and velocity status."""
        if self.initial_altitude is not None:
            altitude_change = self.current_altitude - self.initial_altitude
            
            # Determine behavior
            if abs(self.vertical_velocity) < 0.05:
                behavior = "HOVERING ✓"
            elif self.vertical_velocity > 0.1:
                behavior = "CLIMBING ↑"
            elif self.vertical_velocity < -0.1:
                behavior = "DESCENDING ↓"
            else:
                behavior = "STABLE ~"
            
            self.get_logger().info(
                f'Thrust: {self.thrust_value:.3f} | '
                f'Alt: {self.current_altitude:.2f}m | '
                f'Δ: {altitude_change:+.2f}m | '
                f'Vz: {self.vertical_velocity:+.2f}m/s | '
                f'{behavior}'
            )
            
            # Provide recommendations
            if self.test_mode == 'hover':
                if abs(self.vertical_velocity) < 0.05:
                    self.get_logger().info('>>> GOOD HOVER THRUST! <<<')
                elif self.vertical_velocity > 0.1:
                    self.get_logger().warn('Climbing - reduce thrust!')
                else:
                    self.get_logger().warn('Descending - increase thrust!')

def test_hover_thrust(thrust_start=0.4, thrust_end=0.7, thrust_step=0.05):
    """
    Test different thrust values to find hover thrust.
    
    Usage: Run this, let drone stabilize, observe logs, adjust thrust.
    """
    rclpy.init()
    node = ThrustCalibrationNode()
    node.test_mode = 'hover'
    node.thrust_value = thrust_start
    
    print("\n" + "="*60)
    print("HOVER THRUST CALIBRATION")
    print("="*60)
    print("Instructions:")
    print("1. Arm the drone and takeoff to ~2-3m altitude")
    print("2. Switch to OFFBOARD mode")
    print("3. Observe the altitude change and vertical velocity")
    print("4. Manually adjust 'thrust_value' in the code")
    print("5. Good hover thrust: vertical velocity near 0 m/s")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_forward_flight_thrust(hover_thrust=0.5, pitch_angle=15):
    """
    Test thrust needed for forward flight at a given pitch angle.
    
    Args:
        hover_thrust: Previously determined hover thrust
        pitch_angle: Pitch angle in degrees for forward flight
    """
    rclpy.init()
    node = ThrustCalibrationNode()
    node.test_mode = 'forward'
    node.thrust_value = hover_thrust + 0.1  # Start with slightly higher
    node.pitch_angle_deg = pitch_angle
    
    print("\n" + "="*60)
    print("FORWARD FLIGHT THRUST CALIBRATION")
    print("="*60)
    print(f"Pitch Angle: {pitch_angle}°")
    print(f"Starting Thrust: {node.thrust_value}")
    print("Instructions:")
    print("1. Start with drone in hover")
    print("2. Pitch forward will be applied")
    print("3. Adjust thrust to maintain altitude")
    print(f"4. Rule of thumb: hover_thrust + (0.05 to 0.15)")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    """
    Main function - modify this to run different tests.
    """
    import sys
    
    if len(sys.argv) > 1:
        test_mode = sys.argv[1]
        if test_mode == 'hover':
            test_hover_thrust()
        elif test_mode == 'forward':
            hover_thrust = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
            pitch = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
            test_forward_flight_thrust(hover_thrust, pitch)
        else:
            print(f"Unknown mode: {test_mode}")
            print("Usage: python3 thrust_calibration.py [hover|forward] [hover_thrust] [pitch_angle]")
    else:
        # Default: test hover thrust
        test_hover_thrust()

if __name__ == '__main__':
    main()
