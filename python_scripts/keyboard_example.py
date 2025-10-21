#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion
import math

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_controller')
        
        # Publisher to MAVROS attitude target topic
        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )
        
        # Timer to publish at 20Hz (required for attitude control)
        self.timer = self.create_timer(0.05, self.publish_attitude)
        
        self.get_logger().info('Attitude Controller Started')
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to quaternion
        Returns: Quaternion in [w, x, y, z] order
        """
        # Convert to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        # Calculate quaternion components
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [w, x, y, z]
    
    def publish_attitude(self):
        """
        Publish attitude setpoint
        """
        msg = AttitudeTarget()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Type mask - defines which fields to IGNORE
        # Bit 0: Ignore body roll rate
        # Bit 1: Ignore body pitch rate
        # Bit 2: Ignore body yaw rate
        # Bit 3: Ignore thrust
        # Bit 6: Ignore attitude (use rates instead)
        
        # Example 1: Use attitude + thrust, ignore rates
        msg.type_mask = 0b00000111  # Ignore all rates (7)
        
        # Example 2: Use rates + thrust, ignore attitude
        # msg.type_mask = 0b10000000  # Ignore attitude (128)
        
        # Set desired attitude (quaternion)
        # Example: Roll right 20 degrees
        roll, pitch, yaw = 20.0, 0.0, 0.0  # degrees
        q = self.euler_to_quaternion(roll, pitch, yaw)
        
        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]
        
        # Set body rates (rad/s) - ignored if type_mask bit is set
        msg.body_rate.x = 0.0  # roll rate
        msg.body_rate.y = 0.0  # pitch rate
        msg.body_rate.z = 0.0  # yaw rate
        
        # Set thrust (0.0 to 1.0)
        msg.thrust = 0.5
        
        # Publish
        self.attitude_pub.publish(msg)


class AttitudeExamples(Node):
    """
    Various attitude control examples
    """
    def __init__(self):
        super().__init__('attitude_examples')
        
        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )
        
        self.timer = self.create_timer(0.05, self.publish_attitude)
        self.example_mode = 1  # Change this to switch examples
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (degrees) to quaternion"""
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [w, x, y, z]
    
    def publish_attitude(self):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        if self.example_mode == 1:
            # Example 1: Level hover
            self.get_logger().info('Example 1: Level Hover', throttle_duration_sec=2.0)
            msg.type_mask = 7  # Ignore rates
            q = self.euler_to_quaternion(0, 0, 0)
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]
            msg.thrust = 1
            
        elif self.example_mode == 2:
            # Example 2: Roll right 30 degrees
            self.get_logger().info('Example 2: Roll Right 30°', throttle_duration_sec=2.0)
            msg.type_mask = 7  # Ignore rates
            q = self.euler_to_quaternion(30, 0, 0)
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]
            msg.thrust = 0.5
            
        elif self.example_mode == 3:
            # Example 3: Pitch forward 15 degrees
            self.get_logger().info('Example 3: Pitch Forward 15°', throttle_duration_sec=2.0)
            msg.type_mask = 7  # Ignore rates
            q = self.euler_to_quaternion(0, 15, 0)
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]
            msg.thrust = 0.5
            
        elif self.example_mode == 4:
            # Example 4: Yaw rate control (spin in place)
            self.get_logger().info('Example 4: Yaw Rate 0.5 rad/s', throttle_duration_sec=2.0)
            msg.type_mask = 128  # Ignore attitude, use rates
            msg.body_rate.x = 0.0
            msg.body_rate.y = 0.0
            msg.body_rate.z = 0.5  # rad/s
            msg.thrust = 0.5
            
        elif self.example_mode == 5:
            # Example 5: Combined roll and pitch
            self.get_logger().info('Example 5: Roll 20° + Pitch 10°', throttle_duration_sec=2.0)
            msg.type_mask = 7  # Ignore rates
            q = self.euler_to_quaternion(20, 10, 0)
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]
            msg.thrust = 0.5
        
        self.attitude_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Choose which node to run:
    # Option 1: Simple attitude controller
    # node = AttitudeController()
    
    # Option 2: Run examples (uncomment below and comment above)
    node = AttitudeExamples()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()