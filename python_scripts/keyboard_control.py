#!/usr/bin/env python3
"""
Example of sending manual control commands to ArduPilot SITL using MAVROS with ROS2.
This script uses ROS2 to send joystick/RC-style control inputs.

Requirements:
- ROS2 (Humble/Foxy)
- MAVROS package for ROS2
- ArduPilot SITL running
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

import time

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.manual_control_pub = self.create_publisher(
            ManualControl,
            '/mavros/manual_control/send',
            10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State variables
        self.current_state = State()
        self.connected = False
        
        # Wait for MAVROS connection
        self.get_logger().info('Waiting for MAVROS connection...')
        time.sleep(2)  # Give time for initial state message
        
        # Check connection
        timeout = 10
        start = time.time()
        while not self.connected and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.connected:
            self.get_logger().info('MAVROS connected!')
        else:
            self.get_logger().warn('Connection timeout - proceeding anyway')
    
    def state_callback(self, msg):
        """Callback for state updates."""
        self.current_state = msg
        self.connected = msg.connected
    
    def arm_vehicle(self):
        """Arm the vehicle."""
        self.get_logger().info('Arming vehicle...')
        
        # Wait for service
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Vehicle armed successfully')
                return True
            else:
                self.get_logger().error('Failed to arm vehicle')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def disarm_vehicle(self):
        """Disarm the vehicle."""
        self.get_logger().info('Disarming vehicle...')
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Vehicle disarmed successfully')
                return True
            else:
                self.get_logger().error('Failed to disarm vehicle')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def set_mode(self, mode):
        """Set flight mode."""
        self.get_logger().info(f'Setting mode to {mode}...')
        
        # Wait for service
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service not available, waiting...')
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
                return True
            else:
                self.get_logger().error(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def send_manual_control(self, x, y, z, r, buttons=0):
        """
        Send manual control command.
        
        Args:
            x: pitch (-1000 to 1000, forward/backward)
            y: roll (-1000 to 1000, left/right)
            z: throttle (0 to 1000)
            r: yaw (-1000 to 1000, rotation)
            buttons: button states (bitmask)
        """
        msg = ManualControl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.x = float(x)      # pitch
        msg.y = float(y)      # roll
        msg.z = float(z)      # throttle
        msg.r = float(r)      # yaw
        msg.buttons = buttons
        
        self.manual_control_pub.publish(msg)
    
    def run_demo(self):
        """Run the manual control demonstration."""
        
        # Set to STABILIZE mode
        self.set_mode('STABILIZE')
        time.sleep(1)
        
        # Arm the vehicle
        if not self.arm_vehicle():
            self.get_logger().error('Cannot proceed without arming')
            return
        
        time.sleep(2)
        
        self.get_logger().info('\n=== Starting manual control demo ===')
        
        # Demo 1: Throttle up
        self.get_logger().info('[1/4] Sending throttle up for 5 seconds...')
        rate = self.create_rate(10)  # 10Hz
        start_time = time.time()
        while (time.time() - start_time) < 5:
            # x=0 (no pitch), y=0 (no roll), z=600 (60% throttle), r=0 (no yaw)
            self.send_manual_control(0, 0, 600, 0)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Demo 2: Neutral hover
        self.get_logger().info('[2/4] Hovering with reduced throttle for 3 seconds...')
        start_time = time.time()
        while (time.time() - start_time) < 3:
            # Neutral position with 40% throttle
            self.send_manual_control(0, 0, 400, 0)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Demo 3: Forward pitch
        self.get_logger().info('[3/4] Pitching forward for 3 seconds...')
        start_time = time.time()
        while (time.time() - start_time) < 3:
            # x=300 (forward pitch), z=500 (50% throttle)
            self.send_manual_control(300, 0, 500, 0)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Demo 4: Yaw rotation
        self.get_logger().info('[4/4] Rotating (yaw) for 3 seconds...')
        start_time = time.time()
        while (time.time() - start_time) < 3:
            # r=400 (yaw rotation), z=500 (50% throttle)
            self.send_manual_control(0, 0, 500, 400)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Return to neutral
        self.get_logger().info('Returning to neutral...')
        for _ in range(20):
            self.send_manual_control(0, 0, 400, 0)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        self.get_logger().info('\n=== Manual control demo complete ===')
        self.get_logger().info('Disarming in 2 seconds...')
        time.sleep(2)
        self.disarm_vehicle()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = ManualControlNode()
        controller.run_demo()
    except KeyboardInterrupt:
        print('\nScript interrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()