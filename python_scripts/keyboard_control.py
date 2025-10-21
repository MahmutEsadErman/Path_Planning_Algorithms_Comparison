#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.manual_pub = self.create_publisher(
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
        
        self.current_state = State()
        
        # Timer for publishing at 10Hz
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Control values
        self.x = 0
        self.y = 0
        self.z = 0
        self.r = 0
        self.buttons = 0
        
        self.get_logger().info("Manual Control Node initialized")
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def timer_callback(self):
        """Called at 10Hz to send manual control commands"""
        self.send_manual_control(self.x, self.y, self.z, self.r, self.buttons)
    
    def wait_for_connection(self, timeout_sec=10.0):
        """Wait for FCU connection"""
        self.get_logger().info("Waiting for FCU connection...")
        
        start_time = self.get_clock().now()
        while rclpy.ok():
            if self.current_state.connected:
                self.get_logger().info("FCU connected!")
                return True
            
            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error("Connection timeout!")
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return False
    
    def set_mode(self, mode):
        """Set flight mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().mode_sent
        else:
            self.get_logger().error("Set mode service call failed")
            return False
    
    def arm(self):
        """Arm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error("Arming service call failed")
            return False
    
    def disarm(self):
        """Disarm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return False
        
        request = CommandBool.Request()
        request.value = False
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error("Disarming service call failed")
            return False
    
    def send_manual_control(self, x=0, y=0, z=0, r=0, buttons=0):
        """
        Send manual control command
        x: pitch (-1000 to 1000, forward is positive)
        y: roll (-1000 to 1000, right is positive)
        z: throttle (0 to 1000)
        r: yaw (-1000 to 1000, right is positive)
        buttons: button mask
        """
        manual_msg = ManualControl()
        manual_msg.x = float(x)
        manual_msg.y = float(y)
        manual_msg.z = float(z)
        manual_msg.r = float(r)
        manual_msg.buttons = int(buttons)
        
        self.manual_pub.publish(manual_msg)
    
    def set_control_values(self, x=0, y=0, z=0, r=0, buttons=0):
        """Set the control values that will be sent continuously"""
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.buttons = buttons
    
    def run_demo(self):
        """Run a demo sequence"""
        # Wait for connection
        if not self.wait_for_connection():
            return
        
        self.get_logger().info("Sleeping for 2 seconds...")
        rclpy.spin_once(self, timeout_sec=2.0)
        
        # Set to STABILIZE mode
        self.get_logger().info("Setting STABILIZE mode...")
        if self.set_mode("STABILIZE"):
            self.get_logger().info("Mode set to STABILIZE")
        else:
            self.get_logger().error("Failed to set mode")
            return
        
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Arm
        self.get_logger().info("Arming...")
        if self.arm():
            self.get_logger().info("Armed successfully")
        else:
            self.get_logger().error("Failed to arm")
            return
        
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Send manual control - hover with some throttle
        self.get_logger().info("Sending manual control commands (60% throttle)...")
        self.set_control_values(x=0, y=0, z=600, r=0)
        
        # Run for 5 seconds
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=5)
        
        while rclpy.ok() and (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop sending throttle
        self.get_logger().info("Stopping throttle...")
        self.set_control_values(x=0, y=0, z=0, r=0)
        
        rclpy.spin_once(self, timeout_sec=2.0)
        
        self.get_logger().info("Demo complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = ManualControlNode()
    
    try:
        # Run the demo
        node.run_demo()
        
        # Or just spin to keep sending the current control values
        # rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()