#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

# Keyboard mappings from the user request
KEY_MAP = {
    87: 'w',  # W - Pitch Forward
    83: 's',  # S - Pitch Backward
    65: 'a',  # A - Roll Left
    68: 'd',  # D - Roll Right
    73: 'i',  # I - Throttle Up
    75: 'k',  # K - Throttle Down
    74: 'j',  # J - Yaw Left
    76: 'l',  # L - Yaw Right
    81: 'q',  # Q - Quit/Disarm
    69: 'e'   # E - Arm
}

# Neutral PWM values for RC channels
RC_STEP = 700

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')

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
        self.pos_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pos_callback,
            qos_profile
        )

        self.key_sub = self.create_subscription(Int32, '/keyboard/keypress', self.key_callback, 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.pos = PoseStamped()

        # Control variables
        self.roll = 0
        self.pitch = 0
        self.throttle = 0
        self.yaw = 0
        self.is_armed = False
        self.last_key_press_time = self.get_clock().now()
        self.control_timeout = 0.1  # seconds

        # Timer for publishing at 10Hz
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info("Manual Control Node initialized")
    
    def pos_callback(self, msg):
        self.pos = msg
        print(self.pos)

    def timer_callback(self):
        """
        Timer callback function for publishing manual control commands.
        """
        # Check for timeout
        elapsed = (self.get_clock().now() - self.last_key_press_time).nanoseconds / 1e9
        if elapsed > self.control_timeout:
            # Reset controls to neutral if timeout exceeded
            self.roll = 0
            self.pitch = 0
            if self.is_armed: self.throttle = 500  # Neutral throttle
            else: self.throttle = 0
            self.yaw = 0
        self.send_manual_control(x=self.pitch, y=self.roll, z=self.throttle, r=self.yaw)

    def key_callback(self, msg):
        """
        Callback function for the /keyboard/keypress topic.
        """
        key_code = msg.data
        self.last_key_press_time = self.get_clock().now()

        if key_code in KEY_MAP:
            action = KEY_MAP[key_code]

            if action == 'w': self.pitch = RC_STEP
            elif action == 's': self.pitch = -RC_STEP
            elif action == 'a': self.roll = RC_STEP
            elif action == 'd': self.roll = -RC_STEP
            elif action == 'i': self.throttle = RC_STEP + 500
            elif action == 'k': self.throttle = -RC_STEP
            elif action == 'j': self.yaw = RC_STEP
            elif action == 'l': self.yaw = -RC_STEP
            elif action == 'q':
                self.get_logger().info("Disarm command ('q') received. Shutting down.")
                # The shutdown hook in main will handle disarming
                self.destroy_node()
                rclpy.shutdown()
            elif action == 'e':
                self.arm()

    
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
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            return future.result().mode_sent
        else:
            self.get_logger().error("Set mode service call failed")
            return False
    
    def arm(self):
        """Arm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Arming service not available")
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            self.is_armed = True
            return future.result().success
        else:
            self.get_logger().error("Arming service call failed")
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
    


def main(args=None):
    rclpy.init(args=args)
    
    node = ManualControlNode()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()