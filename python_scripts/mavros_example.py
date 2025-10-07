#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import math
from std_msgs.msg import Int32

KEY_MAP = {
    87: 'w',  # W
    83: 's',  # S
    65: 'a',  # A
    68: 'd',  # D
    73: 'i',  # I
    75: 'k',  # K
    74: 'j',  # J
    76: 'l',  # L
    81: 'q',  # Q
}

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0  # takeoff altitude
        self.yaw = 0.0

        # publish continuously for offboard mode
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info("Teleop started: listening to /keyboard/keypress (Int32 keycodes)")

        # Subscribe to /keyboard/keypress topic
        self.subscription = self.create_subscription(
            Int32,
            '/keyboard/keypress',
            self.keypress_callback,
            10
        )

    def publish_pose(self):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw
        self.publisher.publish(self.pose)

    def move(self, key):
        step = 0.2
        yaw_step = 0.1

        if key == 'w':
            self.pose.pose.position.x += step
        elif key == 's':
            self.pose.pose.position.x -= step
        elif key == 'a':
            self.pose.pose.position.y += step
        elif key == 'd':
            self.pose.pose.position.y -= step
        elif key == 'i':
            self.pose.pose.position.z += step
        elif key == 'k':
            self.pose.pose.position.z -= step
        elif key == 'j':
            self.yaw += yaw_step
        elif key == 'l':
            self.yaw -= yaw_step
        elif key == 'q':
            self.get_logger().info("Exiting teleop...")
            rclpy.shutdown()

    def keypress_callback(self, msg):
        keycode = msg.data
        key = KEY_MAP.get(keycode, None)
        if key:
            self.move(key)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
