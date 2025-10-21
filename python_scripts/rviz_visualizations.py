#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPointStamped

class RvizVisualizationsNode(Node):
    """
    This node subscribes to pose information, publishes it as visual
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and service clients."""
        super().__init__('rviz_visualization_node')
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/simulation_pose_info',
            self.state_callback,
            qos_profile
        )

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/drone/gt_path'
            , 10
        )
        
        self.path_msg = Path()
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = "map"

        # Give publishers time to establish connections
        self.get_logger().info("Waiting for publishers to be ready...")
        
        import time
        time.sleep(2)

        # Timer to publish at 1Hz (required for attitude control)
        self.timer = self.create_timer(1, self.send_path)

        self.get_logger().info("Rviz Visualizations Node initialized")

    def state_callback(self, msg):
        """Callback to receive pose data and republish it."""
        # Assuming the desired pose is the third one in the array
        if len(msg.poses) > 2:
            self.drone_pose = msg.poses[2]
        else:
            self.get_logger().warn("PoseArray does not contain enough poses.")
    
    def send_path(self):
        """Sends the path data to MAVROS."""
        stamped_pose = PoseStamped()
        stamped_pose.pose = self.drone_pose
        stamped_pose.header.frame_id = "map"
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(stamped_pose)

        self.path_pub.publish(self.path_msg)

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    try:
        node = RvizVisualizationsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

