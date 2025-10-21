#!/usr/bin/env python3

############################################################################
#
# Copyright (c) 2024, All rights reserved.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
#
############################################################################

"""
A ROS2 node that publishes visual odometry data to MAVROS and sets the
EKF home position on startup.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPointStamped

class VisualOdometryNode(Node):
    """
    This node subscribes to pose information, publishes it as visual
    odometry to MAVROS, and sets the drone's home position on initialization.
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and service clients."""
        super().__init__('visual_odometry_node')
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.visual_odom_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        # Publisher for setting GPS origin
        self.set_gp_origin_pub = self.create_publisher(
            GeoPointStamped,
            '/mavros/global_position/set_gp_origin',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/simulation_pose_info',
            self.state_callback,
            qos_profile
        )

        # Publishers for pose array and path
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/drone/gt_poses'
            , 10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/drone/gt_path'
            , 10
        )

        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_msg.header.frame_id = "map"

        
        self.path_msg = Path()
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = "map"

        # Give publishers time to establish connections
        self.get_logger().info("Waiting for publishers to be ready...")
        
        import time
        time.sleep(2)

        # Timer to publish at 1Hz (required for attitude control)
        self.timer = self.create_timer(1, self.send_pose_array)

        # Set the home position using the service client
        # Using the current location of Istanbul, Turkey based on current time.
        # self.set_home_position(41.015137, 28.979530, 0.0)
        
        self.get_logger().info("Visual Odometry Node initialized")

    def state_callback(self, msg):
        """Callback to receive pose data and republish it."""
        # Assuming the desired pose is the third one in the array
        if len(msg.poses) > 2:
            self.drone_pose = msg.poses[2]
            self.send_visual_odometry(self.drone_pose)
        else:
            self.get_logger().warn("PoseArray does not contain enough poses.")

    def send_pose_array(self):
        """Sends the pose array data to MAVROS."""
        self.pose_array_msg.poses.append(self.drone_pose)
        self.pose_array_pub.publish(self.pose_array_msg)
        self.send_path()
    
    def send_path(self):
        """Sends the path data to MAVROS."""
        stamped_pose = PoseStamped()
        stamped_pose.pose = self.drone_pose
        stamped_pose.header.frame_id = "map"
        stamped_pose.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(stamped_pose)

        self.path_pub.publish(self.path_msg)

    def send_visual_odometry(self, pose):
        """Sends the visual odometry data to MAVROS."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # Or your relevant frame
        pose_msg.pose = pose
        
        self.visual_odom_pub.publish(pose_msg)

    def set_home_position(self, latitude: float, longitude: float, altitude: float):
        """
        Sets the GPS origin by publishing to /mavros/global_position/set_gp_origin.

        Args:
            latitude: The latitude of the home position.
            longitude: The longitude of the home position.
            altitude: The altitude of the home position.
        """
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.position.latitude = latitude
        msg.position.longitude = longitude
        msg.position.altitude = altitude
        
        # Publish the message
        self.set_gp_origin_pub.publish(msg)
        self.get_logger().info(f"Published GPS origin to Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")

def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    try:
        node = VisualOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

