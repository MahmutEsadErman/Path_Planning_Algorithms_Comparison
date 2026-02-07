#!/usr/bin/env python3
"""
Real-time Path Plotter using Matplotlib
Subscribes to drone pose and target pose topics and plots paths in real-time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from threading import Lock
from datetime import datetime


class PathPlotterNode(Node):
    def __init__(self):
        super().__init__('path_plotter_matplotlib')
        
        # Data storage
        self.real_path_x = []
        self.real_path_y = []
        self.target_path_x = []
        self.target_path_y = []
        self.return_path_x = []
        self.return_path_y = []
        
        # Starting point for offset
        self.starting_x = None
        self.starting_y = None
        self.has_starting_point = False
        
        # Returning flag
        self.returning = False
        
        # Thread safety
        self.data_lock = Lock()
        
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
            self.pose_callback,
            qos_profile
        )
        
        self.target_sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_callback,
            10
        )
        
        self.returning_sub = self.create_subscription(
            Bool,
            '/returning_status',
            self.returning_callback,
            10
        )
        
        self.get_logger().info('Path Plotter (Matplotlib) initialized')
    
    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) > 2:
            pose = msg.poses[2]
            
            with self.data_lock:
                if not self.has_starting_point:
                    self.starting_x = pose.position.x
                    self.starting_y = pose.position.y
                    self.has_starting_point = True
                    self.get_logger().info(f'Starting point set: ({self.starting_x:.2f}, {self.starting_y:.2f})')
                
                x = pose.position.x - self.starting_x
                y = -(pose.position.y - self.starting_y)  # Flip Y axis
                
                if self.returning:
                    self.return_path_x.append(x)
                    self.return_path_y.append(y)
                else:
                    self.real_path_x.append(x)
                    self.real_path_y.append(y)
    
    def target_callback(self, msg: Pose):
        with self.data_lock:
            if not self.has_starting_point:
                return
            
            x = msg.position.x - self.starting_x
            y = -(msg.position.y - self.starting_y)  # Flip Y axis
            
            self.target_path_x.append(x)
            self.target_path_y.append(y)
    
    def returning_callback(self, msg: Bool):
        with self.data_lock:
            self.returning = msg.data
            if self.returning:
                self.get_logger().info('Switching to return path mode')
    
    def get_data(self):
        """Thread-safe data access for plotting"""
        with self.data_lock:
            return (
                self.real_path_x.copy(), self.real_path_y.copy(),
                self.target_path_x.copy(), self.target_path_y.copy(),
                self.return_path_x.copy(), self.return_path_y.copy()
            )
    
    def clear_data(self):
        """Clear all path data"""
        with self.data_lock:
            self.real_path_x.clear()
            self.real_path_y.clear()
            self.target_path_x.clear()
            self.target_path_y.clear()
            self.return_path_x.clear()
            self.return_path_y.clear()
            self.has_starting_point = False
            self.starting_x = None
            self.starting_y = None


class PathPlotter:
    def __init__(self, node: PathPlotterNode):
        self.node = node
        
        # Create figure and axis (square)
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Path Plotter - Matplotlib')
        
        # Initialize plot lines (using nice color tones)
        self.real_line, = self.ax.plot([], [], color='#3498DB', linewidth=2, label='Real Path')  # Dodger Blue
        self.target_line, = self.ax.plot([], [], color='#E74C3C', linewidth=2, label='Target Path')  # Coral Red
        self.return_line, = self.ax.plot([], [], color='#F39C12', linewidth=2, label='Return Path')  # Orange
        self.start_marker, = self.ax.plot([], [], color='#3498DB', marker='^', markersize=12, linestyle='', label='Start')
        
        # Configure plot
        self.ax.set_xlabel('X (meters)', fontsize=12)
        self.ax.set_ylabel('Y (meters)', fontsize=12)
        self.ax.legend(loc='upper right')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        # No aspect ratio constraint - auto-scale to show all data
        
        # Initialize bounds
        self.margin = 5.0
    
    def update(self, frame):
        """Animation update function"""
        # Spin ROS2 node
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Get data
        real_x, real_y, target_x, target_y, return_x, return_y = self.node.get_data()
        
        # Update lines
        self.real_line.set_data(real_x, real_y)
        self.target_line.set_data(target_x, target_y)
        self.return_line.set_data(return_x, return_y)
        
        # Update start marker
        if real_x:
            self.start_marker.set_data([real_x[0]], [real_y[0]])
        
        # Update axis limits with equal scaling
        all_x = real_x + target_x + return_x
        all_y = real_y + target_y + return_y
        
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            
            # Calculate ranges
            x_range = max(x_max - x_min, 1.0)
            y_range = max(y_max - y_min, 1.0)
            
            # Use the same range for both axes (equal scaling)
            max_range = max(x_range, y_range)
            margin = max_range * 0.1
            
            # Center the data and apply equal range
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            
            self.ax.set_xlim(x_center - max_range/2 - margin, x_center + max_range/2 + margin)
            self.ax.set_ylim(y_center - max_range/2 - margin, y_center + max_range/2 + margin)
        
        return self.real_line, self.target_line, self.return_line, self.start_marker
    
    def run(self):
        """Start the animation"""
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,  # 10 Hz
            blit=False, cache_frame_data=False
        )
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    
    node = PathPlotterNode()
    plotter = PathPlotter(node)
    
    try:
        plotter.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
