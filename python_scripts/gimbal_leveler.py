#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64


def quaternion_to_euler_xyz(x: float, y: float, z: float, w: float):
    """Return roll, pitch, yaw in radians (XYZ / aerospace convention)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class GazeboGimbalLeveler(Node):
    def __init__(self):
        super().__init__("gazebo_gimbal_leveler")

        self.declare_parameter("pose_array_topic", "/simulation_pose_info")
        self.declare_parameter("target_pose_index", 2)
        self.declare_parameter("roll_cmd_topic", "/gimbal/cmd_roll")
        self.declare_parameter("pitch_cmd_topic", "/gimbal/cmd_pitch")
        self.declare_parameter("yaw_cmd_topic", "/gimbal/cmd_yaw")
        self.declare_parameter("down_pitch_deg", 90.0)
        self.declare_parameter("roll_comp_sign", -1.0)
        self.declare_parameter("pitch_comp_sign", -1.0)
        self.declare_parameter("control_hz", 15.0)
        self.declare_parameter("imu_lpf_alpha", 0.15)
        self.declare_parameter("deadband_deg", 0.5)
        self.declare_parameter("max_cmd_step_deg", 0.8)
        self.declare_parameter("publish_yaw", False)
        self.declare_parameter("fixed_yaw_deg", 0.0)
        self.declare_parameter("log_hz", 1.0)

        self.pose_array_topic = self.get_parameter("pose_array_topic").value
        self.target_pose_index = int(self.get_parameter("target_pose_index").value)
        self.down_pitch_rad = math.radians(self.get_parameter("down_pitch_deg").value)
        self.roll_comp_sign = float(self.get_parameter("roll_comp_sign").value)
        self.pitch_comp_sign = float(self.get_parameter("pitch_comp_sign").value)
        self.control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self.imu_lpf_alpha = min(1.0, max(0.0, float(self.get_parameter("imu_lpf_alpha").value)))
        self.deadband_rad = math.radians(float(self.get_parameter("deadband_deg").value))
        self.max_cmd_step_rad = math.radians(float(self.get_parameter("max_cmd_step_deg").value))
        self.publish_yaw = bool(self.get_parameter("publish_yaw").value)
        self.fixed_yaw_rad = math.radians(self.get_parameter("fixed_yaw_deg").value)
        self.log_interval_sec = 1.0 / max(0.1, float(self.get_parameter("log_hz").value))

        self.roll_pub = self.create_publisher(
            Float64, self.get_parameter("roll_cmd_topic").value, 10
        )
        self.pitch_pub = self.create_publisher(
            Float64, self.get_parameter("pitch_cmd_topic").value, 10
        )
        self.yaw_pub = self.create_publisher(
            Float64, self.get_parameter("yaw_cmd_topic").value, 10
        )

        self.last_log_time = self.get_clock().now()
        self.latest_roll = None
        self.latest_pitch = None
        self.filtered_roll = None
        self.filtered_pitch = None
        self.last_roll_cmd = None
        self.last_pitch_cmd = None

        self.pose_sub = self.create_subscription(
            PoseArray,
            self.pose_array_topic,
            self.pose_array_cb,
            10,
        )
        self.control_timer = self.create_timer(1.0 / self.control_hz, self.control_cb)

        self.get_logger().info(
            f"Listening {self.pose_array_topic}[{self.target_pose_index}] -> publishing gimbal cmd."
        )

    def pose_array_cb(self, msg: PoseArray):
        if self.target_pose_index < 0 or self.target_pose_index >= len(msg.poses):
            return

        q = msg.poses[self.target_pose_index].orientation
        roll, pitch, _ = quaternion_to_euler_xyz(q.x, q.y, q.z, q.w)
        self.latest_roll = roll
        self.latest_pitch = pitch

        if self.filtered_roll is None:
            self.filtered_roll = roll
            self.filtered_pitch = pitch
            return

        a = self.imu_lpf_alpha
        self.filtered_roll = a * roll + (1.0 - a) * self.filtered_roll
        self.filtered_pitch = a * pitch + (1.0 - a) * self.filtered_pitch

    def control_cb(self):
        if self.filtered_roll is None or self.filtered_pitch is None:
            return

        roll_for_cmd = 0.0 if abs(self.filtered_roll) < self.deadband_rad else self.filtered_roll
        pitch_for_cmd = 0.0 if abs(self.filtered_pitch) < self.deadband_rad else self.filtered_pitch

        # Defaults keep camera normal to ground:
        # roll_cmd = -drone_roll, pitch_cmd = down_pitch - drone_pitch
        target_roll_cmd = self.wrap_to_pi(self.roll_comp_sign * roll_for_cmd)
        target_pitch_cmd = self.wrap_to_pi(self.down_pitch_rad + self.pitch_comp_sign * pitch_for_cmd)

        roll_cmd = self.limit_step(target_roll_cmd, self.last_roll_cmd, self.max_cmd_step_rad)
        pitch_cmd = self.limit_step(target_pitch_cmd, self.last_pitch_cmd, self.max_cmd_step_rad)
        self.last_roll_cmd = roll_cmd
        self.last_pitch_cmd = pitch_cmd

        self.roll_pub.publish(Float64(data=roll_cmd))
        self.pitch_pub.publish(Float64(data=pitch_cmd))
        if self.publish_yaw:
            self.yaw_pub.publish(Float64(data=self.fixed_yaw_rad))

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def limit_step(target: float, prev: float, max_step: float) -> float:
        if prev is None:
            return target
        delta = target - prev
        if delta > max_step:
            return prev + max_step
        if delta < -max_step:
            return prev - max_step
        return target


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGimbalLeveler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
