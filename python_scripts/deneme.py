#!/usr/bin/env python3
"""ROS2 live yaw PID tuner GUI.

Features:
- Subscribes to IMU yaw from /mavros/imu/data
- Plots live yaw + target yaw
- Runs a local PID loop for yaw tracking
- Optionally publishes yaw setpoint to /mavros/setpoint_raw/local
"""

from __future__ import annotations

import math
import time
import threading
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

try:
    from mavros_msgs.msg import PositionTarget
    HAVE_MAVROS_MSG = True
except ImportError:
    PositionTarget = None
    HAVE_MAVROS_MSG = False

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class YawPid:
    kp: float = 0.15
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 0.8
    output_limit: float = 0.6

    integral: float = 0.0
    prev_error: float = 0.0
    initialized: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def step(self, error: float, dt: float) -> float:
        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        if dt <= 0.0 or dt > 0.5:
            dt = 0.0

        if dt > 0.0 and self.ki != 0.0:
            self.integral += error * dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if dt > 0.0:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.output_limit, self.output_limit)


class RosYawBridge(Node):
    def __init__(self, imu_topic: str, cmd_topic: str):
        super().__init__("yaw_pid_tuner_gui")

        self._imu_topic = imu_topic
        self._cmd_topic = cmd_topic
        self._lock = threading.Lock()

        self.latest_yaw: float | None = None
        self.latest_time: float | None = None
        self.msg_count: int = 0
        self.first_msg_time: float | None = None

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )

        self.cmd_pub = None
        if HAVE_MAVROS_MSG:
            self.cmd_pub = self.create_publisher(PositionTarget, cmd_topic, 10)

    def _imu_callback(self, msg: Imu) -> None:
        yaw = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        now = time.monotonic()
        with self._lock:
            self.latest_yaw = yaw
            self.latest_time = now
            self.msg_count += 1
            if self.first_msg_time is None:
                self.first_msg_time = now

    def get_imu_state(self) -> tuple[float | None, float | None, float]:
        with self._lock:
            yaw = self.latest_yaw
            t = self.latest_time
            if self.first_msg_time is None or self.latest_time is None:
                hz = 0.0
            else:
                elapsed = max(1e-6, self.latest_time - self.first_msg_time)
                hz = self.msg_count / elapsed
        return yaw, t, hz

    def yaw_publish_like_follow_path(self, yaw: float) -> bool:
        if self.cmd_pub is None:
            return False

        # Mirror FollowPathNode::yawPublish semantics.
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw = float(yaw)

        self.cmd_pub.publish(msg)
        return True


class YawPidGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ROS2 Yaw PID Tuner")
        self.root.geometry("1220x740")

        self.vars: dict[str, tk.StringVar] = {
            "imu_topic": tk.StringVar(value="/mavros/imu/data"),
            "cmd_topic": tk.StringVar(value="/mavros/setpoint_raw/local"),
            "kp": tk.StringVar(value="0.15"),
            "ki": tk.StringVar(value="0.000"),
            "kd": tk.StringVar(value="0.000"),
            "integral_limit": tk.StringVar(value="0.80"),
            "output_limit": tk.StringVar(value="0.60"),
            "target_deg": tk.StringVar(value="30.0"),
            "settle_band_deg": tk.StringVar(value="2.0"),
            "plot_horizon": tk.StringVar(value="15.0"),
            "control_hz": tk.StringVar(value="25.0"),
        }

        self.publish_enabled = tk.BooleanVar(value=False)

        self.status_var = tk.StringVar(value="Initializing ROS...")
        self.imu_var = tk.StringVar(value="IMU: waiting...")
        self.metric_var = tk.StringVar(value="")

        self.pid = YawPid()

        self.bridge: RosYawBridge | None = None
        self.ros_ready = False

        self.control_running = False
        self.last_control_time: float | None = None

        self.target_rad = deg2rad(30.0)
        self.prev_target_rad = self.target_rad
        self.last_yaw: float | None = None
        self.last_cmd_yaw: float | None = None

        self.step_direction = 1.0
        self.within_band_since: float | None = None
        self.settling_time: float | None = None
        self.max_overshoot = 0.0
        self.abs_error_integral = 0.0
        self.control_effort = 0.0
        self.step_start_time: float | None = None

        self.time_hist: deque[float] = deque(maxlen=4000)
        self.yaw_hist_deg: deque[float] = deque(maxlen=4000)
        self.target_hist_deg: deque[float] = deque(maxlen=4000)
        self.cmd_hist_deg: deque[float] = deque(maxlen=4000)

        self.t0: float | None = None

        self._build_ui()
        self._init_ros()
        self._tick()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        fields = [
            ("IMU topic", "imu_topic"),
            ("Command topic", "cmd_topic"),
            ("Kp", "kp"),
            ("Ki", "ki"),
            ("Kd", "kd"),
            ("I limit (rad*s)", "integral_limit"),
            ("Output limit (rad)", "output_limit"),
            ("Target yaw (deg)", "target_deg"),
            ("Settle band (deg)", "settle_band_deg"),
            ("Control Hz", "control_hz"),
            ("Plot horizon (s)", "plot_horizon"),
        ]

        for row, (label, key) in enumerate(fields):
            ttk.Label(left, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(left, textvariable=self.vars[key], width=18).grid(
                row=row, column=1, sticky="ew", pady=2
            )

        row = len(fields)
        ttk.Checkbutton(
            left,
            text="Publish yaw command",
            variable=self.publish_enabled,
        ).grid(row=row, column=0, columnspan=2, sticky="w", pady=(8, 2))

        ttk.Button(left, text="Reconnect ROS", command=self._reconnect_ros).grid(
            row=row + 1, column=0, sticky="ew", pady=3
        )

        self.start_btn = ttk.Button(left, text="Start PID", command=self._toggle_control)
        self.start_btn.grid(row=row + 1, column=1, sticky="ew", pady=3)

        ttk.Button(left, text="Reset PID", command=self._reset_pid_only).grid(
            row=row + 2, column=0, sticky="ew", pady=3
        )

        ttk.Button(left, text="Set Target = Current", command=self._set_target_current).grid(
            row=row + 2, column=1, sticky="ew", pady=3
        )

        ttk.Button(left, text="Clear Plot", command=self._clear_plot).grid(
            row=row + 3, column=0, sticky="ew", pady=3
        )

        ttk.Button(left, text="Stop", command=self._stop_control).grid(
            row=row + 3, column=1, sticky="ew", pady=3)

        ttk.Label(left, textvariable=self.status_var, foreground="#1f5f8b").grid(
            row=row + 4, column=0, columnspan=2, sticky="w", pady=(10, 2)
        )
        ttk.Label(left, textvariable=self.imu_var).grid(
            row=row + 5, column=0, columnspan=2, sticky="w", pady=(2, 6)
        )
        ttk.Label(left, textvariable=self.metric_var, justify=tk.LEFT).grid(
            row=row + 6, column=0, columnspan=2, sticky="w"
        )

        self.fig = Figure(figsize=(8.6, 5.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Live IMU Yaw")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Yaw (deg)")
        self.ax.grid(True, alpha=0.35)

        (self.yaw_line,) = self.ax.plot([], [], label="IMU yaw", linewidth=2.1, color="#1f77b4")
        (self.target_line,) = self.ax.plot([], [], "--", label="Target", linewidth=2.0, color="#d62728")
        (self.cmd_line,) = self.ax.plot([], [], label="Command yaw", linewidth=1.8, color="#2ca02c", alpha=0.8)
        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _init_ros(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.bridge = RosYawBridge(
                imu_topic=self.vars["imu_topic"].get().strip() or "/mavros/imu/data",
                cmd_topic=self.vars["cmd_topic"].get().strip() or "/mavros/setpoint_raw/local",
            )
            self.ros_ready = True
            if HAVE_MAVROS_MSG:
                self.status_var.set("ROS connected")
            else:
                self.status_var.set("ROS connected (mavros_msgs yok: publish kapali)")
        except Exception as exc:
            self.ros_ready = False
            self.bridge = None
            self.status_var.set(f"ROS init failed: {exc}")

    def _destroy_ros(self) -> None:
        if self.bridge is not None:
            self.bridge.destroy_node()
            self.bridge = None
        self.ros_ready = False

    def _reconnect_ros(self) -> None:
        self._destroy_ros()
        self._init_ros()

    def _get_float(self, key: str, default: float) -> float:
        try:
            return float(self.vars[key].get().strip())
        except ValueError:
            return default

    def _sync_pid_from_gui(self) -> None:
        self.pid.kp = max(0.0, self._get_float("kp", self.pid.kp))
        self.pid.ki = max(0.0, self._get_float("ki", self.pid.ki))
        self.pid.kd = max(0.0, self._get_float("kd", self.pid.kd))
        self.pid.integral_limit = max(0.0, self._get_float("integral_limit", self.pid.integral_limit))
        self.pid.output_limit = max(0.0, self._get_float("output_limit", self.pid.output_limit))

        new_target = deg2rad(self._get_float("target_deg", rad2deg(self.target_rad)))
        new_target = wrap_angle(new_target)

        if abs(wrap_angle(new_target - self.target_rad)) > deg2rad(0.02):
            self.target_rad = new_target
            self._reset_step_metrics()

    def _toggle_control(self) -> None:
        self.control_running = not self.control_running
        self.start_btn.configure(text="Pause PID" if self.control_running else "Start PID")
        if self.control_running:
            self.last_control_time = time.monotonic()
            self._reset_step_metrics()
            self.status_var.set("PID running")
        else:
            self.status_var.set("PID paused")

    def _stop_control(self) -> None:
        self.control_running = False
        self.start_btn.configure(text="Start PID")
        self.status_var.set("Stopped")

    def _reset_pid_only(self) -> None:
        self.pid.reset()
        self.last_control_time = None
        self._reset_step_metrics()
        self.status_var.set("PID reset")

    def _set_target_current(self) -> None:
        if self.last_yaw is None:
            self.status_var.set("IMU yaw yok")
            return
        self.target_rad = wrap_angle(self.last_yaw)
        self.vars["target_deg"].set(f"{rad2deg(self.target_rad):.2f}")
        self._reset_step_metrics()

    def _clear_plot(self) -> None:
        self.time_hist.clear()
        self.yaw_hist_deg.clear()
        self.target_hist_deg.clear()
        self.cmd_hist_deg.clear()
        self.t0 = None
        self.canvas.draw_idle()

    def _reset_step_metrics(self) -> None:
        self.within_band_since = None
        self.settling_time = None
        self.max_overshoot = 0.0
        self.abs_error_integral = 0.0
        self.control_effort = 0.0
        self.step_start_time = time.monotonic()

        if self.last_yaw is None:
            self.step_direction = 1.0
        else:
            self.step_direction = 1.0 if wrap_angle(self.target_rad - self.last_yaw) >= 0.0 else -1.0

    def _update_metrics(self, error: float, control_cmd: float, dt: float) -> None:
        if dt > 0.0:
            self.abs_error_integral += abs(error) * dt
            self.control_effort += abs(control_cmd) * dt

        if self.last_yaw is not None:
            if self.step_direction >= 0.0:
                overshoot = max(0.0, self.last_yaw - self.target_rad)
            else:
                overshoot = max(0.0, self.target_rad - self.last_yaw)
            self.max_overshoot = max(self.max_overshoot, overshoot)

        settle_band = deg2rad(max(0.2, self._get_float("settle_band_deg", 2.0)))
        now = time.monotonic()

        if abs(error) <= settle_band:
            if self.within_band_since is None:
                self.within_band_since = now
            elif self.settling_time is None and (now - self.within_band_since) >= 0.6:
                if self.step_start_time is not None:
                    self.settling_time = self.within_band_since - self.step_start_time
        else:
            self.within_band_since = None

        settling_txt = "-" if self.settling_time is None else f"{self.settling_time:.2f} s"
        cmd_txt = "-" if self.last_cmd_yaw is None else f"{rad2deg(self.last_cmd_yaw):.2f} deg"

        self.metric_var.set(
            "\n".join(
                [
                    f"Error: {rad2deg(error):.2f} deg",
                    f"Cmd yaw: {cmd_txt}",
                    f"Overshoot: {rad2deg(self.max_overshoot):.2f} deg",
                    f"Settling: {settling_txt}",
                    f"IAE: {self.abs_error_integral:.3f}",
                ]
            )
        )

    def _append_plot_sample(self, t_now: float, yaw: float, target: float, cmd_yaw: float | None) -> None:
        if self.t0 is None:
            self.t0 = t_now
        t_rel = t_now - self.t0

        self.time_hist.append(t_rel)
        self.yaw_hist_deg.append(rad2deg(yaw))
        self.target_hist_deg.append(rad2deg(target))
        if cmd_yaw is None:
            self.cmd_hist_deg.append(float("nan"))
        else:
            self.cmd_hist_deg.append(rad2deg(cmd_yaw))

    def _refresh_plot(self) -> None:
        if not self.time_hist:
            return

        t = list(self.time_hist)
        yaw_deg = list(self.yaw_hist_deg)
        target_deg = list(self.target_hist_deg)
        cmd_deg = list(self.cmd_hist_deg)

        self.yaw_line.set_data(t, yaw_deg)
        self.target_line.set_data(t, target_deg)
        self.cmd_line.set_data(t, cmd_deg)

        horizon = max(4.0, self._get_float("plot_horizon", 15.0))
        t_max = t[-1]
        t_min = max(0.0, t_max - horizon)
        self.ax.set_xlim(t_min, t_min + horizon)

        y_candidates = yaw_deg + target_deg
        if any(not math.isnan(v) for v in cmd_deg):
            y_candidates += [v for v in cmd_deg if not math.isnan(v)]

        y_min = min(y_candidates) - 8.0
        y_max = max(y_candidates) + 8.0
        if (y_max - y_min) < 10.0:
            y_min -= 5.0
            y_max += 5.0
        self.ax.set_ylim(y_min, y_max)

        self.canvas.draw_idle()

    def _tick(self) -> None:
        if self.ros_ready and self.bridge is not None:
            # Process ROS callbacks from Tkinter loop.
            rclpy.spin_once(self.bridge, timeout_sec=0.0)

            yaw, sample_time, hz = self.bridge.get_imu_state()
            if yaw is not None and sample_time is not None:
                self.last_yaw = yaw
                self._sync_pid_from_gui()

                if self.control_running:
                    now = time.monotonic()
                    if self.last_control_time is None:
                        self.last_control_time = now
                    dt = now - self.last_control_time

                    desired_hz = max(1.0, self._get_float("control_hz", 25.0))
                    min_dt = 1.0 / desired_hz

                    control_cmd = 0.0
                    cmd_yaw = None
                    if dt >= min_dt:
                        error = wrap_angle(self.target_rad - yaw)
                        control_cmd = self.pid.step(error, dt)
                        cmd_yaw = wrap_angle(yaw + control_cmd)
                        self.last_cmd_yaw = cmd_yaw

                        if self.publish_enabled.get():
                            ok = self.bridge.yaw_publish_like_follow_path(cmd_yaw)
                            if not ok:
                                self.status_var.set("Publish acik ama mavros_msgs yok")

                        self.last_control_time = now
                        self._update_metrics(error, control_cmd, dt)

                    self._append_plot_sample(sample_time, yaw, self.target_rad, self.last_cmd_yaw)
                else:
                    self._append_plot_sample(sample_time, yaw, self.target_rad, None)
                    error = wrap_angle(self.target_rad - yaw)
                    self._update_metrics(error, 0.0, 0.0)

                age = time.monotonic() - sample_time
                self.imu_var.set(
                    f"IMU yaw: {rad2deg(yaw):.2f} deg | rate: {hz:.1f} Hz | age: {age*1000.0:.0f} ms"
                )
            else:
                self.imu_var.set("IMU: veri bekleniyor...")

            self._refresh_plot()

        self.root.after(30, self._tick)

    def shutdown(self) -> None:
        self._stop_control()
        self._destroy_ros()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")

    app = YawPidGui(root)

    def on_close() -> None:
        app.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
