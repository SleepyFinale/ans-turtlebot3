#!/usr/bin/env python3
"""Block forward cmd_vel when stereo reports a close obstacle or a drop.

Only obstacles in the robot's own path count (path_obstacle_m: marks two
frames agree on, within path_half_width_m of the centre line). The gate waits for a second consecutive frame so one bad match does not stop
the robot. It never stops while the depth node is uncalibrated. Turning and
reversing from the controller still pass through, so a robot stopped facing a
remembered obstacle can still turn away from it.
"""

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class StereoCmdVelEnforcer(Node):
    def __init__(self) -> None:
        super().__init__('stereo_cmd_vel_enforcer')
        self.declare_parameter('debug_topic', 'stereo/debug')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        # Controller output before the velocity smoother (Nav2 remaps cmd_vel to this).
        self.declare_parameter('nav_cmd_topic', 'cmd_vel_nav')
        self.declare_parameter('nav_cmd_fresh_sec', 0.3)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('hold_sec', 0.50)
        self.declare_parameter('close_obstacle_m', 0.45)
        self.declare_parameter('required_streak', 2)

        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._publish_hz = max(2.0, float(self.get_parameter('publish_hz').value))
        self._hold_sec = max(0.0, float(self.get_parameter('hold_sec').value))
        self._close_m = float(self.get_parameter('close_obstacle_m').value)
        self._required_streak = max(1, int(self.get_parameter('required_streak').value))
        self._last_hazard_time_s: Optional[float] = None
        self._last_state: Optional[bool] = None
        self._last_reason = ''
        self._nav_cmd: Optional[Twist] = None
        self._nav_cmd_time_s = 0.0
        self._nav_fresh_sec = float(self.get_parameter('nav_cmd_fresh_sec').value)
        self.create_subscription(
            Twist, str(self.get_parameter('nav_cmd_topic').value), self._on_nav_cmd, 10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter('debug_topic').value),
            self._on_debug,
            20,
        )
        self._pub = self.create_publisher(Twist, self._cmd_vel_topic, 20)
        # The override timer exists only while a stop is held: a 20 Hz timer
        # that almost always found nothing to do cost ~0.08 core in rclpy.
        self._timer = None
        self.get_logger().info(
            f'Stereo cmd_vel enforcer started: topic={self._cmd_vel_topic} '
            f'close_m={self._close_m:.2f} streak={self._required_streak}'
        )

    def _on_debug(self, msg: String) -> None:
        text = (msg.data or '').strip()
        if not text:
            return
        try:
            data = json.loads(text)
        except Exception:
            return
        if not isinstance(data, dict) or not data.get('calibrated'):
            return
        if str(data.get('status') or '') != 'ok':
            return
        try:
            obstacle_streak = int(data.get('obstacle_streak') or 0)
        except (TypeError, ValueError):
            obstacle_streak = 0
        try:
            drop_streak = int(data.get('drop_streak') or 0)
        except (TypeError, ValueError):
            drop_streak = 0
        # Confirmed marks inside the robot's path only; the nearest hit
        # anywhere in view (obstacle_range_m) includes door jambs beside it.
        obstacle_range = data.get('path_obstacle_m')
        try:
            range_m = float(obstacle_range) if obstacle_range is not None else None
        except (TypeError, ValueError):
            range_m = None

        reason = ''
        if bool(data.get('drop')) and drop_streak >= self._required_streak:
            reason = 'drop'
        elif (
            range_m is not None
            and range_m <= self._close_m
            and obstacle_streak >= self._required_streak
        ):
            reason = 'close_obstacle'
        if not reason:
            return
        self._last_reason = reason
        self._last_hazard_time_s = self.get_clock().now().nanoseconds * 1e-9
        if self._timer is None:
            self._timer = self.create_timer(1.0 / self._publish_hz, self._tick)
            self._tick()

    def _on_nav_cmd(self, msg: Twist) -> None:
        self._nav_cmd = msg
        self._nav_cmd_time_s = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        active = (
            self._last_hazard_time_s is not None
            and (now_s - self._last_hazard_time_s) <= self._hold_sec
        )
        if active:
            out = Twist()
            nav = self._nav_cmd
            if nav is not None and (now_s - self._nav_cmd_time_s) <= self._nav_fresh_sec:
                out.linear.x = min(0.0, float(nav.linear.x))
                out.angular.z = float(nav.angular.z)
            self._pub.publish(out)
        if self._last_state is None or self._last_state != active:
            self.get_logger().info(
                f'stereo_stop_active={active} reason={self._last_reason or "none"}'
            )
        self._last_state = active
        if not active and self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoCmdVelEnforcer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
