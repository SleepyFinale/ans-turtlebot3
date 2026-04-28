#!/usr/bin/env python3
"""ORCA-inspired shadow/advisory limiter for low-speed TB3 navigation."""

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class OrcaShadowAdvisor(Node):
    def __init__(self) -> None:
        super().__init__('orca_shadow_advisor')
        self.declare_parameter('mode', 'off')
        self.declare_parameter('max_linear_scale', 0.55)
        self.declare_parameter('stop_time_max_sec', 0.80)
        self.declare_parameter('min_predicted_separation_m', 0.26)
        self.declare_parameter('front_cone_half_angle_rad', 0.43)
        self.declare_parameter('front_cone_max_range_m', 1.2)
        self.declare_parameter('input_cmd_topic', 'cmd_vel_nav')
        self.declare_parameter('output_cmd_topic', 'cmd_vel_orca_advised')
        self.declare_parameter('scan_topic', 'scan_normalized')
        self.declare_parameter('triangulation_debug_topic', 'ultrasonic_triangulation_debug')
        self.declare_parameter('nav_path_topic', 'plan')
        self.declare_parameter('log_topic', 'orca_shadow_debug')

        self._mode = str(self.get_parameter('mode').value).strip().lower()
        if self._mode not in ('off', 'shadow', 'advisory'):
            self._mode = 'off'
        self._max_linear_scale = max(
            0.05, min(1.0, float(self.get_parameter('max_linear_scale').value))
        )
        self._stop_time_max_sec = max(
            0.0, float(self.get_parameter('stop_time_max_sec').value)
        )
        self._min_sep = max(
            0.05, float(self.get_parameter('min_predicted_separation_m').value)
        )
        self._front_half = max(
            0.1, float(self.get_parameter('front_cone_half_angle_rad').value)
        )
        self._front_max = max(
            self._min_sep, float(self.get_parameter('front_cone_max_range_m').value)
        )

        self._last_front_range: Optional[float] = None
        self._last_cluster: str = 'none'
        self._last_path_poses = 0
        self._advisory_hold_until = 0.0

        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('triangulation_debug_topic').value),
            self._on_tri_debug,
            20,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('nav_path_topic').value),
            self._on_path,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_cmd_topic').value),
            self._on_cmd,
            30,
        )
        self._advisory_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_topic').value), 30
        )
        self._log_pub = self.create_publisher(
            String, str(self.get_parameter('log_topic').value), 30
        )
        self.get_logger().info(f'orca_shadow_advisor started in mode={self._mode}')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_path(self, msg: Path) -> None:
        self._last_path_poses = len(msg.poses)

    def _on_scan(self, msg: LaserScan) -> None:
        min_front = math.inf
        ang = float(msg.angle_min)
        for r in msg.ranges:
            rv = float(r)
            if abs(ang) <= self._front_half and math.isfinite(rv):
                if msg.range_min <= rv <= min(self._front_max, msg.range_max):
                    min_front = min(min_front, rv)
            ang += float(msg.angle_increment)
        self._last_front_range = None if not math.isfinite(min_front) else float(min_front)

    def _on_tri_debug(self, msg: String) -> None:
        try:
            row = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._last_cluster = str(row.get('cluster', 'none'))

    def _compute_advisory(self, cmd: Twist) -> tuple[Twist, dict]:
        out = Twist()
        out.linear.x = float(cmd.linear.x)
        out.angular.z = float(cmd.angular.z)

        front = self._last_front_range
        min_sep = front if front is not None else math.inf
        emergency = self._last_cluster.startswith('front_emergency')
        conflict = bool(
            emergency or (math.isfinite(min_sep) and min_sep < self._min_sep + 0.18)
        )
        severity = 0.0
        reason = 'none'
        now = self._now_s()

        if conflict:
            if emergency:
                severity = 1.0
                reason = 'stop'
            elif math.isfinite(min_sep):
                severity = max(0.0, min(1.0, (self._min_sep + 0.18 - min_sep) / 0.18))
                reason = 'slowdown'
            else:
                severity = 0.5
                reason = 'yield'
            if severity >= 0.95 and self._stop_time_max_sec > 0.0:
                self._advisory_hold_until = max(
                    self._advisory_hold_until, now + self._stop_time_max_sec
                )

        in_hold = now < self._advisory_hold_until
        if in_hold:
            out.linear.x = 0.0
            out.angular.z = max(min(out.angular.z, 0.6), -0.6)
            reason = 'stop'
            severity = max(severity, 1.0)
        elif severity > 0.0:
            scale = max(self._max_linear_scale, 1.0 - severity)
            out.linear.x *= scale
            out.angular.z = max(min(out.angular.z, 0.9), -0.9)

        log = {
            'event': 'orca_advisory',
            'mode': self._mode,
            'reason': reason,
            'conflict_count': 1 if conflict else 0,
            'min_predicted_separation_m': None if not math.isfinite(min_sep) else min_sep,
            'input_v': float(cmd.linear.x),
            'input_w': float(cmd.angular.z),
            'proposed_v': float(out.linear.x),
            'proposed_w': float(out.angular.z),
            'severity': severity,
            'cluster': self._last_cluster,
            'path_poses': self._last_path_poses,
            'hold_active': in_hold,
        }
        return out, log

    def _on_cmd(self, msg: Twist) -> None:
        advised, log = self._compute_advisory(msg)
        self._log_pub.publish(String(data=json.dumps(log, separators=(',', ':'))))
        if self._mode == 'advisory':
            self._advisory_pub.publish(advised)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrcaShadowAdvisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
