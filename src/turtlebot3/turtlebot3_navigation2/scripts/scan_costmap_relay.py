#!/usr/bin/env python3
"""
Republish laser scans at a capped rate for Nav2 costmaps only.

SLAM can keep full-rate ``scan_normalized`` while local/global costmaps subscribe
to the relay output — fewer message_filters sync callbacks under Pi CPU load.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanCostmapRelay(Node):
    def __init__(self) -> None:
        super().__init__('scan_costmap_relay')
        self.declare_parameter('input_topic', 'scan_normalized')
        self.declare_parameter('output_topic', 'scan_costmap')
        self.declare_parameter('max_hz', 6.0)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        max_hz = float(self.get_parameter('max_hz').get_parameter_value().double_value)
        max_hz = max(max_hz, 0.5)
        self._min_period_ns = int(1e9 / max_hz)
        self._allow_next_ns = 0

        self._pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan,
            in_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Scan costmap relay: {in_topic} -> {out_topic} at <= {max_hz:.2f} Hz'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._allow_next_ns:
            return
        self._allow_next_ns = now_ns + self._min_period_ns
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanCostmapRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
