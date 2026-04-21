#!/usr/bin/env python3
"""Publish Bool when RegulatedPurePursuitController reports collision ahead (via rosout)."""

from typing import Optional

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class Nav2ControllerCollisionWatch(Node):
    """Debounces controller_server \"collision ahead\" logs into a steady Bool topic."""

    def __init__(self):
        super().__init__('nav2_controller_collision_watch')

        self.declare_parameter('rosout_topic', '/rosout')
        self.declare_parameter('node_name_substring', 'controller_server')
        self.declare_parameter('message_substring', 'collision ahead')
        self.declare_parameter('hold_sec', 0.75)
        self.declare_parameter('publish_hz', 4.0)
        self.declare_parameter('publish_topic', 'nav2_collision_ahead')

        self._rosout_topic = str(self.get_parameter('rosout_topic').value)
        self._node_sub = str(
            self.get_parameter('node_name_substring').value
        ).lower()
        self._msg_sub = str(
            self.get_parameter('message_substring').value
        ).lower()
        self._hold_sec = float(self.get_parameter('hold_sec').value)
        self._publish_hz = float(self.get_parameter('publish_hz').value)
        self._publish_topic = str(self.get_parameter('publish_topic').value)

        self._last_match_time: Optional[float] = None
        self._last_pub: Optional[bool] = None

        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Log,
            self._rosout_topic,
            self._rosout_cb,
            qos,
        )
        self._pub = self.create_publisher(Bool, self._publish_topic, 10)
        period = 1.0 / self._publish_hz if self._publish_hz > 0.0 else 0.25
        self.create_timer(period, self._tick)

    def _rosout_cb(self, msg: Log):
        name = (msg.name or '').lower()
        text = (msg.msg or '').lower()
        if self._node_sub not in name:
            return
        if self._msg_sub not in text:
            return
        self._last_match_time = self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        active = False
        if self._last_match_time is not None and self._hold_sec >= 0.0:
            active = (now - self._last_match_time) <= self._hold_sec
        elif self._last_match_time is not None:
            active = True

        m = Bool()
        m.data = bool(active)
        self._pub.publish(m)
        if self._last_pub is None or self._last_pub != active:
            self.get_logger().info(
                f'{self._publish_topic}={active} '
                f'(hold_sec={self._hold_sec})'
            )
        self._last_pub = active


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ControllerCollisionWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
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
