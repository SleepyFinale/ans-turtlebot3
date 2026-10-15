#!/usr/bin/env python3
"""Copy /<robot>/tf onto /<robot>/tf_zenoh without stalling SLAM.

zenoh-bridge-ros2dds subscribes with the publisher's QoS. A reliable
subscription on /<robot>/tf blocks slam_toolbox, so map->odom freezes and
Nav2 reports "Transform data too old". This node subscribes best-effort
(no backpressure) and republishes a throttled copy for the bridge.

/<robot>/tf has several publishers (SLAM, diff drive, robot_state_publisher).
Each message holds only that publisher's edges. The bridge keeps one sample,
so every tick this node sends the edges that changed since the last tick in
one message. Unchanged edges are not resent: a repeated stamp makes tf2 on
the central PC log TF_REPEATED_DATA.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

_BEST_EFFORT_IN = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_BEST_EFFORT_OUT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_LATCHED_IN = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_BEST_EFFORT_LATCHED_OUT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _stamp_ns(tf) -> int:
    return tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec


class TfZenohPublish(Node):
    def __init__(self, robot: str) -> None:
        super().__init__('tf_zenoh_publish')
        # (parent, child) -> latest TransformStamped
        self._tf: dict = {}
        self._tf_static: dict = {}
        # (parent, child) -> stamp already sent
        self._sent: dict = {}
        # One thread: every callback is short, and rclpy's multi-threaded
        # executor costs more CPU on the Pi than the work itself.
        self.create_subscription(
            TFMessage, f'/{robot}/tf', self._on_tf, _BEST_EFFORT_IN)
        self.create_subscription(
            TFMessage, f'/{robot}/tf_static', self._on_tf_static, _LATCHED_IN)
        self._pub_tf = self.create_publisher(
            TFMessage, f'/{robot}/tf_zenoh', _BEST_EFFORT_OUT,
        )
        self._pub_static = self.create_publisher(
            TFMessage, f'/{robot}/tf_static_zenoh', _BEST_EFFORT_LATCHED_OUT,
        )
        self.create_timer(0.1, self._tick_tf)
        self.create_timer(1.0, self._tick_static)
        self.create_timer(30.0, self._tick_log)
        self.get_logger().info(
            f'TF zenoh publish: /{robot}/tf -> /{robot}/tf_zenoh '
            '(best-effort in, changed edges only, 10 Hz)')

    @staticmethod
    def _store(bucket: dict, msg: TFMessage) -> None:
        for tf in msg.transforms:
            parent = tf.header.frame_id
            child = tf.child_frame_id
            if parent and child:
                bucket[(parent, child)] = tf

    def _on_tf(self, msg: TFMessage) -> None:
        self._store(self._tf, msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._store(self._tf_static, msg)
        self._tick_static()

    def _tick_tf(self) -> None:
        fresh = [
            tf for key, tf in self._tf.items()
            if self._sent.get(key) != _stamp_ns(tf)
        ]
        for tf in fresh:
            self._sent[(tf.header.frame_id, tf.child_frame_id)] = _stamp_ns(tf)
        if fresh:
            self._pub_tf.publish(TFMessage(transforms=fresh))

    def _tick_static(self) -> None:
        # Static edges are resent whole every second so a central subscriber
        # that joins late still gets them over the best-effort route.
        transforms = list(self._tf_static.values())
        if transforms:
            self._pub_static.publish(TFMessage(transforms=transforms))

    def _edge_age(self, child_suffix: str):
        now = self.get_clock().now().nanoseconds * 1e-09
        for (parent, child), tf in self._tf.items():
            if child == child_suffix or child.endswith('/' + child_suffix):
                return (parent, child, now - _stamp_ns(tf) * 1e-09)
        return None

    def _tick_log(self) -> None:
        if not rclpy.ok():
            return
        if not self._tf and not self._tf_static:
            self.get_logger().warning('No TF received yet on /tf or /tf_static')
            return
        odom = self._edge_age('odom')
        base = self._edge_age('base_footprint')
        static_count = len(self._tf_static)
        odom_txt = (
            'missing' if odom is None else
            f'{odom[0]}->{odom[1]} age={odom[2]:.2f}s')
        base_txt = (
            'missing' if base is None else
            f'{base[0]}->{base[1]} age={base[2]:.2f}s')
        self.get_logger().info(
            f'TF {odom_txt}; {base_txt}; static edges: {static_count}')


def main() -> int:
    if len(sys.argv) != 2 or not sys.argv[1].strip():
        print('usage: tf_zenoh_publish.py <robot>', file=sys.stderr)
        return 2
    robot = sys.argv[1].strip().strip('/')
    rclpy.init()
    node = TfZenohPublish(robot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C shuts the context down, then the next wait set fails with
        # RCLError ("context is not valid") instead of KeyboardInterrupt.
        if rclpy.ok():
            raise
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
    return 0


if __name__ == '__main__':
    sys.exit(main())
