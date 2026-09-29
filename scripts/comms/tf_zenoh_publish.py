#!/usr/bin/env python3
"""Copy /<robot>/tf onto /<robot>/tf_zenoh without stalling SLAM.

zenoh-bridge-ros2dds subscribes with the publisher's QoS. A reliable
subscription on /<robot>/tf blocks slam_toolbox, so map->odom freezes and
Nav2 reports "Transform data too old". This node subscribes best-effort
(no backpressure) and republishes a throttled copy for the bridge.

/<robot>/tf has several publishers (SLAM, EKF, diff drive). Each message
holds only that publisher's edges. The bridge keeps one sample, so this
node merges the latest edge from every publisher into a single message.
"""

from __future__ import annotations

import copy
import sys
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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

_RELIABLE_OUT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_RELIABLE_LATCHED_OUT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TfZenohPublish(Node):
    def __init__(self, robot: str) -> None:
        super().__init__('tf_zenoh_publish')
        self._lock = threading.Lock()
        self._tf: dict = {}
        self._tf_static: dict = {}
        listen = ReentrantCallbackGroup()
        publish = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            TFMessage, f'/{robot}/tf', self._on_tf, _BEST_EFFORT_IN,
            callback_group=listen,
        )
        self.create_subscription(
            TFMessage, f'/{robot}/tf_static', self._on_tf_static, _LATCHED_IN,
            callback_group=listen,
        )
        self._pub_tf = self.create_publisher(
            TFMessage, f'/{robot}/tf_zenoh', _BEST_EFFORT_OUT,
        )
        self._pub_static = self.create_publisher(
            TFMessage, f'/{robot}/tf_static_zenoh', _BEST_EFFORT_LATCHED_OUT,
        )
        self._pub_global_tf = self.create_publisher(
            TFMessage, '/tf', _RELIABLE_OUT,
        )
        self._pub_global_static = self.create_publisher(
            TFMessage, '/tf_static', _RELIABLE_LATCHED_OUT,
        )
        self.create_timer(0.1, self._tick_tf, callback_group=publish)
        self.create_timer(1.0, self._tick_static, callback_group=publish)
        self.create_timer(5.0, self._tick_log, callback_group=publish)
        self.get_logger().info(
            f'TF zenoh publish: /{robot}/tf -> /{robot}'
            f'/tf_zenoh and /tf (best-effort in, merged, 10 Hz)')

    def _store(self, bucket: dict, msg: TFMessage) -> None:
        updates = []
        for tf in msg.transforms:
            parent = tf.header.frame_id.strip()
            child = tf.child_frame_id.strip()
            if not parent or not child:
                continue
            updates.append(((parent, child), copy.deepcopy(tf)))
        if not updates:
            return
        with self._lock:
            for key, tf in updates:
                bucket[key] = tf

    def _snapshot(self, bucket: dict):
        with self._lock:
            if not bucket:
                return None
            return list(bucket.values())

    def _publish(self, publishers, bucket: dict) -> None:
        transforms = self._snapshot(bucket)
        if not transforms:
            return
        out = TFMessage()
        out.transforms = transforms
        for publisher in publishers:
            publisher.publish(out)

    def _on_tf(self, msg: TFMessage) -> None:
        self._store(self._tf, msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._store(self._tf_static, msg)
        self._publish(
            (self._pub_static, self._pub_global_static), self._tf_static)

    def _tick_tf(self) -> None:
        self._publish((self._pub_tf, self._pub_global_tf), self._tf)

    def _tick_static(self) -> None:
        self._publish(
            (self._pub_static, self._pub_global_static), self._tf_static)

    def _edge_age(self, child_suffix: str):
        now = self.get_clock().now().nanoseconds * 1e-09
        for (parent, child), tf in self._tf.items():
            if child == child_suffix or child.endswith('/' + child_suffix):
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-09
                return (parent, child, now - stamp)
        return None

    def _tick_log(self) -> None:
        if not rclpy.ok():
            return
        if not self._tf and not self._tf_static:
            self.get_logger().warning('No TF received yet on /tf or /tf_static')
            return
        with self._lock:
            odom = self._edge_age('odom')
            base = self._edge_age('base_footprint')
            static_keys = sorted(self._tf_static)
        odom_txt = (
            'missing' if odom is None else
            f'{odom[0]}->{odom[1]} age={odom[2]:.2f}s')
        base_txt = (
            'missing' if base is None else
            f'{base[0]}->{base[1]} age={base[2]:.2f}s')
        static = ', '.join(f'{p}->{c}' for p, c in static_keys) or '(none)'
        self.get_logger().info(
            f'TF {odom_txt}; {base_txt}; static: {static}')


def main() -> int:
    if len(sys.argv) != 2 or not sys.argv[1].strip():
        print('usage: tf_zenoh_publish.py <robot>', file=sys.stderr)
        return 2
    robot = sys.argv[1].strip().strip('/')
    rclpy.init()
    node = TfZenohPublish(robot)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
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
