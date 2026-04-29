#!/usr/bin/env python3
"""
Publish static TF: ``map`` -> ``<robot>/map`` (identity) on this node's namespace.

Used for standalone namespaced SLAM (fleet_mode:=false): SLAM uses frame
``pinky/map`` while Nav2 goals often use ``map``. This link must live on
``/<robot>/tf_static`` so namespaced Nav2's tf listener sees it.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class Bridge(Node):
    def __init__(self):
        super().__init__('standalone_world_map_tf')
        self.declare_parameter('child_frame', '')
        child = self.get_parameter('child_frame').get_parameter_value().string_value
        child = (child or '').strip()
        if not child or '/' not in child:
            raise ValueError(
                'Parameter child_frame must be set (e.g. pinky/map)')

        self._child = child
        # tf2_ros.StaticTransformBroadcaster uses absolute "/tf_static", which
        # bypasses the node namespace — namespaced Nav2 never sees the bridge.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(TFMessage, 'tf_static', qos)
        self._send()
        # Republish periodically: some Nav2/tf2 setups miss the first static msg.
        self.create_timer(1.0, self._send)
        self.get_logger().info('Publishing static TF map -> %s (1 Hz refresh)' % child)

    def _send(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self._child
        t.transform.rotation.w = 1.0
        self._pub.publish(TFMessage(transforms=[t]))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Bridge()
    except ValueError as e:
        print(e, file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main() or 0)
