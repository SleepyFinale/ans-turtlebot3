#!/usr/bin/env python3
"""Copy ``/{robot}/tf`` and ``/{robot}/tf_static`` onto global ``/tf`` and ``/tf_static``.

``navigation_launch_multirobot.py`` remaps Nav2's ``tf`` subscription to ``/tf`` so
costmaps can use the same TF graph as a central PC (map_merge, domain bridge). On a
robot, bringup and slam_toolbox publish on namespaced ``/{robot}/tf`` only, so without
this relay (or a running central stack) transforms such as ``{robot}/odom`` never
appear on ``/tf`` and Nav2 local_costmap activation fails.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

_TF_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_TF_STATIC_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Relay(Node):
    def __init__(self) -> None:
        super().__init__('namespace_tf_to_global_tf_relay')
        self.declare_parameter('robot_namespace', '')
        ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        ns = str(ns).strip().strip('/')
        if not ns:
            self.get_logger().fatal(
                'Parameter robot_namespace is required (e.g. pinky). '
                'Relay runs without a ROS node namespace so /tf stays global.'
            )
            raise ValueError('robot_namespace')
        self._pub_tf = self.create_publisher(TFMessage, '/tf', _TF_QOS)
        self._pub_static = self.create_publisher(TFMessage, '/tf_static', _TF_STATIC_QOS)
        self.create_subscription(
            TFMessage, f'/{ns}/tf', self._on_tf, _TF_QOS)
        self.create_subscription(
            TFMessage, f'/{ns}/tf_static', self._on_tf_static, _TF_STATIC_QOS)
        self.get_logger().info(
            f'Namespace TF relay: /{ns}/tf -> /tf, /{ns}/tf_static -> /tf_static'
        )

    def _on_tf(self, msg: TFMessage) -> None:
        self._pub_tf.publish(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._pub_static.publish(msg)


def main() -> int:
    rclpy.init()
    try:
        node = Relay()
    except ValueError:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        # Launch may already have shut down the default context when SIGINT stops
        # the stack; avoid RCLError from a second rclpy.shutdown().
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
