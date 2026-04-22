#!/usr/bin/env python3
"""
Subscribe to map (namespaced), publish zlib-compressed serialized OccupancyGrid on map_wire_z.

Fleet mode side channel for lower-bandwidth map transfer to the central computer.
"""

import zlib
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import UInt8MultiArray

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapWireCompressedRepublisher(Node):
    def __init__(self):
        super().__init__('map_wire_compressed_republisher')
        self.declare_parameter('input_topic', 'map')
        self.declare_parameter('output_topic', 'map_wire_z')
        self.declare_parameter('max_publish_hz', 1.0)
        self.declare_parameter('compression_level', 3)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        max_publish_hz = float(self.get_parameter('max_publish_hz').value)
        max_publish_hz = max(max_publish_hz, 0.1)
        self._min_publish_period = 1.0 / max_publish_hz
        self._last_publish_time = 0.0

        compression_level = int(self.get_parameter('compression_level').value)
        self._compression_level = max(1, min(9, compression_level))

        self._pub = self.create_publisher(
            UInt8MultiArray, output_topic, MAP_QOS)
        self.create_subscription(
            OccupancyGrid, input_topic, self._cb, MAP_QOS)
        self.get_logger().info(
            f'Map wire: {input_topic} -> {output_topic} '
            f'(zlib OccupancyGrid, <= {max_publish_hz:.2f} Hz, '
            f'compression={self._compression_level})'
        )

    def _cb(self, msg: OccupancyGrid):
        now = time.monotonic()
        if (now - self._last_publish_time) < self._min_publish_period:
            return
        raw = serialize_message(msg)
        out = UInt8MultiArray()
        out.data = list(
            zlib.compress(raw, level=self._compression_level))
        self._pub.publish(out)
        self._last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapWireCompressedRepublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
