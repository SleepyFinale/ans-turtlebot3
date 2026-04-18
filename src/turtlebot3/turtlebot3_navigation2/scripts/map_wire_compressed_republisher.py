#!/usr/bin/env python3
"""
Subscribe to map (namespaced), publish zlib-compressed serialized OccupancyGrid on map_wire_z.

Fleet mode side channel for lower-bandwidth map transfer to the central computer.
"""

import zlib

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
        self._pub = self.create_publisher(
            UInt8MultiArray, 'map_wire_z', MAP_QOS)
        self.create_subscription(
            OccupancyGrid, 'map', self._cb, MAP_QOS)
        self.get_logger().info('Map wire: map -> map_wire_z (zlib OccupancyGrid)')

    def _cb(self, msg: OccupancyGrid):
        raw = serialize_message(msg)
        out = UInt8MultiArray()
        out.data = list(
            zlib.compress(raw, level=zlib.Z_DEFAULT_COMPRESSION))
        self._pub.publish(out)


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
