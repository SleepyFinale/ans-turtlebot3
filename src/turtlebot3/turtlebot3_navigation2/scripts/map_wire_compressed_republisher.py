#!/usr/bin/env python3
"""Publish a compressed OccupancyGrid side channel for fleet map transport.

The normal ``map`` topic stays available locally on the robot. This node adds a
lower-bandwidth ``map_wire_z`` stream that central-side tooling can bridge more
cheaply across Wi-Fi.
"""

import array
import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from rclpy_lean import lean_events

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapWireCompressedRepublisher(Node):
    def __init__(self):
        super().__init__('map_wire_compressed_republisher')
        lean_events(self)
        self.declare_parameter('input_topic', 'map')
        self.declare_parameter('output_topic', 'map_wire_z')
        self.declare_parameter('max_publish_hz', 1.0)
        self.declare_parameter('compression_level', 3)
        # The last map is sent again this often (s) while no new one comes
        # in. map_live only changes when the map does, and a robot standing
        # still after its seeding move publishes nothing new; if the one map
        # sent before the Zenoh link came up was not delivered as latched
        # history, the central PC never got a map and start_central.sh gave
        # up after 30 s. Also covers a Wi-Fi reconnect. 0 disables.
        self.declare_parameter('resend_period_sec', 3.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        max_publish_hz = float(self.get_parameter('max_publish_hz').value)
        max_publish_hz = max(max_publish_hz, 0.1)
        self._min_publish_period = 1.0 / max_publish_hz
        self._last_publish_time = 0.0

        compression_level = int(self.get_parameter('compression_level').value)
        self._compression_level = max(1, min(9, compression_level))
        self._last_out = None
        resend = float(self.get_parameter('resend_period_sec').value)
        if resend > 0.0:
            self._resend_period = resend
            self.create_timer(resend, self._resend)

        # Reuse map-style QoS so late subscribers can still receive the latest
        # full grid without waiting for a fresh SLAM publication.
        self._pub = self.create_publisher(
            UInt8MultiArray, output_topic, MAP_QOS)
        # raw=True hands over the serialized grid, which is what goes on the
        # wire anyway, so the map is never deserialized in Python.
        self.create_subscription(
            OccupancyGrid, input_topic, self._cb, MAP_QOS, raw=True)
        self.get_logger().info(
            f'Map wire: {input_topic} -> {output_topic} '
            f'(zlib OccupancyGrid, <= {max_publish_hz:.2f} Hz, '
            f'compression={self._compression_level})'
        )

    def _cb(self, raw: bytes):
        now = time.monotonic()
        if (now - self._last_publish_time) < self._min_publish_period:
            return
        out = UInt8MultiArray()
        # array('B') skips rclpy's per-element range check on uint8[].
        out.data = array.array(
            'B', zlib.compress(raw, level=self._compression_level))
        self._pub.publish(out)
        self._last_publish_time = now
        self._last_out = out

    def _resend(self):
        now = time.monotonic()
        if self._last_out is None or now - self._last_publish_time < self._resend_period:
            return
        self._pub.publish(self._last_out)
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
