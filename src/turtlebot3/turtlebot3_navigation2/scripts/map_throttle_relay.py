#!/usr/bin/env python3
"""
Throttle full /map and /map_updates publications for Nav2 on bandwidth-constrained links.

Subscribes to merged map topics (typically global /map from map_merge) and republishes
at most max_map_hz, coalescing bursts. Pair with navigation_launch_multirobot remaps
to nav_map_topic=/map_relay (and map_updates remaps).
"""

from __future__ import annotations

import threading

import rclpy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
UPDATE_QOS = QoSProfile(
    depth=50,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapThrottleRelay(Node):
    def __init__(self) -> None:
        super().__init__('map_throttle_relay')
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_relay')
        self.declare_parameter('input_updates_topic', '/map_updates')
        self.declare_parameter('output_updates_topic', '/map_updates_relay')
        self.declare_parameter('max_map_hz', 1.0)
        self.declare_parameter('relay_updates', True)

        in_map = self.get_parameter('input_map_topic').get_parameter_value().string_value
        out_map = self.get_parameter('output_map_topic').get_parameter_value().string_value
        max_hz = self.get_parameter('max_map_hz').get_parameter_value().double_value
        max_hz = max(float(max_hz), 0.1)
        period = 1.0 / max_hz

        self._lock = threading.Lock()
        self._pending_grid: OccupancyGrid | None = None
        self._pending_update: OccupancyGridUpdate | None = None

        self._pub_grid = self.create_publisher(OccupancyGrid, out_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, in_map, self._on_map, MAP_QOS)

        relay_updates = self.get_parameter('relay_updates').get_parameter_value().bool_value
        if relay_updates:
            in_up = self.get_parameter('input_updates_topic').get_parameter_value().string_value
            out_up = self.get_parameter('output_updates_topic').get_parameter_value().string_value
            self._pub_up = self.create_publisher(
                OccupancyGridUpdate, out_up, UPDATE_QOS)
            self.create_subscription(
                OccupancyGridUpdate, in_up, self._on_update, UPDATE_QOS)
        else:
            self._pub_up = None

        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Map throttle relay: {in_map} -> {out_map} at <= {max_hz:.2f} Hz'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._pending_grid = msg

    def _on_update(self, msg: OccupancyGridUpdate) -> None:
        with self._lock:
            self._pending_update = msg

    def _tick(self) -> None:
        grid: OccupancyGrid | None = None
        update: OccupancyGridUpdate | None = None
        with self._lock:
            if self._pending_grid is not None:
                grid = self._pending_grid
                self._pending_grid = None
            if self._pending_update is not None:
                update = self._pending_update
                self._pending_update = None
        if grid is not None:
            self._pub_grid.publish(grid)
        if update is not None and self._pub_up is not None:
            self._pub_up.publish(update)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MapThrottleRelay | None = None
    try:
        node = MapThrottleRelay()
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
