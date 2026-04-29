#!/usr/bin/env python3
"""Publish whether the robot footprint currently sits in a lethal costmap cell.

This gives higher-level recovery logic a lightweight signal that is cheaper to
consume than polling the full costmap from every helper node.
"""

from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros


class Nav2LethalWatch(Node):
    def __init__(self):
        super().__init__('nav2_lethal_watch')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('publish_topic', 'nav2_lethal_inflation')
        self.declare_parameter('lethal_cost_threshold', 100)
        self.declare_parameter('publish_hz', 4.0)

        self.robot_name = str(self.get_parameter('robot_name').value or '')
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.costmap_topic = str(self.get_parameter('costmap_topic').value)
        self.publish_topic = str(self.get_parameter('publish_topic').value)
        self.lethal_cost_threshold = int(
            self.get_parameter('lethal_cost_threshold').value
        )
        publish_hz = float(self.get_parameter('publish_hz').value)

        # Cache the latest costmap and re-sample it on a timer; TF and costmap
        # updates are independent, so the lethal-state check should not depend on
        # either one arriving first.
        self._latest_costmap: Optional[OccupancyGrid] = None
        self._last_state: Optional[bool] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self._costmap_cb,
            10,
        )
        self.pub = self.create_publisher(Bool, self.publish_topic, 10)
        period = 1.0 / publish_hz if publish_hz > 0.0 else 0.25
        self.timer = self.create_timer(period, self._tick)

    def _costmap_cb(self, msg: OccupancyGrid):
        self._latest_costmap = msg

    def _world_to_cell(
        self, m: OccupancyGrid, x: float, y: float
    ) -> Optional[Tuple[int, int]]:
        res = float(m.info.resolution)
        if res <= 0.0:
            return None
        ox = float(m.info.origin.position.x)
        oy = float(m.info.origin.position.y)
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= m.info.width or cy >= m.info.height:
            return None
        return cx, cy

    def _publish_state(self, lethal: bool):
        # Publish every tick so central can recover quickly from packet loss.
        msg = Bool()
        msg.data = lethal
        self.pub.publish(msg)
        if self._last_state is None or self._last_state != lethal:
            self.get_logger().info(
                f'nav2_lethal_inflation={lethal} '
                f'(threshold={self.lethal_cost_threshold})'
            )
        self._last_state = lethal

    def _tick(self):
        m = self._latest_costmap
        if m is None:
            self._publish_state(False)
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self._publish_state(False)
            return

        px = float(t.transform.translation.x)
        py = float(t.transform.translation.y)
        cell = self._world_to_cell(m, px, py)
        if cell is None:
            self._publish_state(False)
            return
        cx, cy = cell
        idx = cy * m.info.width + cx
        if idx < 0 or idx >= len(m.data):
            self._publish_state(False)
            return
        cost = int(m.data[idx])
        self._publish_state(cost >= self.lethal_cost_threshold)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LethalWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        # SIGINT during tf lookup_transform (internal sleeps) ends up here.
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
            # Launch / other nodes may have already shut down this context.
            pass


if __name__ == '__main__':
    main()
