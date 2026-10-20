#!/usr/bin/env python3
"""Record what blocks the controller when "collision ahead" will not go away.

Nav2's controller reports "collision ahead" when its footprint, projected
along the commanded motion, touches a lethal cell of the local costmap. The
costmap merges the LiDAR, stereo and low-obstacle-memory layers, so the log
line alone cannot say which sensor built the wall. When the warning has
repeated for trigger_sec, this node takes the local costmap and the latest
input of each layer, attributes the lethal cells near the robot to the source
that explains them, logs one summary line and saves the raw data under
~/.ros/stuck_snapshots/<namespace>-<time>.npz for later inspection.
"""

import math
import os
import time

import numpy as np
import rclpy
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StuckSnapshot(Node):
    def __init__(self) -> None:
        super().__init__('stuck_snapshot', start_parameter_services=False)
        self.declare_parameter('trigger_sec', 5.0)
        self.declare_parameter('cooldown_sec', 60.0)
        # Lethal cells within this distance of the robot are attributed.
        self.declare_parameter('radius_m', 0.6)
        self.declare_parameter('out_dir', '~/.ros/stuck_snapshots')
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._trigger = float(p('trigger_sec'))
        self._cooldown = float(p('cooldown_sec'))
        self._radius = float(p('radius_m'))
        self._out_dir = os.path.expanduser(str(p('out_dir')))
        ns = self.get_namespace().strip('/')
        self._ns = ns
        self._base = f'{ns}/base_footprint' if ns else 'base_footprint'
        self._controller = f'{ns}.controller_server' if ns else 'controller_server'

        self._first = None   # first warning of the current streak
        self._last = 0.0     # latest warning
        self._last_snapshot = -1e9
        self._busy = False
        self._latest = {}

        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self.create_subscription(LaserScan, 'scan', lambda m: self._latest.__setitem__('scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'stereo_blob_scan',
                                 lambda m: self._latest.__setitem__('stereo', m), 10)
        self.create_subscription(PointCloud2, 'low_obstacle_points',
                                 lambda m: self._latest.__setitem__('memory', m), 5)
        self._costmap = self.create_client(GetCostmap, 'local_costmap/get_costmap')
        self.create_timer(1.0, self._tick)

    def _on_log(self, msg: Log) -> None:
        if msg.name != self._controller or 'collision ahead' not in msg.msg:
            return
        now = time.monotonic()
        if self._first is None or now - self._last > 3.0:
            self._first = now
        self._last = now

    def _tick(self) -> None:
        now = time.monotonic()
        if self._busy or self._first is None or now - self._last > 3.0:
            return
        if now - self._first < self._trigger or now - self._last_snapshot < self._cooldown:
            return
        if not self._costmap.service_is_ready():
            return
        self._busy = True
        self._last_snapshot = now
        self._costmap.call_async(GetCostmap.Request()).add_done_callback(self._on_costmap)

    # --- attribution --------------------------------------------------------
    def _points(self, key, frame):
        """Latest input of a layer as Nx2 points in frame, or an empty array."""
        msg = self._latest.get(key)
        if msg is None:
            return np.zeros((0, 2))
        try:
            tf = self._tf.lookup_transform(frame, msg.header.frame_id, Time())
        except TransformException:
            return np.zeros((0, 2))
        if isinstance(msg, LaserScan):
            r = np.asarray(msg.ranges, dtype=float)
            a = msg.angle_min + np.arange(r.size) * msg.angle_increment
            ok = np.isfinite(r) & (r > 0.0)
            xy = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)
        else:
            xy = np.asarray([[x, y] for x, y, _z in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float).reshape(-1, 2)
        t = tf.transform
        c, s = math.cos(_yaw(t.rotation)), math.sin(_yaw(t.rotation))
        return np.stack([t.translation.x + c * xy[:, 0] - s * xy[:, 1],
                         t.translation.y + s * xy[:, 0] + c * xy[:, 1]], axis=1) if len(xy) else xy

    def _on_costmap(self, future) -> None:
        try:
            self._snapshot(future.result())
        except Exception as exc:  # a diagnostic must never take the stack down
            self.get_logger().warning(f'stuck snapshot failed: {exc}')
        finally:
            self._busy = False

    def _snapshot(self, result) -> None:
        cm = result.map
        frame = cm.header.frame_id
        tf = self._tf.lookup_transform(frame, self._base, Time())
        rx, ry, ryaw = tf.transform.translation.x, tf.transform.translation.y, _yaw(tf.transform.rotation)
        res = cm.metadata.resolution
        ox, oy = cm.metadata.origin.position.x, cm.metadata.origin.position.y
        w, h = cm.metadata.size_x, cm.metadata.size_y
        grid = np.asarray(cm.data, dtype=np.uint8).reshape(h, w)
        js, is_ = np.nonzero(grid >= 254)
        cells = np.stack([ox + (is_ + 0.5) * res, oy + (js + 0.5) * res], axis=1)
        near = np.hypot(cells[:, 0] - rx, cells[:, 1] - ry) <= self._radius
        cells = cells[near]
        sources = {k: self._points(k, frame) for k in ('scan', 'stereo', 'memory')}
        tol = 0.75 * res + 0.02
        counts = {k: 0 for k in sources}
        counts['none'] = 0
        owner = []
        for cx, cy in cells:
            hit = 'none'
            for k in ('scan', 'stereo', 'memory'):
                pts = sources[k]
                if len(pts) and float(np.min(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy))) <= tol:
                    hit = k
                    break
            counts[hit] += 1
            owner.append(hit)
        # Nearest lethal cells, in the robot frame (x ahead, y left).
        c, s = math.cos(ryaw), math.sin(ryaw)
        rel = [((cx - rx) * c + (cy - ry) * s, -(cx - rx) * s + (cy - ry) * c, o)
               for (cx, cy), o in zip(cells, owner)]
        rel.sort(key=lambda v: math.hypot(v[0], v[1]))
        nearest = ', '.join(f'({x:+.2f},{y:+.2f}) {o}' for x, y, o in rel[:5])
        os.makedirs(self._out_dir, exist_ok=True)
        path = os.path.join(self._out_dir, f'{self._ns or "robot"}-{time.strftime("%Y%m%d-%H%M%S")}.npz')
        np.savez_compressed(path, grid=grid, origin=np.array([ox, oy]), resolution=res,
                            robot=np.array([rx, ry, ryaw]), frame=frame,
                            **{f'{k}_xy': v for k, v in sources.items()})
        self.get_logger().warning(
            f'collision ahead for {time.monotonic() - self._first:.0f}s: lethal cells within '
            f'{self._radius:.1f} m from LiDAR={counts["scan"]} stereo={counts["stereo"]} '
            f'memory={counts["memory"]} unexplained={counts["none"]}; nearest (x ahead, y left): '
            f'{nearest or "none"}; saved {path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StuckSnapshot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
