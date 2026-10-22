#!/usr/bin/env python3
"""Remember low obstacles near the robot after they leave the cameras' view.

Sources, all recorded as cells in the SLAM map frame:

- stereo_blob_scan: confirmed stereo marks. Stereo only sees the near face of
  a low object, so each mark is extended extrude_m along the beam; turning
  around a box then treats its unseen back half as occupied.
- bump_points: contact points from stuck_detector (a stall or slip while
  driving). Works on robots without cameras.

stereo_clear_scan removes remembered cells along beams where the cameras
matched floor, so a moved object or a false bump is forgotten once the robot
looks at the spot again. Cells also expire (mark_ttl_sec / bump_ttl_sec).

The memory goes to Nav2 as a point cloud (map frame, one point per cell,
low_obstacle_points) that a dedicated obstacle layer in each costmap marks
and stereo_clear_scan clears. Nav2's recoveries clear costmap layers
entirely; the cloud is republished every second, so the memory is back in
the costmap right after a clear. Being an ordinary layer, it is inflated
like every other obstacle: lethal only on the remembered cells, inscribed
(253) within the robot radius. An earlier version used a keepout filter
mask, which can only express lethal or <= 251; its lethal ring around each
cell put the robot's own footprint in lethal space once it stopped near an
obstacle, and Nav2's spin and backup recoveries then refused to move.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LowObstacleMemory(Node):
    def __init__(self) -> None:
        # rclpy's executor costs a few ms per wake-up on the Pi for every
        # entity it waits on; the parameter services are not needed here.
        super().__init__('low_obstacle_memory', start_parameter_services=False)
        self.declare_parameter('map_frame', '')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('mark_scan_topic', 'stereo_blob_scan')
        self.declare_parameter('clear_scan_topic', 'stereo_clear_scan')
        self.declare_parameter('bump_topic', 'bump_points')
        self.declare_parameter('points_topic', 'low_obstacle_points')
        # The cells within local_radius_m, in the base frame, for
        # stuck_detector: its reflexes must not drive into a remembered caster
        # the LiDAR cannot see. Published with the cloud, plus right after a bump.
        self.declare_parameter('local_points_topic', 'low_obstacle_points_local')
        self.declare_parameter('local_radius_m', 0.8)
        self.declare_parameter('resolution', 0.05)
        # Only cells within this distance of the robot are sent.
        self.declare_parameter('publish_radius_m', 6.0)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('extrude_m', 0.15)
        # Same as the costmap's stereo_clear_min_range (navigation2_slam.launch.py).
        self.declare_parameter('clear_min_range_m', 0.35)
        # Memory is for what the robot passed recently, not a map: a false
        # stereo mark that outlives the moment can wall the robot in, because
        # Nav2's costmap clears no longer remove it.
        self.declare_parameter('mark_ttl_sec', 120.0)
        self.declare_parameter('bump_ttl_sec', 180.0)
        # A stereo cell is remembered only after this many separate scans
        # mark it within confirm_window_sec. Single flickers are dropped.
        self.declare_parameter('confirm_scans', 2)
        self.declare_parameter('confirm_window_sec', 10.0)
        # ...and only when the confirming scan was taken at least
        # confirm_move_m or confirm_turn_rad from the first sighting. A real
        # object is there from every viewpoint; an artefact of one viewpoint
        # (floor noise at the far edge of the stereo band, seen frame after
        # frame while the robot waited) was remembered as an arc on open
        # floor. The local costmap's stereo layer still marks at once.
        self.declare_parameter('confirm_move_m', 0.10)
        self.declare_parameter('confirm_turn_rad', 0.2)
        self.declare_parameter('max_cells', 4000)
        # Only cells right under the robot are left out of the cloud. Low
        # objects closer than ~0.40 m are invisible to stereo and below the
        # LiDAR, so this memory is the only thing that still knows they are
        # there; leaving a wider ring out let a costmap clear erase the near
        # edge of a black box the robot then drove into. A remembered cell
        # touching the footprint outline does block Nav2's spin and backup;
        # stuck_detector's bumper reflex backs the robot off in that case.
        self.declare_parameter('keep_clear_radius_m', 0.12)
        # Bumps that land within bump_link_m of an earlier one (younger than
        # bump_link_sec) are joined by a line of cells. A robot sliding along
        # a large low object touches its face at points a little apart; the
        # line between them is that face, which the bumps alone left open.
        self.declare_parameter('bump_link_m', 0.6)
        self.declare_parameter('bump_link_sec', 120.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        self._map_frame = str(p('map_frame')).strip() or f'{prefix}map'
        self._base_frame = str(p('base_frame')).strip() or f'{prefix}base_footprint'
        self._res = float(p('resolution'))
        self._publish_radius = float(p('publish_radius_m'))
        self._extrude = float(p('extrude_m'))
        self._clear_min = float(p('clear_min_range_m'))
        self._ttl = {'mark': float(p('mark_ttl_sec')), 'bump': float(p('bump_ttl_sec'))}
        self._max_cells = int(p('max_cells'))
        self._keep_clear = float(p('keep_clear_radius_m'))
        self._bump_link = float(p('bump_link_m'))
        self._bump_link_sec = float(p('bump_link_sec'))
        self._recent_bumps = []  # (time, centroid x, centroid y)
        self._confirm_scans = max(1, int(p('confirm_scans')))
        self._confirm_window = float(p('confirm_window_sec'))
        self._confirm_move = float(p('confirm_move_m'))
        self._confirm_turn = float(p('confirm_turn_rad'))
        # (i, j) -> (sightings, first sighting time, first sighting pose) for cells not yet confirmed
        self._pending = {}

        # (i, j) map cell -> (expiry time, source)
        self._cells = {}

        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf, self)

        self._points_pub = self.create_publisher(PointCloud2, str(p('points_topic')), 5)
        self._local_pub = self.create_publisher(PointCloud2, str(p('local_points_topic')), 5)
        self._local_radius = float(p('local_radius_m'))

        self.create_subscription(LaserScan, str(p('mark_scan_topic')), self._on_marks, 10)
        self.create_subscription(LaserScan, str(p('clear_scan_topic')), self._on_clear, 10)
        self.create_subscription(PointCloud2, str(p('bump_topic')), self._on_bump, 10)
        # stereo_depth_node status: when it finds the pair out of alignment,
        # the marks it produced are not trustworthy and nothing will clear
        # them, so they are dropped. Bumps do not depend on the cameras.
        self.create_subscription(String, 'stereo/debug', self._on_stereo_status, 10)
        self.create_timer(1.0 / max(0.2, float(p('publish_hz'))), self._publish_points)
        self.get_logger().info(
            f'low obstacle memory: {self._map_frame} -> {self.resolve_topic_name(str(p("points_topic")))}')

    # --- geometry -------------------------------------------------------
    def _pose(self, frame: str, stamp, latest_ok: bool = False):
        """(x, y, yaw) of frame in the map frame at stamp, or None.

        Scans are only placed with the pose at their own stamp. Stereo frames
        are often a second old by the time they are processed, and the newest
        pose instead would smear a mark sideways while the robot turns.
        """
        try:
            tf = self._tf.lookup_transform(
                self._map_frame, frame, Time.from_msg(stamp), timeout=Duration(seconds=0.05))
        except TransformException:
            if not latest_ok:
                return None
            try:
                tf = self._tf.lookup_transform(self._map_frame, frame, Time())
            except TransformException:
                return None
        t = tf.transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _cell(self, x: float, y: float):
        return int(math.floor(x / self._res)), int(math.floor(y / self._res))

    def _beams(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > 0.0)
        return r[ok], a[ok]

    def _add(self, x: float, y: float, source: str, now: float) -> None:
        key = self._cell(x, y)
        expiry = now + self._ttl[source]
        old = self._cells.get(key)
        if old is None or old[0] < expiry:
            self._cells[key] = (expiry, source)

    # --- inputs ---------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_marks(self, msg: LaserScan) -> None:
        r, a = self._beams(msg)
        if r.size == 0:
            return
        pose = self._pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            return
        px, py, yaw = pose
        now = self._now()
        steps = np.arange(0.0, self._extrude + 1e-6, self._res)
        seen = set()
        for rng, ang in zip(r, a):
            c, s = math.cos(yaw + ang), math.sin(yaw + ang)
            for d in steps:
                seen.add(self._cell(px + (rng + d) * c, py + (rng + d) * s))
        for key in seen:
            if key in self._cells:
                self._cells[key] = (max(self._cells[key][0], now + self._ttl['mark']), self._cells[key][1])
                continue
            count, first, seen_from = self._pending.get(key, (0, now, pose))
            if now - first > self._confirm_window:
                count, first, seen_from = 0, now, pose
            if count > 0 and not self._new_viewpoint(seen_from, pose):
                continue
            count += 1
            if count >= self._confirm_scans:
                self._pending.pop(key, None)
                self._cells[key] = (now + self._ttl['mark'], 'mark')
            else:
                self._pending[key] = (count, first, seen_from)
        self._trim()

    def _new_viewpoint(self, a, b) -> bool:
        turn = abs((b[2] - a[2] + math.pi) % (2.0 * math.pi) - math.pi)
        return math.hypot(b[0] - a[0], b[1] - a[1]) >= self._confirm_move or turn >= self._confirm_turn

    def _on_clear(self, msg: LaserScan) -> None:
        if not self._cells:
            return
        r, a = self._beams(msg)
        if r.size == 0:
            return
        pose = self._pose(msg.header.frame_id, msg.header.stamp)
        if pose is None:
            return
        px, py, yaw = pose
        step = 0.5 * self._res
        for rng, ang in zip(r, a):
            # Stop one cell short of the beam end: a beam that ends on an
            # obstacle must not erase it.
            end = rng - self._res
            if end <= self._clear_min:
                continue
            c, s = math.cos(yaw + ang), math.sin(yaw + ang)
            for d in np.arange(self._clear_min, end, step):
                key = self._cell(px + d * c, py + d * s)
                self._cells.pop(key, None)
                self._pending.pop(key, None)

    def _on_bump(self, msg: PointCloud2) -> None:
        pose = self._pose(msg.header.frame_id, msg.header.stamp, latest_ok=True)
        if pose is None:
            self.get_logger().warning('bump received but no map transform; not remembered')
            return
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        now = self._now()
        pts = [(px + c * x - s * y, py + s * x + c * y) for x, y, _z in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        for wx, wy in pts:
            self._add(wx, wy, 'bump', now)
        linked = 0
        if pts:
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            self._recent_bumps = [b for b in self._recent_bumps if now - b[0] <= self._bump_link_sec]
            for _t, bx, by in self._recent_bumps:
                d = math.hypot(cx - bx, cy - by)
                if 0.0 < d <= self._bump_link:
                    for f in np.arange(0.0, 1.0 + 1e-6, 0.5 * self._res / d):
                        self._add(bx + f * (cx - bx), by + f * (cy - by), 'bump', now)
                    linked += 1
            self._recent_bumps.append((now, cx, cy))
        self._publish_points()
        self.get_logger().info(
            f'remembered bump: {len(pts)} points near ({px:.2f}, {py:.2f})'
            + (f', joined to {linked} earlier bump(s)' if linked else ''))

    def _on_stereo_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data).get('status')
        except (ValueError, AttributeError):
            return
        if status != 'misaligned':
            return
        marks = [k for k, v in self._cells.items() if v[1] == 'mark']
        if marks or self._pending:
            for key in marks:
                del self._cells[key]
            self._pending.clear()
            self.get_logger().warning(
                f'stereo out of alignment: forgot {len(marks)} remembered stereo cells')

    def _trim(self) -> None:
        if len(self._cells) <= self._max_cells:
            return
        keep = sorted(self._cells.items(), key=lambda kv: kv[1][0], reverse=True)[:self._max_cells]
        self._cells = dict(keep)

    # --- output ---------------------------------------------------------
    def _publish_points(self) -> None:
        now = self._now()
        for key in [k for k, v in self._cells.items() if v[0] < now]:
            del self._cells[key]
        for key in [k for k, v in self._pending.items() if now - v[1] > self._confirm_window]:
            del self._pending[key]
        stamp = self.get_clock().now().to_msg()
        try:
            # The pose the TF buffer has now, and its own stamp, so the local
            # cloud says exactly which robot pose it is relative to.
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException:
            return
        t = tf.transform
        rx, ry, ryaw = t.translation.x, t.translation.y, _yaw(t.rotation)
        pts = []
        local = []
        if self._cells:
            cells = np.asarray(list(self._cells.keys()), dtype=float)
            xy = (cells + 0.5) * self._res
            dist = np.hypot(xy[:, 0] - rx, xy[:, 1] - ry)
            keep = (dist > self._keep_clear) & (dist < self._publish_radius)
            pts = [[float(x), float(y), 0.05] for x, y in xy[keep]]
            near = xy[dist < self._local_radius] - np.array([rx, ry])
            c, s = math.cos(ryaw), math.sin(ryaw)
            local = [[float(c * dx + s * dy), float(-s * dx + c * dy), 0.05] for dx, dy in near]
        header = Header()
        header.stamp = stamp
        header.frame_id = self._map_frame
        # Sent every period, empty or not: the layer re-marks after a costmap clear.
        self._points_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
        local_header = Header()
        local_header.stamp = tf.header.stamp
        local_header.frame_id = self._base_frame
        self._local_pub.publish(point_cloud2.create_cloud_xyz32(local_header, local))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowObstacleMemory()
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
