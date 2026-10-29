#!/usr/bin/env python3
"""Clear SLAM obstacles that the LiDAR now sees through (map -> map_live).

slam_toolbox builds each cell from every scan it ever kept: a cell is
occupied while hits / passes > occupancy_threshold (0.1). A door seen closed
for a few dozen scans therefore needs hundreds of beams through it before the
map shows it open, and until then the global costmap (static layer), the
central explorer and map_merge all treat the open doorway as a wall. People
who stood still for a moment, a chair that has since been moved, or a
backpack picked up off the floor leave the same kind of stale mark.

This node republishes the SLAM map with those stale obstacles removed. For
every scan (up to max_scan_hz) it traces each beam through the map: cells a
beam passed through, at least end_margin_m short of where it ended, count one
pass; the cell a beam ends
in counts a hit and resets that cell's count. A cell SLAM calls occupied is
published free once it has been passed through clear_passes times in a row
and has not been hit for hit_hold_sec. The first hit puts it back.

A beam only counts as passing a cell if, after it, the beam gets at least
open_space_m from every SLAM obstacle before it ends. A beam through an open
door carries on into the room beyond. A beam that only seems to cross a
solid wall does not: it grazes the wall, or the LiDAR sees the wall a cell
or two from where SLAM drew it, and either way it ends on that same wall,
never leaving it. Counted as passes, such beams wore walls away whenever the
robot drove off and saw them at a shallow angle (an earlier test on the
wall's direction failed on SLAM's thick, ragged walls), and each hole opened
a frontier onto the unknown space behind. As a last guard, a cell next to
unknown space is never cleared: that is where a hole becomes a frontier.

Unknown cells the beams have passed through the same way are published free,
and unknown cells next to them that beams keep ending in, occupied. SLAM
only adds a scan after the robot has moved (minimum_travel_distance), so a
robot that stops in front of a door that has just opened never fills in the
space right behind it; that unknown pocket then kept the door cells, which
border it, from clearing, and the robot stood boxed in for minutes.

Known free cells are never marked occupied here. Something new in the way
(a closed door, a moved chair) reaches the map through SLAM itself, which
marks a cell after a few hits, and reaches Nav2 at once through the costmaps'
obstacle layers. Moving people are handled the same way: they are marked
in the costmaps while seen and cleared again by the beams behind them.

map_live goes to the global costmap's static layer and, through
map_wire_compressed_republisher, to the central PC.
"""

import array
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from rclpy_lean import lean_events

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


_CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
_SQUARE = np.ones((3, 3), np.uint8)
# last_hit of a cell no beam has ended in.
_NEVER = -1e6


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LiveMapFilter(Node):
    def __init__(self) -> None:
        super().__init__('live_map_filter', start_parameter_services=False)
        lean_events(self)
        self.declare_parameter('map_frame', '')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('scan_topic', 'scan_normalized')
        self.declare_parameter('output_topic', 'map_live')
        # Scans are traced at most this often. Every scan is not needed: a
        # cell needs clear_passes in a row, and the Pi has better uses.
        self.declare_parameter('max_scan_hz', 2.5)
        # Beams are traced to this range. Beyond it one beam per degree is
        # more than a cell apart and range noise grows; the costmap obstacle
        # layers clear to the same 3.5 m.
        self.declare_parameter('clear_max_range_m', 3.5)
        self.declare_parameter('clear_min_range_m', 0.12)
        # A beam counts as passing a cell only up to this short of where it
        # ended. The LiDAR often sees a wall a few cm beyond where SLAM drew
        # it, so beams ended just behind SLAM's wall cells and, at 0.10 m,
        # wore a solid wall away. Through an open door a beam carries on well
        # past the door cells.
        self.declare_parameter('end_margin_m', 0.5)
        self.declare_parameter('clear_passes', 6)
        # Hits in a row an unknown cell needs to be published as an obstacle.
        self.declare_parameter('confirm_hits', 3)
        # ...and the cell has not been hit for this long. Chair legs and
        # glass return some beams and not others; they stay.
        self.declare_parameter('hit_hold_sec', 8.0)
        # SLAM values at or above this are obstacles.
        self.declare_parameter('occupied_threshold', 65)
        # See module doc: how far from every obstacle a beam must get, after
        # a cell, for it to count as passing that cell.
        self.declare_parameter('open_space_m', 0.2)
        self.declare_parameter('publish_hz', 1.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        self._map_frame = str(p('map_frame')).strip() or f'{prefix}map'
        self._min_scan_period = 1.0 / max(0.5, float(p('max_scan_hz')))
        self._clear_max = float(p('clear_max_range_m'))
        self._clear_min = float(p('clear_min_range_m'))
        self._end_margin = float(p('end_margin_m'))
        self._clear_passes = max(1, int(p('clear_passes')))
        self._confirm_hits = max(1, int(p('confirm_hits')))
        self._hit_hold = float(p('hit_hold_sec'))
        self._occ = int(p('occupied_threshold'))
        self._open_m = float(p('open_space_m'))
        # Per cell: distance (m) to the nearest SLAM obstacle.
        self._clearance = None

        self._map = None          # latest SLAM OccupancyGrid
        self._grid = None         # its data as int8 (h, w)
        # Per-cell evidence, aligned with self._grid: passes since the last
        # hit, and the time of the last hit (seconds since start).
        self._passes = None
        self._hits = None         # hits since the last pass
        self._last_hit = None
        self._dirty = False
        self._last_scan = 0.0
        self._pending = None      # scan waiting for its pose (see _on_scan)
        self._cleared_logged = 0
        # Times are kept relative to start: float32 cannot hold epoch seconds.
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self._tf = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(OccupancyGrid, str(p('output_topic')), MAP_QOS)
        self.create_subscription(OccupancyGrid, str(p('map_topic')), self._on_map, MAP_QOS)
        # raw=True: scans that arrive between traces are dropped without being
        # deserialized (the LiDAR sends 10 Hz; Python conversion is the cost).
        self.create_subscription(
            LaserScan, str(p('scan_topic')), self._on_scan, qos_profile_sensor_data, raw=True)
        self.create_timer(1.0 / max(0.2, float(p('publish_hz'))), self._publish)
        self.get_logger().info(
            f'live map: {self.resolve_topic_name(str(p("map_topic")))} -> '
            f'{self.resolve_topic_name(str(p("output_topic")))} (clears obstacles seen through '
            f'{self._clear_passes} scans in a row, none hit for {self._hit_hold:.0f}s)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self._t0

    # --- map ------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        h, w = msg.info.height, msg.info.width
        if h == 0 or w == 0:
            return
        grid = np.frombuffer(memoryview(msg.data), dtype=np.int8).reshape(h, w)
        old = self._map
        if old is None or self._passes is None:
            self._passes = np.zeros((h, w), dtype=np.uint8)
            self._hits = np.zeros((h, w), dtype=np.uint8)
            self._last_hit = np.full((h, w), _NEVER, dtype=np.float32)
        elif (old.info.width, old.info.height) != (w, h) or \
                old.info.origin.position.x != msg.info.origin.position.x or \
                old.info.origin.position.y != msg.info.origin.position.y or \
                old.info.resolution != msg.info.resolution:
            self._passes, self._hits, self._last_hit = self._realign(old.info, msg.info)
        self._map = msg
        self._grid = grid
        self._clearance = cv2.distanceTransform(
            (grid < self._occ).astype(np.uint8), cv2.DIST_L2, 3) * msg.info.resolution
        self._dirty = True

    def _realign(self, old, new):
        """Carry the evidence over when SLAM grows or moves the grid."""
        passes = np.zeros((new.height, new.width), dtype=np.uint8)
        hits = np.zeros((new.height, new.width), dtype=np.uint8)
        last_hit = np.full((new.height, new.width), _NEVER, dtype=np.float32)
        if abs(old.resolution - new.resolution) > 1e-6:
            return passes, hits, last_hit
        res = new.resolution
        # Cell (0, 0) of the old grid is at (dj, di) in the new one.
        dj = int(round((old.origin.position.x - new.origin.position.x) / res))
        di = int(round((old.origin.position.y - new.origin.position.y) / res))
        oi0, oj0 = max(0, -di), max(0, -dj)
        ni0, nj0 = max(0, di), max(0, dj)
        rows = min(old.height - oi0, new.height - ni0)
        cols = min(old.width - oj0, new.width - nj0)
        if rows > 0 and cols > 0:
            passes[ni0:ni0 + rows, nj0:nj0 + cols] = self._passes[oi0:oi0 + rows, oj0:oj0 + cols]
            hits[ni0:ni0 + rows, nj0:nj0 + cols] = self._hits[oi0:oi0 + rows, oj0:oj0 + cols]
            last_hit[ni0:ni0 + rows, nj0:nj0 + cols] = self._last_hit[oi0:oi0 + rows, oj0:oj0 + cols]
        return passes, hits, last_hit

    # --- scans ----------------------------------------------------------
    def _on_scan(self, raw: bytes) -> None:
        if self._map is None:
            return
        # A scan is traced one scan later, with the pose at its own stamp.
        # When it arrives, the newest map -> base transform is usually a few
        # ms older than it, and with the TF listener on this node's thread a
        # lookup timeout cannot wait for a newer one. Tracing with the newest
        # pose instead was up to 0.2 s off: turning at 1 rad/s that is 11
        # degrees, 40 cm at 2 m, and beams swept straight through walls.
        pending, self._pending = self._pending, None
        if pending is not None:
            try:
                tf = self._tf.lookup_transform(
                    self._map_frame, pending.header.frame_id, Time.from_msg(pending.header.stamp))
            except TransformException:
                tf = None
            if tf is not None:
                self._trace(pending, tf)
        now = self._now()
        if now - self._last_scan >= self._min_scan_period:
            self._last_scan = now
            self._pending = deserialize_message(raw, LaserScan)

    def _trace(self, msg: LaserScan, tf) -> None:
        now = self._now()
        t = tf.transform
        px, py, yaw = t.translation.x, t.translation.y, _yaw(t.rotation)

        r = np.asarray(msg.ranges, dtype=np.float32)
        a = (msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment + yaw)
        # A beam at range_max (or inf/0) found nothing; it says nothing about
        # where it stopped, so it is not used at all.
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.01)) & (r < msg.range_max - 1e-3)
        r, a = r[ok], a[ok]
        if r.size == 0:
            return
        info = self._map.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        h, w = self._passes.shape
        c, s = np.cos(a), np.sin(a)

        # Samples every half cell from clear_min to where the beam ended (or
        # clear_max), each with its distance to the nearest SLAM obstacle.
        d = np.arange(self._clear_min, self._clear_max, 0.5 * res, dtype=np.float32)
        inside = d[None, :] < np.minimum(r, self._clear_max)[:, None]
        pj = np.floor((px + c[:, None] * d[None, :] - ox) / res).astype(np.int64)
        pi = np.floor((py + s[:, None] * d[None, :] - oy) / res).astype(np.int64)
        inb = (pi >= 0) & (pi < h) & (pj >= 0) & (pj < w)
        flat = np.where(inb, pi * w + pj, 0)
        # Off the map counts as open space.
        dist = np.where(inb, self._clearance.reshape(-1)[flat], 1e3)
        dist[~inside] = 0.0
        # Furthest the beam gets from any obstacle from each sample on.
        ahead = np.maximum.accumulate(dist[:, ::-1], axis=1)[:, ::-1]
        # A pass needs the beam to end end_margin_m further on and, before
        # that, to reach open space (see module doc).
        passing = (
            inb & inside
            & (d[None, :] < (r - self._end_margin)[:, None])
            & (ahead >= self._open_m)
        )
        passed = np.unique(flat[passing])

        # Hit cells: where beams within clear_max ended.
        near = r <= self._clear_max
        hj = np.floor((px + c[near] * r[near] - ox) / res).astype(np.int64)
        hi = np.floor((py + s[near] * r[near] - oy) / res).astype(np.int64)
        inb = (hi >= 0) & (hi < h) & (hj >= 0) & (hj < w)
        hit = np.unique(hi[inb] * w + hj[inb])

        flat_p = self._passes.reshape(-1)
        flat_h = self._last_hit.reshape(-1)
        passed = np.setdiff1d(passed, hit, assume_unique=True)
        flat_p[passed] = np.minimum(flat_p[passed].astype(np.int16) + 1, 255).astype(np.uint8)
        flat_p[hit] = 0
        flat_h[hit] = now
        flat_n = self._hits.reshape(-1)
        flat_n[passed] = 0
        flat_n[hit] = np.minimum(flat_n[hit].astype(np.int16) + 1, 255).astype(np.uint8)
        self._dirty = True

    # --- output ---------------------------------------------------------
    def _publish(self) -> None:
        if not self._dirty or self._map is None:
            return
        self._dirty = False
        now = self._now()
        out = self._grid.copy()
        seen_free = (self._passes >= self._clear_passes) & ((now - self._last_hit) >= self._hit_hold)
        out[(out < 0) & seen_free] = 0
        near_unknown = cv2.dilate((out < 0).astype(np.uint8), _CROSS) > 0
        clear = (out >= self._occ) & ~near_unknown & seen_free
        n = int(np.count_nonzero(clear))
        out[clear] = 0
        # Unknown cells beams keep ending in are obstacles. After SLAM shifts
        # the map a little, the LiDAR finds a wall a cell or two beyond where
        # SLAM drew it: the old wall cells clear, the strip up to the real
        # wall is seen free, and the real wall left unknown was a frontier the
        # whole length of the wall. Only cells hit confirm_hits times with no
        # beam through since, next to free space and not next to an obstacle,
        # are added: with one hit, every stray beam thickened the walls and
        # dotted the unknown space outside them.
        occ = (out >= self._occ).astype(np.uint8)
        free = (out == 0).astype(np.uint8)
        add = (
            (out < 0) & (self._hits >= self._confirm_hits)
            & (cv2.dilate(free, _CROSS) > 0)
            & ~(cv2.dilate(occ, _SQUARE) > 0)
        )
        out[add] = 100
        msg = OccupancyGrid()
        msg.header = self._map.header
        msg.info = self._map.info
        msg.data = array.array('b', out.tobytes())
        self._pub.publish(msg)
        # Log when the number of cleared cells moves by a door's worth.
        if abs(n - self._cleared_logged) >= 10:
            self.get_logger().info(f'{n} stale SLAM obstacle cell(s) cleared in map_live')
            self._cleared_logged = n


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveMapFilter()
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
