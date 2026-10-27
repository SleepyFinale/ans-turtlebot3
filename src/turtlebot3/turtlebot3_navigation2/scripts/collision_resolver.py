#!/usr/bin/env python3
"""Work out why the controller gave up on "collision ahead", and act on it.

Nav2's controller aborts follow_path when its projected footprint keeps
touching lethal cells of the local costmap. The costmap merges LiDAR, stereo
and the low-obstacle memory, so the abort alone cannot say whether something
is really there. On Inky the same blocked spot was retried for minutes: a
stereo mark only the local costmap had (so the global plan kept going
through it), a remembered cell the cameras could no longer see, or a stale
mark nothing explained any more.

On each abort this node takes the local costmap from just before it (the
behavior tree clears the costmap right after), finds the lethal cells around
and ahead of the robot, and checks each against what the sensors see now:

- LiDAR return there: real.
- In the cameras' view (0.35-1.4 m ahead, within their bearing): a stereo
  mark there is real; floor matched through it is fake; no evidence either
  way is unverified.
- Out of view: a remembered cell is unverified; a cell no sensor explains
  any more is stale.

Then:

- fake / stale: forget those cells in low_obstacle_memory (memory/forget)
  and clear the local costmap (and the global one for stale cells);
- real from stereo or memory: remember them (memory/confirm) so the global
  planner, which has no stereo layer of its own, routes around them, and
  back off a little if they are close;
- real from LiDAR: back off a little if close, so the controller has room
  to turn onto a new plan;
- unverified in front: back off so the cells come into the cameras' view
  (stuck/reflex_request, run by stuck_detector with its own clear-strip
  check), then look again; still unverified counts as real.

Each verdict is logged in one line and published as JSON on
collision/diagnosis.
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String

LETHAL = 100  # nav2 publishes cost 254 as 100 in the OccupancyGrid


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _scan_xy(msg: LaserScan, x0: float = 0.0) -> np.ndarray:
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r > 0.0)
    return np.stack([r[ok] * np.cos(a[ok]) + x0, r[ok] * np.sin(a[ok])], axis=1)


def _near(cells: np.ndarray, pts: np.ndarray, tol: float) -> np.ndarray:
    if len(cells) == 0 or len(pts) == 0:
        return np.zeros(len(cells), dtype=bool)
    d = np.hypot(cells[:, None, 0] - pts[None, :, 0], cells[:, None, 1] - pts[None, :, 1])
    return d.min(axis=1) <= tol


class CollisionResolver(Node):
    def __init__(self) -> None:
        super().__init__('collision_resolver', start_parameter_services=False)
        self.declare_parameter('lidar_x_m', -0.032)
        # Lethal cells looked at: ahead of the axle up to ahead_m within
        # half_width_m of the centre line, or anywhere within around_m
        # (the controller also stops turns that would clip something).
        self.declare_parameter('ahead_m', 0.50)
        self.declare_parameter('half_width_m', 0.22)
        self.declare_parameter('around_m', 0.28)
        # The cameras check obstacles and floor between these ranges
        # (stereo_clear_min_range in navigation2_slam.launch.py).
        self.declare_parameter('view_min_m', 0.35)
        self.declare_parameter('view_max_m', 1.40)
        # Distance from a cell centre at which a sensor point explains it.
        self.declare_parameter('match_m', 0.07)
        # Real cells closer than this get a short back-off (metres).
        self.declare_parameter('backoff_within_m', 0.30)
        self.declare_parameter('backoff_m', 0.10)
        self.declare_parameter('cooldown_sec', 3.0)
        # This many real blocks within give_up_window_sec with the robot within
        # 0.3 m of the same spot: cancel the navigation goal, so the explorer
        # picks another frontier instead of Nav2 retrying the same approach.
        # On Inky one pocket among chair legs gave six real blocks in a minute.
        # 0 disables.
        self.declare_parameter('give_up_blocks', 3)
        self.declare_parameter('give_up_window_sec', 60.0)
        # How long to wait for fresh sensor data after an abort.
        self.declare_parameter('wait_sec', 3.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._lidar_x = float(p('lidar_x_m'))
        self._ahead = float(p('ahead_m'))
        self._half = float(p('half_width_m'))
        self._around = float(p('around_m'))
        self._view_min = float(p('view_min_m'))
        self._view_max = float(p('view_max_m'))
        self._match = float(p('match_m'))
        self._backoff_within = float(p('backoff_within_m'))
        self._backoff = float(p('backoff_m'))
        self._cooldown = float(p('cooldown_sec'))
        self._give_up_blocks = int(p('give_up_blocks'))
        self._give_up_window = float(p('give_up_window_sec'))
        self._wait = float(p('wait_sec'))
        ns = self.get_namespace().strip('/')
        self._base_frame = f'{ns}/base_footprint' if ns else 'base_footprint'

        # Recent local costmaps: (monotonic time, grid HxW int8, (ox, oy, res)).
        self._grids = deque(maxlen=4)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'local_costmap/costmap', self._on_grid, latched)
        self.create_subscription(
            OccupancyGridUpdate, 'local_costmap/costmap_updates', self._on_grid_update, 10)
        self.create_subscription(
            GoalStatusArray, 'follow_path/_action/status', self._on_follow_status, 10)
        self._reflex_active = False
        self._reflex_end_t = -1e9  # when the last reflex stopped (monotonic)
        self.create_subscription(Bool, 'stuck/reflex_active', self._on_reflex_active, 5)
        # Nav2's own recoveries move the robot too; a back-off requested
        # while one runs fought it (a spin and a reflex at once on Inky).
        self._behaviors = {}
        for name in ('spin', 'backup', 'wait'):
            self.create_subscription(
                GoalStatusArray, f'{name}/_action/status',
                lambda m, n=name: self._behaviors.__setitem__(n, any(
                    st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                    for st in m.status_list)), 5)
        self._seen_aborts = set()

        self._forget_pub = self.create_publisher(PointCloud2, 'memory/forget', 5)
        self._confirm_pub = self.create_publisher(PointCloud2, 'memory/confirm', 5)
        self._reflex_pub = self.create_publisher(Float32, 'stuck/reflex_request', 5)
        self._diag_pub = self.create_publisher(String, 'collision/diagnosis', 5)
        self._clear_local = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self._cancel_nav = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')

        # Sensor inputs are subscribed only during an episode: scan and odom
        # at 10-30 Hz in rclpy cost the Pi CPU the rest of the time.
        self._subs = []
        self._latest = {}
        self._arrived = {}
        self._episode = None
        self._last_episode_end = -1e9
        self._recent_real = deque(maxlen=10)  # (time, odom x, y)
        self.create_timer(0.25, self._tick)
        self.get_logger().info('collision resolver: diagnosing controller give-ups')

    # --- local costmap ------------------------------------------------------
    def _on_grid(self, msg: OccupancyGrid) -> None:
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        self._grids.append((time.monotonic(), grid,
                            (info.origin.position.x, info.origin.position.y, info.resolution)))

    def _on_grid_update(self, msg: OccupancyGridUpdate) -> None:
        if not self._grids:
            return
        _t, grid, meta = self._grids[-1]
        grid = grid.copy()
        h, w = int(msg.height), int(msg.width)
        y0, x0 = int(msg.y), int(msg.x)
        if y0 + h > grid.shape[0] or x0 + w > grid.shape[1]:
            return
        grid[y0:y0 + h, x0:x0 + w] = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        self._grids.append((time.monotonic(), grid, meta))

    # --- trigger ------------------------------------------------------------
    def _on_follow_status(self, msg: GoalStatusArray) -> None:
        # A new path replaces the running follow_path goal, and the replaced
        # one is reported ABORTED with its successor already active in the
        # same status array. Only an abort with nothing else running is the
        # controller giving up.
        if any(st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
               for st in msg.status_list):
            for st in msg.status_list:
                if st.status == GoalStatus.STATUS_ABORTED:
                    self._seen_aborts.add(bytes(st.goal_info.goal_id.uuid))
            return
        for st in msg.status_list:
            if st.status != GoalStatus.STATUS_ABORTED:
                continue
            key = bytes(st.goal_info.goal_id.uuid)
            if key in self._seen_aborts:
                continue
            self._seen_aborts.add(key)
            if len(self._seen_aborts) > 500:
                self._seen_aborts = {key}
            self._start_episode()

    def _start_episode(self) -> None:
        now = time.monotonic()
        if self._episode is not None or now - self._last_episode_end < self._cooldown:
            return
        # The behavior tree clears the local costmap right after the abort;
        # take the most-blocked grid of the last 1.5 s.
        recent = [g for g in self._grids if now - g[0] <= 1.5]
        if not recent:
            return
        snap = max(recent, key=lambda g: int(np.count_nonzero(g[1] == LETHAL)))
        self._episode = {'t0': now, 'grid': snap, 'recheck': None}
        self._subscribe_inputs()

    def _on_reflex_active(self, msg: Bool) -> None:
        if self._reflex_active and not msg.data:
            self._reflex_end_t = time.monotonic()
        self._reflex_active = bool(msg.data)

    def _subscribe_inputs(self) -> None:
        self._latest = {}
        self._arrived = {}
        if self._subs:
            return

        def put(key, msg):
            self._latest[key] = msg
            self._arrived[key] = time.monotonic()
        self._subs = [
            self.create_subscription(Odometry, 'odom', lambda m: put('odom', m), 5),
            self.create_subscription(LaserScan, 'scan', lambda m: put('scan', m), qos_profile_sensor_data),
            self.create_subscription(LaserScan, 'stereo_blob_scan', lambda m: put('marks', m), 5),
            self.create_subscription(LaserScan, 'stereo_clear_scan', lambda m: put('clear', m), 5),
            self.create_subscription(PointCloud2, 'low_obstacle_points_local', lambda m: put('memory', m), 5),
        ]

    def _end_episode(self) -> None:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        self._episode = None
        self._last_episode_end = time.monotonic()

    def _tick(self) -> None:
        ep = self._episode
        if ep is None:
            return
        now = time.monotonic()
        if now - ep['t0'] > 20.0:
            self._end_episode()
            return
        if self._reflex_active or self._behavior_busy():
            return
        start = ep['t0']
        if ep['recheck'] is not None:
            # Look again only once the back-off has had time to start and
            # has finished, with data that arrived after the robot stopped;
            # a frame from before it moved re-checked from the same spot.
            if now - ep['recheck'] < 1.0:
                return
            start = max(ep['recheck'] + 1.0, self._reflex_end_t)
        have = {k for k in ('odom', 'scan', 'clear') if self._arrived.get(k, -1e9) > start}
        ready = 'odom' in have and 'scan' in have
        # Wait for a camera frame too, unless the cameras are silent.
        if ready and ('clear' in have or now - start >= self._wait):
            try:
                self._diagnose()
            except Exception as exc:  # a diagnostic must never take the stack down
                self.get_logger().warning(f'collision diagnosis failed: {exc}')
                self._end_episode()
        elif now - start > 2.0 * self._wait:
            self._end_episode()

    def _behavior_busy(self) -> bool:
        return any(self._behaviors.values())

    def _request_backoff(self, dist: float) -> bool:
        if self._behavior_busy() or self._reflex_active:
            return False
        self._reflex_pub.publish(Float32(data=-dist))
        return True

    # --- diagnosis ----------------------------------------------------------
    def _pose(self):
        o = self._latest['odom'].pose.pose
        return o.position.x, o.position.y, _yaw(o.orientation)

    def _to_base(self, xy_odom: np.ndarray) -> np.ndarray:
        rx, ry, ryaw = self._pose()
        c, s = math.cos(ryaw), math.sin(ryaw)
        dx, dy = xy_odom[:, 0] - rx, xy_odom[:, 1] - ry
        return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)

    def _to_odom(self, xy_base: np.ndarray) -> np.ndarray:
        rx, ry, ryaw = self._pose()
        c, s = math.cos(ryaw), math.sin(ryaw)
        return np.stack([rx + c * xy_base[:, 0] - s * xy_base[:, 1],
                         ry + s * xy_base[:, 0] + c * xy_base[:, 1]], axis=1)

    def _blocking_cells(self) -> np.ndarray:
        """Lethal cells of the snapshot around and ahead of the robot, in the odom frame."""
        _t, grid, (ox, oy, res) = self._episode['grid']
        js, is_ = np.nonzero(grid == LETHAL)
        cells = np.stack([ox + (is_ + 0.5) * res, oy + (js + 0.5) * res], axis=1)
        if len(cells) == 0:
            return cells
        b = self._to_base(cells)
        keep = ((b[:, 0] >= -0.05) & (b[:, 0] <= self._ahead) & (np.abs(b[:, 1]) <= self._half)) \
            | (np.hypot(b[:, 0], b[:, 1]) <= self._around)
        return cells[keep]

    def _classify(self, cells_odom: np.ndarray):
        """Per cell: lidar / stereo / floor / unseen / close / memory / stale (base-frame xy too)."""
        b = self._to_base(cells_odom)
        lidar = _scan_xy(self._latest['scan'], self._lidar_x)
        memory = np.zeros((0, 2))
        if 'memory' in self._latest:
            memory = point_cloud2.read_points_numpy(
                self._latest['memory'], field_names=('x', 'y'), skip_nans=True).astype(float).reshape(-1, 2)
        # Marks and clear beams come from the same stereo frame; marks from an
        # older frame (before a back-off) would sit in the wrong place.
        fresh_marks = 'marks' in self._latest and \
            self._arrived.get('marks', -1e9) >= self._arrived.get('clear', -1e9) - 0.3
        marks = _scan_xy(self._latest['marks']) if fresh_marks else np.zeros((0, 2))
        clear = self._latest.get('clear')
        rng = np.hypot(b[:, 0], b[:, 1])
        brg = np.arctan2(b[:, 1], b[:, 0])
        in_view = np.zeros(len(b), dtype=bool)
        free_beyond = np.zeros(len(b), dtype=bool)
        if clear is not None and len(clear.ranges):
            cr = np.asarray(clear.ranges, dtype=float)
            idx = np.rint((brg - clear.angle_min) / clear.angle_increment).astype(int)
            in_view = (b[:, 0] > 0.0) & (rng >= self._view_min) & (rng <= self._view_max) \
                & (idx >= 0) & (idx < cr.size)
            safe = np.clip(idx, 0, cr.size - 1)
            # Neighbouring beams too: a cell is 5 cm wide, a beam 1 degree.
            lo, hi = np.clip(safe - 1, 0, cr.size - 1), np.clip(safe + 1, 0, cr.size - 1)
            reach = np.nanmax(np.stack([np.where(np.isfinite(cr[i]), cr[i], -1.0) for i in (lo, safe, hi)]), axis=0)
            free_beyond = in_view & (reach >= rng + 0.03)
        near_lidar = _near(b, lidar, self._match)
        near_mark = _near(b, marks, self._match + 0.05)
        near_memory = _near(b, memory, self._match)
        labels = np.full(len(b), 'stale', dtype=object)
        # Ahead, inside the cameras' bearing but closer than they check: only
        # stereo can have put it there, and only stereo can tell. Treated like
        # a remembered cell: back off and look.
        if clear is not None:
            half_fov = max(abs(clear.angle_min), abs(clear.angle_min + clear.angle_increment * (len(clear.ranges) - 1)))
            labels[(b[:, 0] > 0.0) & (rng < self._view_min) & (np.abs(brg) <= half_fov)] = 'close'
        labels[near_memory] = 'memory'
        labels[in_view] = 'unseen'
        labels[in_view & free_beyond] = 'floor'
        labels[in_view & near_mark] = 'stereo'
        labels[near_lidar] = 'lidar'
        return labels, b

    def _diagnose(self) -> None:
        ep = self._episode
        if ep['recheck'] is not None:
            cells = ep['cells']
        else:
            cells = self._blocking_cells()
            ep['cells'] = cells
        if len(cells) == 0:
            self._report('none', {}, None, 'nothing lethal near the robot in the costmap')
            self._end_episode()
            return
        labels, b = self._classify(cells)
        counts = {k: int(np.count_nonzero(labels == k)) for k in ('lidar', 'stereo', 'floor', 'unseen', 'close', 'memory', 'stale')}
        order = np.argsort(np.hypot(b[:, 0], b[:, 1]))
        k = int(order[0])
        nearest = (float(b[k, 0]), float(b[k, 1]), str(labels[k]))
        real = np.isin(labels, ['lidar', 'stereo'])
        unsure = np.isin(labels, ['unseen', 'close', 'memory'])
        fake = np.isin(labels, ['floor', 'stale'])
        rechecked = ep['recheck'] is not None

        if np.any(unsure) and not rechecked and not np.any(real):
            # Bring the cells in front into the cameras' band and look again.
            ahead = b[unsure & (b[:, 0] > 0.0)]
            if len(ahead):
                need = self._view_min + 0.05 - float(np.min(np.hypot(ahead[:, 0], ahead[:, 1])))
                back = min(0.20, max(0.08, need))
                moved = self._request_backoff(back)
                ep['recheck'] = time.monotonic()
                self._report('unverified', counts, nearest,
                             f'backing off {back:.2f} m to look at it with the cameras' if moved
                             else 'a Nav2 recovery is moving the robot; looking again after it')
                return

        actions = []
        if rechecked:
            # After backing off, the cameras looked straight at 'unseen' cells
            # and matched neither an obstacle nor floor: a mark stereo no
            # longer reproduces from a better view. Remembered cells were
            # confirmed from several views and 'close' ones could not be
            # looked at, so those stay real.
            fake = fake | (labels == 'unseen')
            unsure = unsure & ~(labels == 'unseen')
        if np.any(fake):
            self._publish_points(self._forget_pub, b[fake])
            actions.append(f'forgot {int(np.count_nonzero(fake))} fake cell(s)')
        confirm = (labels == 'stereo') | (unsure if rechecked else np.zeros(len(b), dtype=bool))
        if np.any(confirm):
            # Remembered cells reach the global costmap, so the next plan avoids them.
            self._publish_points(self._confirm_pub, b[confirm])
            actions.append(f'remembered {int(np.count_nonzero(confirm))} real cell(s) for the planner')
        if np.any(fake):
            # Fake cells that only the local stereo layer held are not in the
            # memory, so forgetting does not remove them; clearing does. Real
            # cells are back within a second (LiDAR, and the memory, which
            # now holds the confirmed ones), and the robot is stopped.
            self._clear('local')
            if counts['stale']:
                self._clear('global')
            actions.append('cleared the costmap')
        if np.any(real | confirm):
            dist = np.hypot(b[:, 0], b[:, 1])
            # Ahead only: backing up gives no turning room from something beside the robot.
            close = (real | confirm) & (b[:, 0] > 0.05) & (dist < self._backoff_within)
            if np.any(close) and self._request_backoff(self._backoff):
                actions.append(f'backing off {self._backoff:.2f} m for room to turn')
            rx, ry, _ = self._pose()
            now = time.monotonic()
            self._recent_real.append((now, rx, ry))
            here = [r for r in self._recent_real
                    if now - r[0] <= self._give_up_window and math.hypot(r[1] - rx, r[2] - ry) <= 0.3]
            if 0 < self._give_up_blocks <= len(here) and self._cancel_nav.service_is_ready():
                # An empty GoalInfo (zero id and stamp) cancels every goal.
                self._cancel_nav.call_async(CancelGoal.Request(goal_info=GoalInfo()))
                self._recent_real.clear()
                actions.append(f'{len(here)} real blocks here in {self._give_up_window:.0f} s: '
                               'cancelled the navigation goal so the explorer picks another')
            verdict = 'real'
        elif np.any(fake):
            verdict = 'fake'
        else:
            verdict = 'unverified'
        if rechecked and np.any(unsure):
            actions.append('still unverified after looking: treated as real')
        self._report(verdict, counts, nearest, '; '.join(actions) or 'no action')
        self._end_episode()

    def _publish_points(self, pub, xy_base: np.ndarray) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._base_frame
        pub.publish(point_cloud2.create_cloud_xyz32(header, [[float(x), float(y), 0.02] for x, y in xy_base]))

    def _clear(self, which: str) -> None:
        cli = self._clear_local if which == 'local' else self._clear_global
        if cli.service_is_ready():
            cli.call_async(ClearEntireCostmap.Request())

    def _report(self, verdict, counts, nearest, action) -> None:
        sources = ' '.join(f'{k}={v}' for k, v in counts.items() if v)
        near = '' if nearest is None else f'; nearest ({nearest[0]:+.2f},{nearest[1]:+.2f}) {nearest[2]}'
        line = f'collision ahead: {verdict} ({sources or "no cells"}{near}): {action}'
        if verdict in ('fake', 'unverified'):
            self.get_logger().warning(line)
        else:
            self.get_logger().info(line)
        self._diag_pub.publish(String(data=json.dumps({
            'verdict': verdict, 'counts': counts, 'nearest': nearest, 'action': action})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionResolver()
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
