#!/usr/bin/env python3
"""Notice when the robot is driving but not actually moving.

Three checks run over a sliding window (window_sec):

- stalled: cmd_vel asks for motion but the wheel encoders barely turn
  (motors pushing against something).
- slipping: the wheels turn but the robot does not translate. Evidence comes
  from the LiDAR (does the current scan fit "moved as far as the wheels say"
  or "did not move" better?) and from stereo visual odometry when present.
  A scan that cannot tell the two apart (long featureless corridor) abstains
  instead of guessing.
- not turning: cmd_vel asks for a turn but the heading (EKF, i.e. the gyro)
  barely changes.

On a stall or slip while driving forward or backward, the contact side is
published as bump_points (base_footprint frame) so low_obstacle_memory keeps
a lethal mark there that survives Nav2's costmap clears, and the planner
routes around the spot the robot could not see. stuck (Bool) is the latched
state and stuck/event carries one JSON line per detection.
"""

import json
import math
from collections import deque

import cv2
import numpy as np

cv2.setNumThreads(1)
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _interp(buf, t: float):
    """Nearest sample in a (t, value) deque, or None if t is outside it."""
    if not buf or t < buf[0][0] - 0.1 or t > buf[-1][0] + 0.1:
        return None
    return min(buf, key=lambda item: abs(item[0] - t))[1]


class StuckDetector(Node):
    def __init__(self) -> None:
        # rclpy's executor costs a few ms per wake-up on the Pi for every
        # entity it waits on; the parameter services are not needed here.
        super().__init__('stuck_detector', start_parameter_services=False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('wheel_odom_topic', 'odom/wheel')
        self.declare_parameter('visual_odom_topic', 'odom/visual')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('window_sec', 2.0)
        # Minimum motion inside the window before a check is meaningful.
        self.declare_parameter('min_travel_m', 0.07)
        self.declare_parameter('min_turn_rad', 0.5)
        # "Not moving" = actual motion below this fraction of the expected.
        self.declare_parameter('moved_fraction', 0.3)
        # LiDAR must prefer "not moved" by at least this mean fit error (m).
        self.declare_parameter('lidar_margin_m', 0.008)
        # Positive checks in a row before declaring stuck (checks run per scan).
        self.declare_parameter('confirm_checks', 2)
        # LiDAR mount in base_footprint (turtlebot3_burger.urdf base_scan).
        self.declare_parameter('lidar_x_m', -0.032)
        self.declare_parameter('lidar_yaw_rad', 0.0)
        # Contact line published on a stall/slip, at the body edge that
        # touched: the Burger reaches about 0.10 m ahead of base_footprint and
        # 0.12 m behind. A line further out lands on floor beyond a thin
        # object, where stereo correctly clears it, and the object itself is
        # forgotten. During contact the line sits inside Nav2's footprint,
        # which stops Nav2 pushing; the bumper reflex then backs the robot
        # off, leaving the line outside the footprint for spin and backup.
        self.declare_parameter('bump_front_m', 0.13)
        self.declare_parameter('bump_rear_m', 0.14)
        self.declare_parameter('bump_half_width_m', 0.09)
        self.declare_parameter('bump_cooldown_sec', 3.0)
        # Bumper reflex: after a contact that places a bump, back straight
        # away reflex_distance_m. Nav2's backup cannot do this: it refuses to
        # start while any lethal cell touches the footprint outline, and the
        # object the robot just touched is exactly that. The reflex drives
        # cmd_vel directly, faster than the velocity smoother publishes, and
        # only if the LiDAR shows the strip it will drive through is empty.
        # (A low object behind is invisible to the LiDAR; the robot came
        # from that side moments ago, which is why straight back is chosen.)
        self.declare_parameter('reflex_enabled', True)
        self.declare_parameter('reflex_distance_m', 0.20)
        self.declare_parameter('reflex_speed', 0.06)
        self.declare_parameter('reflex_timeout_sec', 6.0)
        self.declare_parameter('reflex_rate_hz', 40.0)
        self.declare_parameter('reflex_cooldown_sec', 10.0)
        # Burger body extent from base_footprint and the extra clearance the
        # LiDAR check asks for beyond the distance driven.
        self.declare_parameter('body_front_m', 0.10)
        self.declare_parameter('body_rear_m', 0.12)
        self.declare_parameter('body_half_width_m', 0.11)
        self.declare_parameter('reflex_margin_m', 0.10)
        # A turn in place that keeps stalling: a wheel against a chair leg or
        # a box corner beside the robot, outside every sensor's view. No bump
        # is placed (the contact could be on either side), but after
        # turn_stall_events stalls within turn_stall_window_sec the robot
        # moves straight back (or forward, if the back is blocked) by
        # turn_reflex_distance_m, which frees the wheel so the turn can work.
        self.declare_parameter('turn_stall_events', 2)
        self.declare_parameter('turn_stall_window_sec', 8.0)
        self.declare_parameter('turn_reflex_distance_m', 0.12)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        ns = self.get_namespace().strip('/')
        self._base_frame = str(p('base_frame')).strip() or (f'{ns}/base_footprint' if ns else 'base_footprint')
        self._window = float(p('window_sec'))
        self._min_travel = float(p('min_travel_m'))
        self._min_turn = float(p('min_turn_rad'))
        self._frac = float(p('moved_fraction'))
        self._lidar_margin = float(p('lidar_margin_m'))
        self._confirm = max(1, int(p('confirm_checks')))
        self._lidar_x = float(p('lidar_x_m'))
        self._lidar_yaw = float(p('lidar_yaw_rad'))
        self._bump_front = float(p('bump_front_m'))
        self._bump_rear = float(p('bump_rear_m'))
        self._bump_half = float(p('bump_half_width_m'))
        self._bump_cooldown = float(p('bump_cooldown_sec'))
        self._reflex_enabled = bool(p('reflex_enabled'))
        self._reflex_dist = float(p('reflex_distance_m'))
        self._reflex_speed = abs(float(p('reflex_speed')))
        self._reflex_timeout = float(p('reflex_timeout_sec'))
        self._reflex_cooldown = float(p('reflex_cooldown_sec'))
        self._body_front = float(p('body_front_m'))
        self._body_rear = float(p('body_rear_m'))
        self._body_half = float(p('body_half_width_m'))
        self._reflex_margin = float(p('reflex_margin_m'))
        self._turn_stall_events = max(1, int(p('turn_stall_events')))
        self._turn_stall_window = float(p('turn_stall_window_sec'))
        self._turn_reflex_dist = float(p('turn_reflex_distance_m'))
        self._turn_stalls = deque()
        self._reflex = None  # {'dir', 'start', 't0'} while backing away
        self._last_reflex = -1e9

        keep = int(self._window * 60) + 50
        self._cmd = deque(maxlen=keep)       # (wall t, v, w)
        self._wheel = deque(maxlen=keep)     # (t, (x, y, yaw))
        self._imu = deque(maxlen=keep)       # (t, yaw)
        self._vo = deque(maxlen=keep)        # (t, vx)
        self._scans = deque(maxlen=40)       # (t, Nx2 points in base)
        self._streak = {'stalled': 0, 'slipping': 0, 'not_turning': 0}
        self._stuck = False
        self._last_bump = 0.0
        self._last_check = {}

        self.create_subscription(Twist, str(p('cmd_vel_topic')), self._on_cmd, 20)
        self.create_subscription(Odometry, str(p('wheel_odom_topic')), self._on_wheel, 20)
        # odom supplies the heading (the EKF's, which follows the IMU; with
        # use_ekf:=false the driver's, which is the IMU heading). It also
        # stands in for odom/wheel when that topic is silent (use_ekf:=false).
        self._last_wheel_msg = -1e9
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_subscription(Odometry, str(p('visual_odom_topic')), self._on_vo, 10)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._on_scan, qos_profile_sensor_data)
        self._stuck_pub = self.create_publisher(Bool, 'stuck', 10)
        self._event_pub = self.create_publisher(String, 'stuck/event', 10)
        self._debug_pub = self.create_publisher(String, 'stuck/debug', 10)
        self._bump_pub = self.create_publisher(PointCloud2, 'bump_points', 10)
        self._cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic')), 10)
        self._reflex_pub = self.create_publisher(Bool, 'stuck/reflex_active', 10)
        self.create_timer(1.0 / max(5.0, float(p('reflex_rate_hz'))), self._reflex_tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info('stuck detector: cmd_vel vs wheels, wheels vs LiDAR/VO, turn vs gyro')

    # --- inputs ---------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd.append((self._now(), float(msg.linear.x), float(msg.angular.z)))

    def _on_odom(self, msg: Odometry) -> None:
        self._imu.append((_stamp(msg), _yaw(msg.pose.pose.orientation)))
        if self._now() - self._last_wheel_msg > 1.0:
            self._store_wheel(msg)

    def _on_wheel(self, msg: Odometry) -> None:
        self._last_wheel_msg = self._now()
        self._store_wheel(msg)

    def _store_wheel(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._wheel.append((_stamp(msg), (p.position.x, p.position.y, _yaw(p.orientation))))

    def _on_vo(self, msg: Odometry) -> None:
        self._vo.append((_stamp(msg), float(msg.twist.twist.linear.x)))

    def _on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment + self._lidar_yaw
        ok = np.isfinite(r) & (r > max(0.12, msg.range_min)) & (r < min(4.0, msg.range_max))
        pts = np.stack([r[ok] * np.cos(a[ok]) + self._lidar_x, r[ok] * np.sin(a[ok])], axis=1)
        t = _stamp(msg)
        self._scans.append((t, pts))
        self._check(t)

    # --- evidence -------------------------------------------------------
    def _commanded(self, t0: float, t1: float):
        """(linear distance, |angle|, mean v sign, steady, turn_steady) asked for in [t0, t1].

        steady is False when the drive commands were mixed with stops: the
        stereo and ultrasonic enforcers publish zero cmd_vel next to Nav2's
        commands, and the base then barely moves on purpose. A stall only
        counts while every publisher kept asking for the same direction.
        turn_steady is the same test for the turn rate (not-turning check).
        """
        samples = [c for c in self._cmd if t0 <= c[0] <= t1]
        if not samples:
            return 0.0, 0.0, 0.0, False, False
        moving = [v for (_t, v, _w) in samples if abs(v) >= 0.01]
        steady = (
            len(moving) >= 0.9 * len(samples)
            and (all(v > 0 for v in moving) or all(v < 0 for v in moving))
        )
        turning = [w for (_t, _v, w) in samples if abs(w) >= 0.05]
        turn_steady = (
            len(turning) >= 0.9 * len(samples)
            and (all(w > 0 for w in turning) or all(w < 0 for w in turning))
        )
        dist = turn = signed = 0.0
        prev_t = t0
        for i, (t, v, w) in enumerate(samples):
            nxt = samples[i + 1][0] if i + 1 < len(samples) else t1
            # A command older than 0.5 s is not being repeated: the base's
            # own cmd_vel timeout stops the motors.
            dt = max(0.0, min(nxt, t + 0.5) - max(t, prev_t))
            dist += abs(v) * dt
            signed += v * dt
            turn += abs(w) * dt
            prev_t = t
        return dist, turn, (math.copysign(1.0, signed) if signed else 0.0), steady, turn_steady

    def _lidar_fraction(self, ref_pts, cur_pts, tx, ty, dyaw):
        """Fraction s in [0, 1] of the wheel translation the scans support.

        Returns (s, gain) where gain is how much better the best s fits than
        the worse end of the range; small gain means the scan cannot tell.
        """
        res, half = 0.02, 4.0
        size = int(2 * half / res)
        grid = np.full((size, size), 255, dtype=np.uint8)
        ij = np.floor((ref_pts + half) / res).astype(int)
        ok = (ij >= 0).all(axis=1) & (ij < size).all(axis=1)
        if np.count_nonzero(ok) < 40 or len(cur_pts) < 40:
            return None, 0.0
        grid[ij[ok, 1], ij[ok, 0]] = 0
        dist = np.minimum(cv2.distanceTransform(grid, cv2.DIST_L2, 5) * res, 0.25)
        c, s_ = math.cos(dyaw), math.sin(dyaw)
        rotated = cur_pts @ np.array([[c, s_], [-s_, c]])
        costs = []
        fractions = np.linspace(0.0, 1.0, 6)
        for f in fractions:
            moved = rotated + np.array([f * tx, f * ty])
            k = np.floor((moved + half) / res).astype(int)
            inside = (k >= 0).all(axis=1) & (k < size).all(axis=1)
            if np.count_nonzero(inside) < 30:
                return None, 0.0
            costs.append(float(np.mean(dist[k[inside, 1], k[inside, 0]])))
        costs = np.asarray(costs)
        best = int(np.argmin(costs))
        return float(fractions[best]), float(max(costs[0], costs[-1]) - costs[best])

    def _check(self, t_scan: float) -> None:
        if self._reflex is not None:
            return  # its own commands are not evidence of being stuck
        now = self._now()
        t0 = t_scan - self._window
        ref = next((s for s in self._scans if s[0] >= t0), None)
        if ref is None or t_scan - ref[0] < 0.6 * self._window:
            return
        w_ref, w_cur = _interp(self._wheel, ref[0]), _interp(self._wheel, t_scan)
        y_ref, y_cur = _interp(self._imu, ref[0]), _interp(self._imu, t_scan)
        if w_ref is None or w_cur is None:
            return
        span = t_scan - ref[0]
        cmd_dist, cmd_turn, cmd_dir, cmd_steady, turn_steady = self._commanded(now - span, now)

        # Wheel motion in the reference pose's frame.
        dx, dy = w_cur[0] - w_ref[0], w_cur[1] - w_ref[1]
        cr, sr = math.cos(w_ref[2]), math.sin(w_ref[2])
        tx, ty = cr * dx + sr * dy, -sr * dx + cr * dy
        wheel_travel = math.hypot(tx, ty)
        gyro_turn = abs(_wrap(y_cur - y_ref)) if y_ref is not None and y_cur is not None else None
        dyaw = _wrap(y_cur - y_ref) if gyro_turn is not None else _wrap(w_cur[2] - w_ref[2])

        info = {'cmd_dist': round(cmd_dist, 3), 'cmd_turn': round(cmd_turn, 3), 'cmd_steady': cmd_steady,
                'wheel_travel': round(wheel_travel, 3),
                'gyro_turn': None if gyro_turn is None else round(gyro_turn, 3)}

        stalled = (cmd_steady and cmd_dist >= self._min_travel
                   and wheel_travel < self._frac * cmd_dist)

        slipping = False
        # Matching scans for translation needs the heading to hold still: a
        # turn of a few tenths of a radian moves every far point more than
        # the translation being tested, and a slip read during a turn put a
        # false bump in front of the robot.
        if wheel_travel >= self._min_travel and abs(dyaw) < 0.3:
            frac, gain = self._lidar_fraction(ref[1], self._scans[-1][1], tx, ty, dyaw)
            vo = [v for (t, v) in self._vo if ref[0] <= t <= t_scan]
            vo_travel = abs(sum(vo)) * span / len(vo) if vo else None
            vo_covered = vo and len(vo) >= 0.5 * span * 4.0  # VO runs ~6 Hz
            lidar_still = frac is not None and frac <= self._frac and gain >= self._lidar_margin
            lidar_moving = frac is not None and frac >= 0.6 and gain >= self._lidar_margin
            vo_still = vo_covered and vo_travel < self._frac * wheel_travel
            vo_moving = vo_covered and vo_travel >= 0.6 * wheel_travel
            slipping = (lidar_still and not vo_moving) or (vo_still and not lidar_moving)
            info.update({'lidar_fraction': frac, 'lidar_gain': round(gain, 4),
                         'vo_travel': None if vo_travel is None else round(vo_travel, 3)})

        not_turning = (
            turn_steady and gyro_turn is not None and cmd_turn >= self._min_turn
            and cmd_dist < self._min_travel and gyro_turn < self._frac * cmd_turn
        )

        moving = (cmd_dist < 0.02 and cmd_turn < 0.1) or (
            not stalled and not slipping and not not_turning
            and (wheel_travel >= self._min_travel or (gyro_turn or 0.0) >= 0.2))
        for name, hit in (('stalled', stalled), ('slipping', slipping), ('not_turning', not_turning)):
            self._streak[name] = self._streak[name] + 1 if hit else 0
        self._last_check = info

        fired = [n for n, c in self._streak.items() if c == self._confirm]
        if fired:
            self._declare(fired, cmd_dir, info)
        elif self._stuck and moving:
            self._stuck = False
            self.get_logger().info('no longer stuck')
            self._publish_state()

    # --- outputs --------------------------------------------------------
    def _declare(self, reasons, cmd_dir, info) -> None:
        self._stuck = True
        now = self._now()
        bump = None
        # A bump is placed only for a steady push in one direction. In a
        # "collision ahead" loop Nav2 sends short start/stop commands, the
        # wheels jerk and slip-like readings appear with nothing touched; four
        # of five bumps in one fleet run came from such windows and walled the
        # robot in where it had stopped.
        if ('stalled' in reasons or 'slipping' in reasons) and cmd_dir != 0.0 \
                and info.get('cmd_steady') and now - self._last_bump >= self._bump_cooldown:
            bump = 'front' if cmd_dir > 0 else 'rear'
            self._publish_bump(cmd_dir)
            self._last_bump = now
            self._start_reflex(-cmd_dir, now)
        elif 'not_turning' in reasons or (
                not info.get('cmd_steady') and info.get('cmd_turn', 0.0) >= self._min_turn):
            self._turn_stalled(now)
        self.get_logger().warning(f'stuck: {"+".join(reasons)} bump={bump} {info}')
        evt = String()
        evt.data = json.dumps(dict(info, reasons=reasons, bump=bump, t=round(now, 3)))
        self._event_pub.publish(evt)
        self._publish_state()

    # --- bumper reflex ----------------------------------------------------
    def _strip_clear(self, direction: float, distance: float) -> bool:
        """True when the latest scan has nothing in the strip the body sweeps."""
        if not self._scans:
            return False
        pts = self._scans[-1][1]
        reach = distance + self._reflex_margin
        if direction < 0:
            near, far = -self._body_rear, -self._body_rear - reach
            in_x = (pts[:, 0] <= near) & (pts[:, 0] >= far)
        else:
            near, far = self._body_front, self._body_front + reach
            in_x = (pts[:, 0] >= near) & (pts[:, 0] <= far)
        return not bool(np.any(in_x & (np.abs(pts[:, 1]) <= self._body_half)))

    def _turn_stalled(self, now: float) -> None:
        self._turn_stalls.append(now)
        while self._turn_stalls and now - self._turn_stalls[0] > self._turn_stall_window:
            self._turn_stalls.popleft()
        if len(self._turn_stalls) < self._turn_stall_events:
            return
        self._turn_stalls.clear()
        dist = self._turn_reflex_dist
        direction = -1.0 if self._strip_clear(-1.0, dist) else 1.0
        self.get_logger().warning('turn keeps stalling: freeing the wheels')
        self._start_reflex(direction, now, dist)

    def _start_reflex(self, direction: float, now: float, distance: float = None) -> bool:
        if not self._reflex_enabled or self._reflex is not None or not self._wheel:
            return False
        if now - self._last_reflex < self._reflex_cooldown:
            return False
        dist = self._reflex_dist if distance is None else float(distance)
        side = 'back' if direction < 0 else 'forward'
        if not self._strip_clear(direction, dist):
            self.get_logger().warning(f'bumper reflex: LiDAR shows something {side}; not moving')
            return False
        x, y, _ = self._wheel[-1][1]
        self._reflex = {'dir': direction, 'start': (x, y), 't0': now, 'dist': dist}
        self._last_reflex = now
        self._reflex_pub.publish(Bool(data=True))
        self.get_logger().warning(f'bumper reflex: driving {side} {dist:.2f} m')
        return True

    def _reflex_tick(self) -> None:
        r = self._reflex
        if r is None:
            return
        now = self._now()
        x, y, _ = self._wheel[-1][1]
        done = math.hypot(x - r['start'][0], y - r['start'][1])
        reason = None
        if done >= r['dist']:
            reason = f'done ({done:.2f} m)'
        elif now - r['t0'] > self._reflex_timeout:
            reason = f'timed out after {done:.2f} m'
        elif not self._strip_clear(r['dir'], max(0.0, r['dist'] - done)):
            reason = f'LiDAR blocked after {done:.2f} m'
        if reason is not None:
            for _ in range(3):
                self._cmd_pub.publish(Twist())
            self._reflex = None
            self._reflex_pub.publish(Bool(data=False))
            self.get_logger().info(f'bumper reflex: {reason}')
            return
        cmd = Twist()
        cmd.linear.x = r['dir'] * self._reflex_speed
        self._cmd_pub.publish(cmd)

    def _publish_bump(self, direction: float) -> None:
        x = self._bump_front if direction > 0 else -self._bump_rear
        ys = np.arange(-self._bump_half, self._bump_half + 1e-6, 0.03)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._base_frame
        self._bump_pub.publish(point_cloud2.create_cloud_xyz32(header, [[x, float(y), 0.02] for y in ys]))

    def _publish_state(self) -> None:
        self._stuck_pub.publish(Bool(data=self._stuck))
        dbg = String()
        dbg.data = json.dumps(dict(self._last_check, stuck=self._stuck, streak=self._streak))
        self._debug_pub.publish(dbg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StuckDetector()
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
