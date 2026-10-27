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
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
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
        # A contact is marked wider than the robot: the object is usually
        # wider than the patch the bumper touched, and an 18 cm line let Nav2
        # plan around its end and hit a large low box again 12 cm further on.
        # Two rows deep (bump_depth_m) so the planner sees a solid edge.
        self.declare_parameter('bump_half_width_m', 0.18)
        self.declare_parameter('bump_depth_m', 0.05)
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
        # Long enough for Nav2 to replan around a new bump before the next
        # reflex. At 10 s a robot caught on a chair leg spent most of each
        # attempt waiting.
        self.declare_parameter('reflex_cooldown_sec', 4.0)
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
        # One confirmed stall is enough: the gyro showing under a third of a
        # half-radian turn is unambiguous, and waiting for a second one kept a
        # wheel against a chair caster for 2-5 s per attempt. The stalled turn
        # also leaves a small bump ahead of the wheel that was driving forward
        # (a clockwise turn drives the left wheel forward), the side a caster
        # beside a front wheel stops it from, so Nav2 stops repeating that turn.
        # The reflex distance takes that bump outside the global costmap's
        # 0.18 m robot radius, so Nav2 can plan again from where it stops.
        self.declare_parameter('turn_stall_events', 1)
        self.declare_parameter('turn_stall_window_sec', 8.0)
        self.declare_parameter('turn_reflex_distance_m', 0.15)
        self.declare_parameter('turn_bump_x_m', (0.08, 0.13))
        self.declare_parameter('turn_bump_y_m', (0.07, 0.10, 0.13))
        # Remembered low obstacles near the robot (low_obstacle_memory, base
        # frame). Reflexes and the clearance escape check them as well as the
        # LiDAR, which passes over casters and chair arms.
        self.declare_parameter('memory_topic', 'low_obstacle_points_local')
        self.declare_parameter('memory_max_age_sec', 5.0)
        # Clearance escape. Nav2's planner (Smac 2D) refuses to start while
        # the robot's own cell is inscribed, i.e. anything is within the
        # global costmap's robot_radius (0.18 m) of the axle. The controller
        # lets the robot stop closer than that (0.15 m), and the Burger's
        # front edge is only 0.04 m ahead of the axle, so a stop facing a
        # box or a wall left Nav2 unable to plan from where it stood; nothing
        # moved it, and the explorer blacklisted goal after goal. With no
        # motion command for clearance_idle_sec and a LiDAR return within
        # clearance_radius_m, the robot moves straight forward or back the
        # shortest distance (at most clearance_max_m) that clears it, through
        # a strip the LiDAR shows empty. The planner compares 5 cm cells, so
        # a return 0.225 m away can sit in a cell 0.19 m from the robot's own
        # (0.18 m plus two half-cell diagonals is ~0.25 m); at 0.21 m Inky
        # stayed in "Starting point in lethal space" for four minutes.
        # When the obstacle runs alongside and no straight move clears it,
        # the robot first turns to face away from it (only when nothing is
        # within clearance_turn_min_m, the body's turning radius plus a
        # margin), then makes the straight move.
        self.declare_parameter('clearance_escape_enabled', True)
        self.declare_parameter('clearance_radius_m', 0.26)
        self.declare_parameter('clearance_idle_sec', 3.0)
        self.declare_parameter('clearance_max_m', 0.15)
        self.declare_parameter('clearance_turn_min_m', 0.16)
        self.declare_parameter('turn_speed', 0.6)
        # Plans that keep failing while the robot stands still: clear the
        # global costmap once. Its obstacle layer keeps LiDAR marks until a
        # beam passes through them, so a chair moved or a person's legs seen
        # where the LiDAR cannot look again stay forever; on Inky they closed
        # every way out of a spot the robot had driven into, while the SLAM
        # map alone left it open. Nav2's own navigation clears the costmap
        # on failure, but the central explorer's path checks call the planner
        # directly. After plan_fail_clear_count aborted plans within
        # plan_fail_window_sec, idle, the global costmap is cleared (at most
        # once per plan_fail_cooldown_sec); LiDAR, stereo and the low-obstacle
        # memory re-mark what is really there within a second.
        self.declare_parameter('plan_fail_clear_count', 4)
        self.declare_parameter('plan_fail_window_sec', 30.0)
        # 30 s left Inky boxed in among chair legs for ~35 s per episode
        # before the squeeze below started; a clear is cheap (the sensors
        # re-mark within a second), so the squeeze may follow 12 s after.
        self.declare_parameter('plan_fail_cooldown_sec', 12.0)
        # Squeeze. The global planner keeps squeeze_normal_radius_m (0.18 m)
        # from obstacles, the controller only 0.15 m, so the robot can drive
        # through a gap the planner then will not route back through. On Inky
        # it entered a room through a doorway with exactly 0.18 m clearance
        # and stayed: every plan out failed, and clearing the costmap changed
        # nothing (the doorway was in the SLAM map). When plans keep failing
        # within squeeze_after_clear_sec of a clear, the global costmap's
        # robot_radius is set to squeeze_radius_m (the controller's) until the
        # robot has moved squeeze_release_m or squeeze_sec has passed.
        self.declare_parameter('squeeze_enabled', True)
        self.declare_parameter('squeeze_radius_m', 0.15)
        self.declare_parameter('squeeze_normal_radius_m', 0.18)
        self.declare_parameter('squeeze_after_clear_sec', 90.0)
        self.declare_parameter('squeeze_sec', 60.0)
        self.declare_parameter('squeeze_release_m', 0.8)
        self.declare_parameter('global_costmap_node', 'global_costmap/global_costmap')
        # Tilt escape. A Burger that drives onto a chair leg or a cable ends
        # up with one end raised; on Inky, a chair leg under the rear caster
        # left it 11 deg nose-down, and three back-off reflexes each moved
        # 2-5 cm because backing drove further onto the leg. When the
        # accelerometer shows more than tilt_threshold_deg for tilt_hold_sec,
        # the robot drives toward its lower end (forward when nose-down, back
        # when nose-up) by tilt_escape_m and leaves a bump at the raised end
        # so the planner keeps away from the object afterwards.
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('tilt_threshold_deg', 5.0)
        self.declare_parameter('tilt_hold_sec', 1.0)
        self.declare_parameter('tilt_escape_m', 0.15)
        # Only when the robot is held: the wheels moved less than this while
        # it stayed tilted. Rolling over a floor cable tilts the robot just
        # as much, and backing off then left a bump wall across the cable.
        self.declare_parameter('tilt_max_travel_m', 0.03)
        # A stuck state that lasts this long gets a reflex even if no second
        # stall event arrives: a wheel caught on a chair leg mid-turn was
        # declared stuck once and then left for 32 s.
        self.declare_parameter('stuck_reflex_after_sec', 4.0)
        self.declare_parameter('stuck_reflex_distance_m', 0.15)

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
        self._bump_depth = max(0.0, float(p('bump_depth_m')))
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
        self._turn_bump_x = [float(v) for v in p('turn_bump_x_m')]
        self._turn_bump_y = [float(v) for v in p('turn_bump_y_m')]
        self._turn_stalls = deque()
        self._memory_max_age = float(p('memory_max_age_sec'))
        # (receipt time, Nx2 points in the base frame, wheel pose they are relative to)
        self._memory = None
        self._clearance_enabled = bool(p('clearance_escape_enabled'))
        self._clearance_radius = float(p('clearance_radius_m'))
        self._clearance_turn_min = float(p('clearance_turn_min_m'))
        self._turn_speed = abs(float(p('turn_speed')))
        self._clearance_idle = float(p('clearance_idle_sec'))
        self._clearance_max = float(p('clearance_max_m'))
        self._last_motion_cmd = self.get_clock().now().nanoseconds * 1e-9
        self._tilt_threshold = math.radians(float(p('tilt_threshold_deg')))
        self._tilt_hold = float(p('tilt_hold_sec'))
        self._tilt_escape = float(p('tilt_escape_m'))
        self._accel = deque(maxlen=200)   # (t, ax, ay, az)
        self._tilted_since = None
        self._tilt_start_pose = None
        self._tilt_max_travel = float(p('tilt_max_travel_m'))
        # Direction and distance covered by the last reflex; a reflex that
        # barely moved means that way is held, so the next one tries the other.
        self._last_reflex_result = None
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
        self._stuck_since = None
        self._stuck_reflex_after = float(p('stuck_reflex_after_sec'))
        self._stuck_reflex_dist = float(p('stuck_reflex_distance_m'))
        self._last_bump = 0.0
        self._last_check = {}

        self.create_subscription(Twist, str(p('cmd_vel_topic')), self._on_cmd, 20)
        self.create_subscription(Odometry, str(p('wheel_odom_topic')), self._on_wheel, 20)
        # The heading comes from the IMU's orientation (the same yaw the EKF
        # follows). odom stands in for odom/wheel only when that topic is
        # silent (use_ekf:=false); it is subscribed just then, because each
        # 30 Hz topic received in Python costs CPU on the Pi.
        self._last_wheel_msg = -1e9
        self._odom_sub = None
        self._odom_check = self.create_timer(3.0, self._maybe_subscribe_odom)
        self.create_subscription(Odometry, str(p('visual_odom_topic')), self._on_vo, 10)
        self.create_subscription(LaserScan, str(p('scan_topic')), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, str(p('imu_topic')), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, str(p('memory_topic')), self._on_memory, 5)
        self._stuck_pub = self.create_publisher(Bool, 'stuck', 10)
        self._event_pub = self.create_publisher(String, 'stuck/event', 10)
        self._debug_pub = self.create_publisher(String, 'stuck/debug', 10)
        self._bump_pub = self.create_publisher(PointCloud2, 'bump_points', 10)
        self._cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic')), 10)
        self._reflex_pub = self.create_publisher(Bool, 'stuck/reflex_active', 10)
        self._lethal_pub = self.create_publisher(Bool, 'nav2_lethal_inflation', 10)
        # The 40 Hz reflex timer exists only while a reflex drives: every
        # wake-up of rclpy's executor costs CPU on the Pi.
        self._reflex_period = 1.0 / max(5.0, float(p('reflex_rate_hz')))
        self._reflex_timer = None
        self._scan_count = 0
        self.create_timer(1.0, self._publish_state)
        self.create_timer(1.0, self._clearance_tick)
        self._plan_fail_count = max(1, int(p('plan_fail_clear_count')))
        self._plan_fail_window = float(p('plan_fail_window_sec'))
        self._plan_fail_cooldown = float(p('plan_fail_cooldown_sec'))
        self._plan_failures = deque()
        self._plan_seen = set()
        self._last_costmap_clear = -1e9
        self.create_subscription(
            GoalStatusArray, 'compute_path_to_pose/_action/status', self._on_plan_status, 10)
        self._clear_cli = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        self._squeeze_enabled = bool(p('squeeze_enabled'))
        self._squeeze_radius = float(p('squeeze_radius_m'))
        self._normal_radius = float(p('squeeze_normal_radius_m'))
        self._squeeze_after = float(p('squeeze_after_clear_sec'))
        self._squeeze_sec = float(p('squeeze_sec'))
        self._squeeze_release = float(p('squeeze_release_m'))
        self._squeeze = None  # {'t0', 'pose'} while the radius is lowered
        self._radius_cli = self.create_client(
            SetParameters, f"{str(p('global_costmap_node')).strip('/')}/set_parameters")
        self.create_timer(1.0, self._squeeze_tick)
        self.create_timer(0.5, self._tilt_tick)
        self.get_logger().info('stuck detector: cmd_vel vs wheels, wheels vs LiDAR/VO, turn vs gyro')

    # --- inputs ---------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        now = self._now()
        self._cmd.append((now, float(msg.linear.x), float(msg.angular.z)))
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self._last_motion_cmd = now

    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        self._accel.append((self._now(), float(a.x), float(a.y), float(a.z)))
        self._imu.append((_stamp(msg), _yaw(msg.orientation)))

    def _maybe_subscribe_odom(self) -> None:
        self._odom_check.cancel()
        if self._now() - self._last_wheel_msg > 2.0 and self._odom_sub is None:
            self.get_logger().info('odom/wheel is silent; using odom for wheel motion')
            self._odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 20)

    def _tilt(self):
        """(tilt rad, forward component of gravity) averaged over the last second, or None.

        The accelerometer measures the reaction to gravity, so at rest it
        points up in the body frame. Its x part is negative when the nose is
        lower than the tail."""
        now = self._now()
        recent = [(ax, ay, az) for (t, ax, ay, az) in self._accel if now - t <= 1.0]
        if len(recent) < 5:
            return None
        ax, ay, az = (float(np.mean(v)) for v in zip(*recent))
        return math.atan2(math.hypot(ax, ay), az), ax

    def _on_odom(self, msg: Odometry) -> None:
        if self._now() - self._last_wheel_msg > 1.0:
            self._store_wheel(msg)

    def _on_wheel(self, msg: Odometry) -> None:
        self._last_wheel_msg = self._now()
        self._store_wheel(msg)

    def _store_wheel(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._wheel.append((_stamp(msg), (p.position.x, p.position.y, _yaw(p.orientation))))

    def _on_memory(self, msg: PointCloud2) -> None:
        # Vectorized: a per-point Python loop over up to 4000 cells each second.
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y'), skip_nans=True).astype(float).reshape(-1, 2)
        t = _stamp(msg)
        ref = _interp(self._wheel, t)
        if ref is None and self._wheel and abs(self._wheel[-1][0] - t) < 0.5:
            ref = self._wheel[-1][1]
        self._memory = None if ref is None else (self._now(), pts, ref)

    def _memory_points(self) -> np.ndarray:
        """Remembered low obstacles near the robot, moved into the current base frame by wheel odometry."""
        m = self._memory
        if m is None or not self._wheel or len(m[1]) == 0 \
                or self._now() - m[0] > self._memory_max_age:
            return np.zeros((0, 2))
        (x0, y0, a0), (x1, y1, a1) = m[2], self._wheel[-1][1]
        c0, s0 = math.cos(a0), math.sin(a0)
        dx, dy = c0 * (x1 - x0) + s0 * (y1 - y0), -s0 * (x1 - x0) + c0 * (y1 - y0)
        c, s = math.cos(a1 - a0), math.sin(a1 - a0)
        p = m[1] - np.array([dx, dy])
        return np.stack([c * p[:, 0] + s * p[:, 1], -s * p[:, 0] + c * p[:, 1]], axis=1)

    def _obstacles(self):
        """(LiDAR points, remembered points) in the base frame; empty arrays when missing."""
        lidar = self._scans[-1][1] if self._scans else np.zeros((0, 2))
        memory = self._memory_points()
        # Cells under the body are left out of the costmap too (keep_clear_radius_m).
        memory = memory[np.hypot(memory[:, 0], memory[:, 1]) > 0.12]
        return lidar, memory

    def _on_vo(self, msg: Odometry) -> None:
        self._vo.append((_stamp(msg), float(msg.twist.twist.linear.x)))

    def _on_scan(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment + self._lidar_yaw
        ok = np.isfinite(r) & (r > max(0.12, msg.range_min)) & (r < min(4.0, msg.range_max))
        pts = np.stack([r[ok] * np.cos(a[ok]) + self._lidar_x, r[ok] * np.sin(a[ok])], axis=1)
        t = _stamp(msg)
        self._scans.append((t, pts))
        # Every other scan (5 Hz) is plenty for 2 s windows and halves the
        # cost of the LiDAR match while driving.
        self._scan_count += 1
        if self._scan_count % 2 == 0:
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

        turn_sum = sum(w for (t, _v, w) in self._cmd if now - span <= t <= now)
        info = {'cmd_dist': round(cmd_dist, 3), 'cmd_turn': round(cmd_turn, 3), 'cmd_steady': cmd_steady,
                'turn_dir': 0 if abs(turn_sum) < 1e-6 else int(math.copysign(1, turn_sum)),
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
            self._stuck_since = None
            self.get_logger().info('no longer stuck')
            self._publish_state()
        elif self._stuck and self._stuck_since is not None \
                and now - self._stuck_since >= self._stuck_reflex_after:
            self._stuck_since = now  # next attempt only after another full wait
            dist = self._stuck_reflex_dist
            tilt = self._tilt()
            if tilt is not None and tilt[0] >= self._tilt_threshold:
                preferred = 1.0 if tilt[1] < 0.0 else -1.0
            elif cmd_dir != 0.0 and cmd_dist >= self._min_travel:
                preferred = -cmd_dir          # away from where it was pushing
            else:
                preferred = -1.0 if self._strip_clear(-1.0, dist) else 1.0
            direction = self._pick_direction(preferred)
            self.get_logger().warning(
                f'still stuck after {self._stuck_reflex_after:.0f} s: freeing the wheels')
            self._start_reflex(direction, now, dist)

    def _on_plan_status(self, msg: GoalStatusArray) -> None:
        now = self._now()
        for st in msg.status_list:
            if st.status != GoalStatus.STATUS_ABORTED:
                continue
            key = bytes(st.goal_info.goal_id.uuid)
            if key in self._plan_seen:
                continue
            self._plan_seen.add(key)
            self._plan_failures.append(now)
        if len(self._plan_seen) > 500:
            self._plan_seen = set()
        while self._plan_failures and now - self._plan_failures[0] > self._plan_fail_window:
            self._plan_failures.popleft()
        if len(self._plan_failures) < self._plan_fail_count:
            return
        if now - self._last_motion_cmd < self._clearance_idle or self._reflex is not None:
            return
        if now - self._last_costmap_clear < self._plan_fail_cooldown:
            return
        if not self._clear_cli.service_is_ready():
            return
        squeeze = (
            self._squeeze_enabled and self._squeeze is None
            and now - self._last_costmap_clear < self._squeeze_after
            and self._wheel and self._set_global_radius(self._squeeze_radius))
        self._last_costmap_clear = now
        self._plan_failures.clear()
        self._clear_cli.call_async(ClearEntireCostmap.Request())
        if squeeze:
            self._squeeze = {'t0': now, 'pose': self._wheel[-1][1][:2]}
            self.get_logger().warning(
                'plans still fail after clearing the costmap: global robot_radius '
                f'{self._normal_radius:.2f} -> {self._squeeze_radius:.2f} m (the controller\'s) '
                f'until the robot has moved {self._squeeze_release:.1f} m or {self._squeeze_sec:.0f} s')
            return
        self.get_logger().warning(
            f'{self._plan_fail_count}+ plans failed in {self._plan_fail_window:.0f} s with the robot '
            'standing still: clearing the global costmap (stale LiDAR marks)')

    def _set_global_radius(self, radius: float) -> bool:
        if not self._radius_cli.service_is_ready():
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name='robot_radius', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(radius)))]
        self._radius_cli.call_async(req)
        return True

    def _squeeze_tick(self) -> None:
        sq = self._squeeze
        if sq is None or not self._wheel:
            return
        x, y, _ = self._wheel[-1][1]
        moved = math.hypot(x - sq['pose'][0], y - sq['pose'][1])
        age = self._now() - sq['t0']
        if moved < self._squeeze_release and age < self._squeeze_sec:
            return
        if self._set_global_radius(self._normal_radius):
            self._squeeze = None
            self.get_logger().info(
                f'global robot_radius back to {self._normal_radius:.2f} m '
                f'(moved {moved:.2f} m in {age:.0f} s)')

    # --- outputs --------------------------------------------------------
    def _declare(self, reasons, cmd_dir, info) -> None:
        now = self._now()
        if not self._stuck:
            self._stuck_since = now
        self._stuck = True
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
            # Only a clean not-turning reading says which way the turn was held.
            turn_dir = info.get('turn_dir', 0) if 'not_turning' in reasons else 0
            if self._turn_stalled(now, turn_dir):
                bump = 'right wheel' if info.get('turn_dir', 0) > 0 else 'left wheel'
        self.get_logger().warning(f'stuck: {"+".join(reasons)} bump={bump} {info}')
        evt = String()
        evt.data = json.dumps(dict(info, reasons=reasons, bump=bump, t=round(now, 3)))
        self._event_pub.publish(evt)
        self._publish_state()

    # --- bumper reflex ----------------------------------------------------
    def _strip_clear(self, direction: float, distance: float) -> bool:
        """True when neither the latest scan nor the memory has anything in the strip the body sweeps."""
        if not self._scans:
            return False
        lidar, memory = self._obstacles()
        reach = distance + self._reflex_margin
        if direction < 0:
            near, far = -self._body_rear, -self._body_rear - reach
        else:
            near, far = self._body_front, self._body_front + reach
        lo, hi = min(near, far), max(near, far)
        # Remembered cells are 5 cm squares given by their centres.
        for pts, grow in ((lidar, 0.0), (memory, 0.025)):
            if len(pts) and bool(np.any(
                    (pts[:, 0] >= lo - grow) & (pts[:, 0] <= hi + grow)
                    & (np.abs(pts[:, 1]) <= self._body_half + grow))):
                return False
        return True

    def _clearance_tick(self, chained: bool = False) -> None:
        """chained: called right after the escape's own turn, skipping the idle and cooldown waits."""
        if not self._scans:
            return
        now = self._now()
        idle = now - self._last_motion_cmd >= self._clearance_idle
        # Remembered cells count too: a bump or a caster mark beside the robot
        # left Nav2 failing "Starting point in lethal space" for over a minute
        # with nothing near the robot in the LiDAR.
        lidar, memory = self._obstacles()
        pts = np.vstack([lidar, memory]) if len(memory) else lidar
        dist = np.hypot(pts[:, 0], pts[:, 1]) if len(pts) else np.zeros(0)
        near = float(np.min(dist)) if len(dist) else np.inf
        # nav2_lethal_inflation for the central explorer: standing where the
        # planner cannot start. While it is true the explorer holds the goal
        # instead of blacklisting every goal that fails to plan; in one run
        # four minutes of this blacklisted every frontier and exploration
        # ended with six left. Only while idle, so passing close to furniture
        # does not hold anything.
        self._lethal_pub.publish(Bool(data=bool(
            idle and self._reflex is None and near < self._clearance_radius)))
        if not self._clearance_enabled or self._reflex is not None:
            return
        if not chained:
            if not idle:
                return
            if now - self._last_reflex < self._reflex_cooldown:
                return
        if near >= self._clearance_radius:
            return
        source = 'remembered low obstacle' if len(memory) and near < float(
            np.min(np.hypot(lidar[:, 0], lidar[:, 1])) if len(lidar) else np.inf) else 'LiDAR return'
        # Shortest straight move, either way, after which everything is
        # clearance_radius_m away, through a strip that is empty.
        best = None
        for step in np.arange(0.02, self._clearance_max + 1e-6, 0.01):
            for direction in (-1.0, 1.0):
                moved = np.hypot(pts[:, 0] - direction * step, pts[:, 1])
                if float(np.min(moved)) >= self._clearance_radius and self._strip_clear(direction, step):
                    best = (direction, float(step))
                    break
            if best is not None:
                break
        if best is None:
            if chained:
                return
            # Alongside: turn to face away from what is close, then move.
            close = pts[dist < self._clearance_radius]
            away = math.atan2(-float(np.mean(close[:, 1])), -float(np.mean(close[:, 0])))
            if near < self._clearance_turn_min or abs(away) < 0.35:
                return
            self.get_logger().warning(
                f'too close to stand and plan: {source} {near:.2f} m from the axle, alongside; '
                f'turning {math.degrees(away):+.0f} deg to move away from it')
            self._start_turn(away, now)
            return
        self.get_logger().warning(
            f'too close to stand and plan: {source} {near:.2f} m from the axle '
            f'(Nav2 needs {self._clearance_radius:.2f}); moving clear')
        if chained:
            self._last_reflex = now - self._reflex_cooldown  # the turn was part of this escape
        self._start_reflex(best[0], now, best[1])

    def _start_turn(self, angle: float, now: float) -> None:
        """Turn in place by angle (rad) on wheel odometry, then try the straight clearance move."""
        if not self._reflex_enabled or self._reflex is not None or not self._wheel:
            return
        yaw0 = self._wheel[-1][1][2]
        self._reflex = {'turn': True, 'target': yaw0 + angle, 'dir': math.copysign(1.0, angle),
                        't0': now, 'start': self._wheel[-1][1][:2], 'dist': 0.0}
        if self._reflex_timer is None:
            self._reflex_timer = self.create_timer(self._reflex_period, self._reflex_tick)
        else:
            self._reflex_timer.reset()
        self._last_reflex = now
        self._reflex_pub.publish(Bool(data=True))

    def _tilt_tick(self) -> None:
        tilt = self._tilt()
        now = self._now()
        if tilt is None or tilt[0] < self._tilt_threshold:
            self._tilted_since = None
            return
        pose = self._wheel[-1][1] if self._wheel else None
        if self._tilted_since is None:
            self._tilted_since = now
            self._tilt_start_pose = pose
            return
        if pose is not None and self._tilt_start_pose is not None and math.hypot(
                pose[0] - self._tilt_start_pose[0], pose[1] - self._tilt_start_pose[1]) > self._tilt_max_travel:
            # Still moving while tilted: driving over something, not held by it.
            self._tilted_since = now
            self._tilt_start_pose = pose
            return
        if self._reflex is not None or now - self._tilted_since < self._tilt_hold:
            return
        if now - self._last_reflex < self._reflex_cooldown:
            return
        angle, forward_g = tilt
        downhill = 1.0 if forward_g < 0.0 else -1.0
        direction = self._pick_direction(downhill)
        end = 'nose-down' if forward_g < 0.0 else 'nose-up'
        self.get_logger().warning(
            f'tilted {math.degrees(angle):.0f} deg {end}: up on something; '
            f'driving {"forward" if direction > 0 else "back"} {self._tilt_escape:.2f} m')
        if now - self._last_bump >= self._bump_cooldown:
            # The raised end is on the object.
            self._publish_bump(-downhill)
            self._last_bump = now
        self._start_reflex(direction, now, self._tilt_escape)

    def _pick_direction(self, preferred: float) -> float:
        """preferred, unless the last reflex that way barely moved."""
        last = self._last_reflex_result
        if last is not None and last[0] == preferred and last[1] < 0.03 \
                and self._now() - last[2] < 60.0:
            return -preferred
        return preferred

    def _turn_stalled(self, now: float, turn_dir: int) -> bool:
        """Count a stalled turn; on enough of them free the wheels. True when a bump was placed."""
        self._turn_stalls.append(now)
        while self._turn_stalls and now - self._turn_stalls[0] > self._turn_stall_window:
            self._turn_stalls.popleft()
        if len(self._turn_stalls) < self._turn_stall_events:
            return False
        self._turn_stalls.clear()
        bumped = False
        if turn_dir != 0 and now - self._last_bump >= self._bump_cooldown:
            # Counter-clockwise drives the right wheel forward, clockwise the left.
            side = -1.0 if turn_dir > 0 else 1.0
            self._publish_points_base([(x, side * y) for x in self._turn_bump_x for y in self._turn_bump_y])
            self._last_bump = now
            bumped = True
        dist = self._turn_reflex_dist
        tilt = self._tilt()
        if tilt is not None and tilt[0] >= self._tilt_threshold:
            preferred = 1.0 if tilt[1] < 0.0 else -1.0   # toward the lower end
        else:
            preferred = -1.0 if self._strip_clear(-1.0, dist) else 1.0
        direction = self._pick_direction(preferred)
        self.get_logger().warning('turn stalled: freeing the wheels')
        self._start_reflex(direction, now, dist)
        return bumped

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
        if self._reflex_timer is None:
            self._reflex_timer = self.create_timer(self._reflex_period, self._reflex_tick)
        else:
            self._reflex_timer.reset()
        self._last_reflex = now
        self._reflex_pub.publish(Bool(data=True))
        self.get_logger().warning(f'bumper reflex: driving {side} {dist:.2f} m')
        return True

    def _reflex_tick(self) -> None:
        r = self._reflex
        if r is None:
            return
        now = self._now()
        if r.get('turn'):
            left = _wrap(r['target'] - self._wheel[-1][1][2])
            if abs(left) < 0.08 or now - r['t0'] > self._reflex_timeout:
                for _ in range(3):
                    self._cmd_pub.publish(Twist())
                self._reflex = None
                if self._reflex_timer is not None:
                    self._reflex_timer.cancel()
                self._reflex_pub.publish(Bool(data=False))
                self.get_logger().info(f'clearance turn: {"done" if abs(left) < 0.08 else "timed out"}')
                self._clearance_tick(chained=True)
                return
            cmd = Twist()
            # Slow down over the last 0.3 rad so the turn does not overshoot.
            cmd.angular.z = math.copysign(max(0.25, min(self._turn_speed, 2.0 * abs(left))), left)
            self._cmd_pub.publish(cmd)
            return
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
            if self._reflex_timer is not None:
                self._reflex_timer.cancel()
            self._last_reflex_result = (r['dir'], done, now)
            self._reflex_pub.publish(Bool(data=False))
            self.get_logger().info(f'bumper reflex: {reason}')
            return
        cmd = Twist()
        cmd.linear.x = r['dir'] * self._reflex_speed
        self._cmd_pub.publish(cmd)

    def _publish_bump(self, direction: float) -> None:
        x0 = self._bump_front if direction > 0 else -self._bump_rear
        sign = 1.0 if direction > 0 else -1.0
        ys = np.arange(-self._bump_half, self._bump_half + 1e-6, 0.03)
        xs = x0 + sign * np.arange(0.0, self._bump_depth + 1e-6, 0.05)
        self._publish_points_base([(x, y) for x in xs for y in ys])

    def _publish_points_base(self, xy) -> None:
        """Contact points (base frame) for low_obstacle_memory."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._base_frame
        self._bump_pub.publish(point_cloud2.create_cloud_xyz32(
            header, [[float(x), float(y), 0.02] for x, y in xy]))

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
