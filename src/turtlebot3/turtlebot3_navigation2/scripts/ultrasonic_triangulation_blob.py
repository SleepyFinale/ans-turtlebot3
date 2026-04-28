#!/usr/bin/env python3
"""
Triangulate overlapping ultrasonic detections into a compact obstacle blob scan.
"""

import json
import math
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
import tf2_ros


def _stamp_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class UltrasonicTriangulationBlob(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_triangulation_blob')

        self.declare_parameter('range_topics', ['ultrasonic_l', 'ultrasonic_f', 'ultrasonic_r'])
        self.declare_parameter('range_frame_ids', ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right'])
        self.declare_parameter('base_scan_frame', 'base_scan')
        self.declare_parameter('output_scan_topic', 'ultrasonic_blob_scan')
        self.declare_parameter('output_scan_topic_emergency', 'ultrasonic_blob_scan_emergency')
        self.declare_parameter('output_scan_topic_side', 'ultrasonic_blob_scan_side')
        self.declare_parameter('debug_topic', 'ultrasonic_triangulation_debug')
        self.declare_parameter('triangulation_enabled', True)
        self.declare_parameter('max_age_sec', 0.15)
        self.declare_parameter('min_valid_range_m', 0.02)
        self.declare_parameter('max_valid_range_m', 0.50)
        self.declare_parameter('similarity_m', 0.06)
        self.declare_parameter('similarity_scale_per_m', 0.18)
        self.declare_parameter('similarity_max_m', 0.14)
        self.declare_parameter('blob_radius_m', 0.08)
        self.declare_parameter('require_pair_agreement', True)
        self.declare_parameter('front_emergency_range_m', 0.38)
        self.declare_parameter('front_emergency_strict_range_m', 0.34)
        self.declare_parameter('front_emergency_required_streak', 1)
        self.declare_parameter('front_emergency_blob_radius_m', 0.16)
        self.declare_parameter('front_emergency_hold_sec', 1.20)
        self.declare_parameter('front_emergency_close_blob_dist_m', 0.34)
        self.declare_parameter('front_emergency_close_extra_hold_sec', 0.60)
        self.declare_parameter('front_emergency_close_radius_scale', 1.18)
        self.declare_parameter('front_emergency_cone_scale', 1.35)
        self.declare_parameter('front_emergency_head_on_angle_rad', 0.45)
        self.declare_parameter('front_emergency_head_on_cone_boost', 1.10)
        self.declare_parameter('front_emergency_head_on_cone_max', 1.62)
        self.declare_parameter('hard_block_duration_sec', 1.00)
        self.declare_parameter('memory_decay_duration_sec', 8.00)
        self.declare_parameter('memory_replay_inward_m', 0.08)
        self.declare_parameter('memory_replay_max_odom_travel_m', 0.32)
        self.declare_parameter('memory_replay_max_odom_yaw_rad', 0.90)
        self.declare_parameter('memory_replay_in_decay', False)
        self.declare_parameter('lateral_blob_cancel_min_odom_yaw_rad', 0.32)
        self.declare_parameter('lateral_blob_cancel_min_abs_blob_angle_rad', 0.48)
        self.declare_parameter('blob_hold_sec', 0.90)
        self.declare_parameter('publish_hz', 12.0)
        self.declare_parameter('scan_angle_min', -1.57)
        self.declare_parameter('scan_angle_max', 1.57)
        self.declare_parameter('scan_angle_increment', 0.0174533)

        self._range_topics = list(self.get_parameter('range_topics').value)
        self._range_frames = list(self.get_parameter('range_frame_ids').value)
        self._base_scan_frame = str(self.get_parameter('base_scan_frame').value).strip()
        self._triangulation_enabled = bool(self.get_parameter('triangulation_enabled').value)
        self._max_age_sec = max(0.05, float(self.get_parameter('max_age_sec').value))
        self._min_valid_range = max(0.0, float(self.get_parameter('min_valid_range_m').value))
        self._max_valid_range = max(0.05, float(self.get_parameter('max_valid_range_m').value))
        self._similarity_m = max(0.01, float(self.get_parameter('similarity_m').value))
        self._similarity_scale_per_m = max(
            0.0, float(self.get_parameter('similarity_scale_per_m').value)
        )
        self._similarity_max_m = max(
            self._similarity_m,
            float(self.get_parameter('similarity_max_m').value)
        )
        self._blob_radius_m = max(0.03, float(self.get_parameter('blob_radius_m').value))
        self._publish_hz = max(2.0, float(self.get_parameter('publish_hz').value))
        self._scan_angle_min = float(self.get_parameter('scan_angle_min').value)
        self._scan_angle_max = float(self.get_parameter('scan_angle_max').value)
        self._scan_angle_increment = max(1e-4, float(self.get_parameter('scan_angle_increment').value))
        self._require_pair_agreement = bool(self.get_parameter('require_pair_agreement').value)
        self._front_emergency_range_m = max(
            self._min_valid_range,
            min(self._max_valid_range, float(self.get_parameter('front_emergency_range_m').value))
        )
        _strict_raw = float(self.get_parameter('front_emergency_strict_range_m').value)
        self._front_emergency_strict_range_m = max(
            self._min_valid_range,
            min(self._front_emergency_range_m - 0.01, _strict_raw),
        )
        self._front_emergency_required_streak = max(
            1, int(self.get_parameter('front_emergency_required_streak').value)
        )
        self._front_emergency_blob_radius_m = max(
            self._blob_radius_m,
            float(self.get_parameter('front_emergency_blob_radius_m').value)
        )
        self._front_emergency_hold_sec = max(
            0.0, float(self.get_parameter('front_emergency_hold_sec').value)
        )
        self._front_emergency_close_blob_dist_m = max(
            self._min_valid_range,
            float(self.get_parameter('front_emergency_close_blob_dist_m').value),
        )
        self._front_emergency_close_extra_hold_sec = max(
            0.0, float(self.get_parameter('front_emergency_close_extra_hold_sec').value)
        )
        self._front_emergency_close_radius_scale = max(
            1.0, float(self.get_parameter('front_emergency_close_radius_scale').value)
        )
        self._front_emergency_cone_scale = max(
            1.0, float(self.get_parameter('front_emergency_cone_scale').value)
        )
        self._front_emergency_head_on_angle_rad = max(
            0.05, float(self.get_parameter('front_emergency_head_on_angle_rad').value)
        )
        self._front_emergency_head_on_cone_boost = max(
            1.0, float(self.get_parameter('front_emergency_head_on_cone_boost').value)
        )
        self._front_emergency_head_on_cone_max = max(
            self._front_emergency_cone_scale,
            float(self.get_parameter('front_emergency_head_on_cone_max').value),
        )
        self._hard_block_duration_sec = max(
            0.0, float(self.get_parameter('hard_block_duration_sec').value)
        )
        self._memory_decay_duration_sec = max(
            0.0, float(self.get_parameter('memory_decay_duration_sec').value)
        )
        self._blob_hold_sec = max(0.0, float(self.get_parameter('blob_hold_sec').value))
        self._memory_replay_inward_m = max(
            0.0, float(self.get_parameter('memory_replay_inward_m').value)
        )
        self._memory_replay_max_odom_travel_m = max(
            0.05, float(self.get_parameter('memory_replay_max_odom_travel_m').value)
        )
        self._memory_replay_max_odom_yaw_rad = max(
            0.10, float(self.get_parameter('memory_replay_max_odom_yaw_rad').value)
        )
        self._memory_replay_in_decay = bool(
            self.get_parameter('memory_replay_in_decay').value
        )
        self._lateral_blob_cancel_min_odom_yaw_rad = max(
            0.0, float(self.get_parameter('lateral_blob_cancel_min_odom_yaw_rad').value)
        )
        self._lateral_blob_cancel_min_abs_blob_angle_rad = max(
            0.0, float(self.get_parameter('lateral_blob_cancel_min_abs_blob_angle_rad').value)
        )

        # Keep namespaced frame lookup robust.
        if len(self._range_frames) != 3:
            self._range_frames = ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right']
        ns = self.get_namespace().strip('/')
        if ns and '/' not in self._base_scan_frame:
            self._base_scan_frame = f'{ns}/{self._base_scan_frame}'
        for i in range(3):
            f = str(self._range_frames[i]).strip()
            if ns and '/' not in f:
                self._range_frames[i] = f'{ns}/{f}'
            else:
                self._range_frames[i] = f

        self._last_msg: Dict[int, Optional[Range]] = {0: None, 1: None, 2: None}
        self._last_recv_s: Dict[int, Optional[float]] = {0: None, 1: None, 2: None}
        self._front_emergency_streak = 0
        self._last_blob = None
        self._last_blob_stamp_s = None
        self._last_hard_detection_s = None
        self._memory_odom_anchor: Optional[Tuple[float, float, float]] = None

        if '/' in self._base_scan_frame:
            _ns_prefix = self._base_scan_frame.split('/')[0]
            self._odom_frame = f'{_ns_prefix}/odom'
            self._base_frame = f'{_ns_prefix}/base_footprint'
        else:
            self._odom_frame = 'odom'
            self._base_frame = 'base_footprint'

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        output_scan_topic = str(self.get_parameter('output_scan_topic').value)
        output_scan_topic_emergency = str(self.get_parameter('output_scan_topic_emergency').value)
        output_scan_topic_side = str(self.get_parameter('output_scan_topic_side').value)
        self._scan_pub = self.create_publisher(
            LaserScan,
            output_scan_topic,
            qos_profile_sensor_data,
        )
        self._scan_pub_emergency = self.create_publisher(
            LaserScan,
            output_scan_topic_emergency,
            qos_profile_sensor_data,
        )
        self._scan_pub_side = self.create_publisher(
            LaserScan,
            output_scan_topic_side,
            qos_profile_sensor_data,
        )
        self._debug_pub = self.create_publisher(
            String,
            str(self.get_parameter('debug_topic').value),
            10,
        )

        for i, topic in enumerate(self._range_topics[:3]):
            self.create_subscription(
                Range,
                topic,
                lambda msg, idx=i: self._on_range(idx, msg),
                qos_profile_sensor_data,
            )

        self.create_timer(1.0 / self._publish_hz, self._on_timer)
        self.get_logger().info(
            f'Ultrasonic triangulation blob started: max_valid_range={self._max_valid_range:.2f}m '
            f'similarity_base={self._similarity_m:.3f}m '
            f'similarity_scale={self._similarity_scale_per_m:.3f}/m '
            f'similarity_max={self._similarity_max_m:.3f}m'
        )

    def _is_valid(self, value: float) -> bool:
        return (
            math.isfinite(value)
            and value >= self._min_valid_range
            and value <= self._max_valid_range
        )

    def _on_range(self, idx: int, msg: Range) -> None:
        value = float(msg.range)
        if not math.isfinite(value) or value < self._min_valid_range:
            return
        self._last_msg[idx] = msg
        self._last_recv_s[idx] = self.get_clock().now().nanoseconds * 1e-9

    def _to_point(self, sensor_idx: int, msg: Range):
        source_frame = msg.header.frame_id or self._range_frames[sensor_idx]
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_scan_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.10),
            )
        except Exception as exc:  # noqa: BLE001
            return None, str(exc)

        sx = tf.transform.translation.x
        sy = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        r = float(msg.range)
        ox = sx + r * math.cos(yaw)
        oy = sy + r * math.sin(yaw)
        return (ox, oy, r), None

    def _emergency_cone_for_xy(self, blob_x: float, blob_y: float) -> float:
        """Widen emergency cone slightly for near-head-on hits (low pipes at shallow angle)."""
        cone = self._front_emergency_cone_scale
        bore = abs(math.atan2(blob_y, blob_x))
        if bore < self._front_emergency_head_on_angle_rad:
            cone = min(
                self._front_emergency_head_on_cone_max,
                cone * self._front_emergency_head_on_cone_boost,
            )
        return cone

    def _paint_blob(
        self,
        scan: LaserScan,
        blob_x: float,
        blob_y: float,
        radius: float,
        cone_scale: float = 1.0,
    ) -> bool:
        blob_dist = math.hypot(blob_x, blob_y)
        if not math.isfinite(blob_dist) or blob_dist < self._min_valid_range or blob_dist > self._max_valid_range:
            return False
        angle = math.atan2(blob_y, blob_x)
        cs = max(1.0, float(cone_scale))
        half_width = min(0.55, math.atan2(max(0.01, radius), max(0.01, blob_dist)) * cs)
        start = angle - half_width
        end = angle + half_width
        count = len(scan.ranges)
        i0 = max(0, int((start - scan.angle_min) / scan.angle_increment))
        i1 = min(count - 1, int((end - scan.angle_min) / scan.angle_increment))
        for i in range(i0, i1 + 1):
            scan.ranges[i] = blob_dist
        return True

    def _clear_ultrasonic_memory(self) -> None:
        self._last_hard_detection_s = None
        self._memory_odom_anchor = None
        self._last_blob = None
        self._last_blob_stamp_s = None

    def _lookup_odom_xy_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def _front_still_in_emergency_band(self, now_s: float) -> bool:
        """True if front ultrasonic still reports a near hit (do not odom-clear memory)."""
        msg = self._last_msg.get(1)
        if msg is None:
            return False
        if now_s - _stamp_to_s(msg.header.stamp) > self._max_age_sec:
            return False
        v = float(msg.range)
        return self._is_valid(v) and v <= self._front_emergency_range_m * 1.22

    def _invalidate_memory_if_odom_exceeded(self, now_s: float) -> None:
        if self._memory_odom_anchor is None:
            return
        cur = self._lookup_odom_xy_yaw()
        if cur is None:
            return
        ax, ay, ayaw = self._memory_odom_anchor
        cx, cy, cyaw = cur
        # Body-fixed LaserScan marks rotate with the base. After a substantial yaw
        # change, stale blob_xy must not be replayed while the front range gate is
        # still true — that combination led to "turn then drive forward into pipe".
        if _yaw_delta(cyaw, ayaw) > self._memory_replay_max_odom_yaw_rad:
            self._clear_ultrasonic_memory()
            return
        if self._front_still_in_emergency_band(now_s):
            return
        d = math.hypot(cx - ax, cy - ay)
        if d > self._memory_replay_max_odom_travel_m:
            self._clear_ultrasonic_memory()

    def _invalidate_lateral_blob_on_yaw_drift(self, now_s: float) -> None:
        """Drop held/replay body-frame snapshot after moderate yaw (side-biased clusters only).

        Small turns otherwise keep painting the same blob_x/y in base_scan while the base
        rotates; costmap marks then feel like they \"follow\" the robot. front_emergency
        and centered ``front`` are excluded so pipe approaches are not opened briefly while
        Nav2 still commands forward.         Cancelling only when the blob is clearly off-boresight avoids stripping shallow angled
        pipe marks. For ``front_left`` / ``front_right``, yaw-cancel may run even if the front
        range is still in the emergency band so a floor bag does not leave a frozen mark.
        """
        thr = self._lateral_blob_cancel_min_odom_yaw_rad
        if (
            thr <= 0.0
            or self._memory_odom_anchor is None
            or self._last_blob is None
        ):
            return
        base = str(self._last_blob.get('cluster', '')).replace('_held', '')
        if base not in ('front_left', 'front_right', 'front_hint'):
            return
        angle = abs(
            math.atan2(
                float(self._last_blob['blob_y']),
                float(self._last_blob['blob_x']),
            )
        )
        min_abs = self._lateral_blob_cancel_min_abs_blob_angle_rad
        if min_abs > 0.0 and angle < min_abs:
            return
        # Bag bulk often keeps the front range in the emergency band while the robot yaws;
        # that previously blocked all lateral yaw-cancel and the held blob "followed" the
        # base. Allow cancel for clear left/right triangulation after the angle gate; keep
        # the band guard for front_hint and shallow-angle cases only.
        strongly_side = base in ('front_left', 'front_right')
        if not strongly_side and self._front_still_in_emergency_band(now_s):
            return
        cur = self._lookup_odom_xy_yaw()
        if cur is None:
            return
        _, _, ayaw = self._memory_odom_anchor
        _, _, cyaw = cur
        if _yaw_delta(cyaw, ayaw) < thr:
            return
        self._last_blob = None
        self._last_blob_stamp_s = None
        self._memory_odom_anchor = cur

    def _emergency_extra_hold_for_blob_dist(self, blob_dist: float) -> float:
        if (
            math.isfinite(blob_dist)
            and blob_dist < self._front_emergency_close_blob_dist_m
        ):
            return self._front_emergency_close_extra_hold_sec
        return 0.0

    def _try_publish_held_blob(self, now_s: float, scan: LaserScan, debug: dict, reason: str) -> bool:
        hold_sec = self._blob_hold_sec
        if self._last_blob is not None:
            cl = str(self._last_blob.get('cluster'))
            lx = float(self._last_blob['blob_x'])
            ly = float(self._last_blob['blob_y'])
            dist = math.hypot(lx, ly)
            if cl == 'front_emergency':
                hold_sec = max(hold_sec, self._front_emergency_hold_sec)
            elif cl == 'front':
                hold_sec = max(hold_sec, self._blob_hold_sec)
            if cl in ('front', 'front_emergency'):
                hold_sec += self._emergency_extra_hold_for_blob_dist(dist)
            elif cl in ('front_left', 'front_right') and dist <= self._front_emergency_range_m:
                hold_sec += self._emergency_extra_hold_for_blob_dist(dist)
        if (
            self._last_blob is None
            or self._last_blob_stamp_s is None
            or (now_s - self._last_blob_stamp_s) > hold_sec
        ):
            return False
        blob_x = float(self._last_blob['blob_x'])
        blob_y = float(self._last_blob['blob_y'])
        held_cluster = str(self._last_blob['cluster']) + '_held'
        base_cl = str(self._last_blob['cluster'])
        radius = self._blob_radius_m
        if base_cl == 'front_emergency':
            radius = max(radius, self._front_emergency_blob_radius_m)
        held_dist = math.hypot(blob_x, blob_y)
        if base_cl in ('front', 'front_emergency', 'front_left', 'front_right') and (
            held_dist < self._front_emergency_close_blob_dist_m
        ):
            radius = max(
                radius,
                self._front_emergency_blob_radius_m * self._front_emergency_close_radius_scale,
            )
        if str(held_cluster).startswith('front_emergency'):
            cone = self._emergency_cone_for_xy(blob_x, blob_y)
        else:
            cone = 1.0
        if not self._paint_blob(scan, blob_x, blob_y, radius, cone_scale=cone):
            return False
        debug.update({
            'cluster': held_cluster,
            'blob_x_m': blob_x,
            'blob_y_m': blob_y,
            'blob_dist_m': math.hypot(blob_x, blob_y),
            'blob_angle_rad': math.atan2(blob_y, blob_x),
            'blob_radius_m': radius,
            'hold_reason': reason,
        })
        self._publish_cluster_scans(scan, held_cluster)
        self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
        return True

    def _publish_cluster_scans(self, scan: LaserScan, cluster: str) -> None:
        self._scan_pub.publish(scan)
        emergency_scan = LaserScan()
        emergency_scan.header = scan.header
        emergency_scan.angle_min = scan.angle_min
        emergency_scan.angle_max = scan.angle_max
        emergency_scan.angle_increment = scan.angle_increment
        emergency_scan.range_min = scan.range_min
        emergency_scan.range_max = scan.range_max
        emergency_scan.intensities = []
        emergency_scan.ranges = [math.inf] * len(scan.ranges)
        side_scan = LaserScan()
        side_scan.header = scan.header
        side_scan.angle_min = scan.angle_min
        side_scan.angle_max = scan.angle_max
        side_scan.angle_increment = scan.angle_increment
        side_scan.range_min = scan.range_min
        side_scan.range_max = scan.range_max
        side_scan.intensities = []
        side_scan.ranges = [math.inf] * len(scan.ranges)
        c = str(cluster)
        emergency_hit = False
        side_hit = False
        if c == 'none':
            pass
        elif c.startswith('front_emergency'):
            emergency_hit = True
        elif c == 'front_emergency_replay':
            emergency_hit = True
        elif c in ('front', 'front_held'):
            emergency_hit = True
            side_hit = True
        elif c == 'front_hint':
            side_hit = True
        else:
            side_hit = True
        if emergency_hit:
            emergency_scan.ranges = list(scan.ranges)
        if side_hit:
            side_scan.ranges = list(scan.ranges)
        self._scan_pub_emergency.publish(emergency_scan)
        self._scan_pub_side.publish(side_scan)

    def _try_publish_memory_phase_blob(
        self, now_s: float, scan: LaserScan, debug: dict
    ) -> bool:
        """Republish last frontal obstacle during hard/decay window (costmap-only safety).

        Debug previously exposed memory_phase without republishing scans; Nav2 then lost the
        mark whenever triangulation returned none (stale ranges, pair disagreement, etc.).
        """
        if self._last_blob is None or self._last_hard_detection_s is None:
            return False
        base = str(self._last_blob.get('cluster')).replace('_held', '')
        if base not in (
            'front', 'front_emergency', 'front_left', 'front_right',
        ):
            return False
        age = now_s - self._last_hard_detection_s
        if age <= self._hard_block_duration_sec:
            phase = 'hard'
        elif age <= self._hard_block_duration_sec + self._memory_decay_duration_sec:
            phase = 'decay'
        else:
            return False
        if phase == 'decay' and not self._memory_replay_in_decay:
            return False
        blob_x0 = float(self._last_blob['blob_x'])
        blob_y0 = float(self._last_blob['blob_y'])
        blob_dist0 = math.hypot(blob_x0, blob_y0)
        if (
            not math.isfinite(blob_dist0)
            or blob_dist0 < self._min_valid_range
            or blob_dist0 > self._max_valid_range
        ):
            return False
        angle = math.atan2(blob_y0, blob_x0)
        eff_dist = blob_dist0
        msg_f = self._last_msg.get(1)
        if msg_f is not None:
            age_fr = now_s - _stamp_to_s(msg_f.header.stamp)
            if age_fr <= self._max_age_sec:
                v = float(msg_f.range)
                if self._is_valid(v):
                    eff_dist = min(eff_dist, v)
        stored_fr = self._last_blob.get('front_range_m')
        if stored_fr is not None:
            try:
                sv = float(stored_fr)
                if self._is_valid(sv):
                    eff_dist = min(eff_dist, sv)
            except (TypeError, ValueError):
                pass
        inward = (
            self._memory_replay_inward_m
            if phase == 'hard'
            else self._memory_replay_inward_m * 0.5
        )
        eff_dist = max(self._min_valid_range, eff_dist - inward)
        blob_x = eff_dist * math.cos(angle)
        blob_y = eff_dist * math.sin(angle)
        blob_dist = eff_dist
        radius = self._blob_radius_m
        if base == 'front_emergency':
            radius = max(radius, self._front_emergency_blob_radius_m)
        if blob_dist < self._front_emergency_close_blob_dist_m:
            radius = max(
                radius,
                self._front_emergency_blob_radius_m * self._front_emergency_close_radius_scale,
            )
        if phase == 'decay':
            fade = (age - self._hard_block_duration_sec) / max(
                self._memory_decay_duration_sec, 1e-6
            )
            radius *= max(0.55, 1.0 - 0.45 * min(1.0, fade))
        if not self._paint_blob(
            scan, blob_x, blob_y, radius, cone_scale=self._emergency_cone_for_xy(blob_x, blob_y)
        ):
            return False
        debug.update({
            'cluster': 'front_emergency_replay',
            'blob_x_m': blob_x,
            'blob_y_m': blob_y,
            'blob_dist_m': blob_dist,
            'blob_dist_stored_m': blob_dist0,
            'blob_angle_rad': angle,
            'blob_radius_m': radius,
            'memory_replay': True,
            'memory_replay_phase': phase,
            'memory_replay_inward_m': inward,
        })
        self._publish_cluster_scans(scan, 'front_emergency_replay')
        self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
        return True

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._invalidate_memory_if_odom_exceeded(now_s)
        self._invalidate_lateral_blob_on_yaw_drift(now_s)
        active = {}
        for idx in (0, 1, 2):
            msg = self._last_msg[idx]
            if msg is None:
                continue
            age = now_s - _stamp_to_s(msg.header.stamp)
            if age > self._max_age_sec:
                continue
            value = float(msg.range)
            if self._is_valid(value):
                active[idx] = msg

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._base_scan_frame
        scan.angle_min = self._scan_angle_min
        scan.angle_max = self._scan_angle_max
        scan.angle_increment = self._scan_angle_increment
        scan.range_min = self._min_valid_range
        scan.range_max = self._max_valid_range
        span = scan.angle_max - scan.angle_min
        count = int(max(2, round(span / scan.angle_increment) + 1))
        scan.ranges = [math.inf] * count
        scan.intensities = []

        debug = {
            'event': 'triangulation',
            'cluster': 'none',
            'active_sensors': sorted([['l', 'f', 'r'][i] for i in active.keys()]),
            'blob_x_m': None,
            'blob_y_m': None,
            'blob_dist_m': None,
            'blob_angle_rad': None,
            'similarity_threshold_m': None,
            'delta_lf_m': None,
            'delta_rf_m': None,
            'left_sim': None,
            'right_sim': None,
            'error': None,
        }
        hard_active = False
        memory_age_sec = None
        memory_phase = 'none'
        if self._last_hard_detection_s is not None:
            age = max(0.0, now_s - self._last_hard_detection_s)
            memory_age_sec = age
            if age <= self._hard_block_duration_sec:
                hard_active = True
                memory_phase = 'hard'
            elif age <= (self._hard_block_duration_sec + self._memory_decay_duration_sec):
                memory_phase = 'decay'
            else:
                self._last_hard_detection_s = None
                self._memory_odom_anchor = None
                memory_age_sec = None
        debug.update({
            'hard_active': hard_active,
            'memory_phase': memory_phase,
            'memory_age_sec': memory_age_sec,
        })

        if not self._triangulation_enabled:
            self._publish_cluster_scans(scan, 'none')
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return
        if 1 not in active:
            if self._try_publish_held_blob(now_s, scan, debug, 'front_not_active'):
                return
            if self._try_publish_memory_phase_blob(now_s, scan, debug):
                return
            self._publish_cluster_scans(scan, 'none')
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        points = {}
        for idx, msg in active.items():
            point, err = self._to_point(idx, msg)
            if err is not None:
                debug['error'] = err[:200]
                if not self._try_publish_memory_phase_blob(now_s, scan, debug):
                    self._publish_cluster_scans(scan, 'none')
                    self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
                return
            points[idx] = point

        if 1 not in points:
            if not self._try_publish_memory_phase_blob(now_s, scan, debug):
                self._publish_cluster_scans(scan, 'none')
                self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        left = points.get(0)
        front = points.get(1)
        right = points.get(2)
        similarity_threshold = self._similarity_m
        if front is not None:
            similarity_threshold = min(
                self._similarity_max_m,
                self._similarity_m + self._similarity_scale_per_m * max(0.0, front[2])
            )
        delta_lf = None if left is None else abs(left[2] - front[2])
        delta_rf = None if right is None else abs(right[2] - front[2])
        left_sim = left is not None and delta_lf <= similarity_threshold
        right_sim = right is not None and delta_rf <= similarity_threshold
        debug['similarity_threshold_m'] = similarity_threshold
        debug['delta_lf_m'] = delta_lf
        debug['delta_rf_m'] = delta_rf
        debug['left_sim'] = left_sim
        debug['right_sim'] = right_sim

        blob_x = front[0]
        blob_y = front[1]
        cluster = 'front'
        has_pair = left_sim or right_sim
        # Promote close front detections to emergency regardless of side agreement.
        # This closes a gap where front_left/front_right could pass through near
        # low obstacles without entering the stronger emergency behavior.
        front_emergency_active = False
        if front[2] <= self._front_emergency_range_m:
            self._front_emergency_streak += 1
        else:
            self._front_emergency_streak = 0
        if self._front_emergency_streak >= self._front_emergency_required_streak:
            front_emergency_active = True
        if self._require_pair_agreement and not has_pair:
            if front_emergency_active:
                if front[2] <= self._front_emergency_strict_range_m:
                    cluster = 'front_emergency'
                    blob_x = front[0]
                    blob_y = front[1]
                else:
                    cluster = 'front_hint'
                    blob_x = front[0]
                    blob_y = front[1]
            else:
                self._front_emergency_streak = max(0, self._front_emergency_streak)
                if (
                    self._try_publish_held_blob(now_s, scan, debug, 'pair_disagreement')
                ):
                    return
                if self._try_publish_memory_phase_blob(now_s, scan, debug):
                    return
                self._publish_cluster_scans(scan, 'none')
                debug['cluster'] = 'none'
                self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
                return
        else:
            if front_emergency_active:
                if front[2] <= self._front_emergency_strict_range_m:
                    cluster = 'front_emergency'
                    blob_x = front[0]
                    blob_y = front[1]
                else:
                    cluster = 'front_hint'
                    blob_x = front[0]
                    blob_y = front[1]
            else:
                self._front_emergency_streak = 0

        if cluster == 'front_emergency':
            pass
        elif cluster == 'front_hint':
            pass
        elif left_sim and right_sim and left is not None and right is not None:
            cluster = 'front'
            blob_x = (left[0] + front[0] + right[0]) / 3.0
            blob_y = (left[1] + front[1] + right[1]) / 3.0
        elif left_sim and left is not None:
            cluster = 'front_left'
            # Keep lateral placement biased to the side sensor to avoid
            # over-centering near corridor/table edges.
            side_w = 0.70
            front_w = 0.30
            blob_x = side_w * left[0] + front_w * front[0]
            blob_y = side_w * left[1] + front_w * front[1]
        elif right_sim and right is not None:
            cluster = 'front_right'
            side_w = 0.70
            front_w = 0.30
            blob_x = side_w * right[0] + front_w * front[0]
            blob_y = side_w * right[1] + front_w * front[1]

        blob_dist = math.hypot(blob_x, blob_y)
        if not math.isfinite(blob_dist) or blob_dist < self._min_valid_range or blob_dist > self._max_valid_range:
            if not self._try_publish_memory_phase_blob(now_s, scan, debug):
                self._publish_cluster_scans(scan, 'none')
                debug['cluster'] = cluster
                self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        radius = self._blob_radius_m
        if cluster == 'front_emergency':
            radius = max(radius, self._front_emergency_blob_radius_m)
            if blob_dist < self._front_emergency_close_blob_dist_m:
                radius = max(
                    radius,
                    self._front_emergency_blob_radius_m * self._front_emergency_close_radius_scale,
                )
        elif cluster == 'front_hint':
            radius = max(radius, self._blob_radius_m * 1.12)
        cone_scale = 1.0
        if cluster == 'front_emergency':
            cone_scale = self._emergency_cone_for_xy(blob_x, blob_y)
        elif cluster in ('front_left', 'front_right') and (
            blob_dist <= self._front_emergency_range_m
        ):
            cone_scale = min(1.30, self._front_emergency_cone_scale * 0.92)
        if not self._paint_blob(scan, blob_x, blob_y, radius, cone_scale=cone_scale):
            if not self._try_publish_memory_phase_blob(now_s, scan, debug):
                self._publish_cluster_scans(scan, 'none')
                debug['cluster'] = cluster
                self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        self._last_blob = {
            'cluster': str(cluster).replace('_held', ''),
            'blob_x': blob_x,
            'blob_y': blob_y,
            'front_range_m': float(front[2]),
        }
        self._last_blob_stamp_s = now_s
        if cluster in ('front', 'front_emergency'):
            self._last_hard_detection_s = now_s
            self._memory_odom_anchor = self._lookup_odom_xy_yaw()
        elif cluster in ('front_left', 'front_right') and (
            blob_dist <= self._front_emergency_range_m
        ):
            self._last_hard_detection_s = now_s
            self._memory_odom_anchor = self._lookup_odom_xy_yaw()

        angle = math.atan2(blob_y, blob_x)
        close_extra = self._emergency_extra_hold_for_blob_dist(blob_dist)
        debug.update({
            'cluster': cluster,
            'blob_x_m': blob_x,
            'blob_y_m': blob_y,
            'blob_dist_m': blob_dist,
            'blob_angle_rad': angle,
            'blob_radius_m': radius,
            'emergency_close_range': bool(cluster == 'front_emergency' and close_extra > 0.0),
            'emergency_close_extra_hold_sec': close_extra,
            'cone_scale': cone_scale,
        })
        self._publish_cluster_scans(scan, cluster)
        self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicTriangulationBlob()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
