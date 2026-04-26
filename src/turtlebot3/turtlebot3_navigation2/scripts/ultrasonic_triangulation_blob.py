#!/usr/bin/env python3
"""
Triangulate overlapping ultrasonic detections into a compact obstacle blob scan.
"""

import json
import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
import tf2_ros


def _stamp_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class UltrasonicTriangulationBlob(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_triangulation_blob')

        self.declare_parameter('range_topics', ['ultrasonic_l', 'ultrasonic_f', 'ultrasonic_r'])
        self.declare_parameter('range_frame_ids', ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right'])
        self.declare_parameter('base_scan_frame', 'base_scan')
        self.declare_parameter('output_scan_topic', 'ultrasonic_blob_scan')
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
        self.declare_parameter('front_emergency_required_streak', 1)
        self.declare_parameter('front_emergency_blob_radius_m', 0.16)
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
        self._front_emergency_required_streak = max(
            1, int(self.get_parameter('front_emergency_required_streak').value)
        )
        self._front_emergency_blob_radius_m = max(
            self._blob_radius_m,
            float(self.get_parameter('front_emergency_blob_radius_m').value)
        )
        self._blob_hold_sec = max(0.0, float(self.get_parameter('blob_hold_sec').value))

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

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('output_scan_topic').value),
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
        if not self._is_valid(float(msg.range)):
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

    def _paint_blob(self, scan: LaserScan, blob_x: float, blob_y: float, radius: float) -> bool:
        blob_dist = math.hypot(blob_x, blob_y)
        if not math.isfinite(blob_dist) or blob_dist < self._min_valid_range or blob_dist > self._max_valid_range:
            return False
        angle = math.atan2(blob_y, blob_x)
        half_width = min(0.5, math.atan2(max(0.01, radius), max(0.01, blob_dist)))
        start = angle - half_width
        end = angle + half_width
        count = len(scan.ranges)
        i0 = max(0, int((start - scan.angle_min) / scan.angle_increment))
        i1 = min(count - 1, int((end - scan.angle_min) / scan.angle_increment))
        for i in range(i0, i1 + 1):
            scan.ranges[i] = blob_dist
        return True

    def _try_publish_held_blob(self, now_s: float, scan: LaserScan, debug: dict, reason: str) -> bool:
        if (
            self._last_blob is None
            or self._last_blob_stamp_s is None
            or (now_s - self._last_blob_stamp_s) > self._blob_hold_sec
        ):
            return False
        blob_x = float(self._last_blob['blob_x'])
        blob_y = float(self._last_blob['blob_y'])
        held_cluster = str(self._last_blob['cluster']) + '_held'
        radius = self._blob_radius_m
        if str(self._last_blob['cluster']) == 'front_emergency':
            radius = max(radius, self._front_emergency_blob_radius_m)
        if not self._paint_blob(scan, blob_x, blob_y, radius):
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
        self._scan_pub.publish(scan)
        self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
        return True

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        active = {}
        for idx in (0, 1, 2):
            msg = self._last_msg[idx]
            if msg is None:
                continue
            age = now_s - _stamp_to_s(msg.header.stamp)
            if age > self._max_age_sec:
                continue
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

        if not self._triangulation_enabled:
            self._scan_pub.publish(scan)
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return
        if 1 not in active:
            if self._try_publish_held_blob(now_s, scan, debug, 'front_not_active'):
                return
            self._scan_pub.publish(scan)
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        points = {}
        for idx, msg in active.items():
            point, err = self._to_point(idx, msg)
            if err is not None:
                debug['error'] = err[:200]
                self._scan_pub.publish(scan)
                self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
                return
            points[idx] = point

        if 1 not in points:
            self._scan_pub.publish(scan)
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
        if self._require_pair_agreement and not has_pair:
            if front[2] <= self._front_emergency_range_m:
                self._front_emergency_streak += 1
            else:
                self._front_emergency_streak = 0
            if self._front_emergency_streak >= self._front_emergency_required_streak:
                cluster = 'front_emergency'
                blob_x = front[0]
                blob_y = front[1]
            else:
                self._front_emergency_streak = max(0, self._front_emergency_streak)
                if (
                    self._try_publish_held_blob(now_s, scan, debug, 'pair_disagreement')
                ):
                    return
                else:
                    self._scan_pub.publish(scan)
                    debug['cluster'] = 'none'
                    self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
                    return
        else:
            self._front_emergency_streak = 0

        if left_sim and right_sim and left is not None and right is not None:
            cluster = 'front'
            blob_x = (left[0] + front[0] + right[0]) / 3.0
            blob_y = (left[1] + front[1] + right[1]) / 3.0
        elif left_sim and left is not None:
            cluster = 'front_left'
            blob_x = 0.5 * (left[0] + front[0])
            blob_y = 0.5 * (left[1] + front[1])
        elif right_sim and right is not None:
            cluster = 'front_right'
            blob_x = 0.5 * (right[0] + front[0])
            blob_y = 0.5 * (right[1] + front[1])

        blob_dist = math.hypot(blob_x, blob_y)
        if not math.isfinite(blob_dist) or blob_dist < self._min_valid_range or blob_dist > self._max_valid_range:
            self._scan_pub.publish(scan)
            debug['cluster'] = cluster
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        radius = self._blob_radius_m
        if cluster == 'front_emergency':
            radius = max(radius, self._front_emergency_blob_radius_m)
        if not self._paint_blob(scan, blob_x, blob_y, radius):
            self._scan_pub.publish(scan)
            debug['cluster'] = cluster
            self._debug_pub.publish(String(data=json.dumps(debug, separators=(',', ':'))))
            return

        self._last_blob = {
            'cluster': str(cluster).replace('_held', ''),
            'blob_x': blob_x,
            'blob_y': blob_y,
        }
        self._last_blob_stamp_s = now_s

        angle = math.atan2(blob_y, blob_x)
        debug.update({
            'cluster': cluster,
            'blob_x_m': blob_x,
            'blob_y_m': blob_y,
            'blob_dist_m': blob_dist,
            'blob_angle_rad': angle,
            'blob_radius_m': radius,
        })
        self._scan_pub.publish(scan)
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
