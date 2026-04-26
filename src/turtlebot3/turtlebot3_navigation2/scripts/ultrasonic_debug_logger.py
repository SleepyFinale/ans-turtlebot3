#!/usr/bin/env python3
"""
Ultrasonic diagnostics logger.

Records ultrasonic ranges, message health, TF availability, scan fusion deltas,
and Nav2 range-layer related context into JSONL for post-run analysis.
"""

import json
import math
import os
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String
import tf2_ros


def _now_iso() -> str:
    return datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'


def _stamp_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class UltrasonicDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_debug_logger')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('output_dir', '~/turtlebot3/logs')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('stale_after_s', 0.6)
        self.declare_parameter('flatline_epsilon_m', 0.01)
        self.declare_parameter('flatline_streak_threshold', 20)
        self.declare_parameter('scan_compare_window_bins', 2)
        self.declare_parameter('scan_fusion_delta_min_m', 0.03)
        self.declare_parameter('base_scan_frame', 'base_scan')
        self.declare_parameter('range_topic_prefix', 'ultrasonic_')
        self.declare_parameter('triangulation_debug_topic', 'ultrasonic_triangulation_debug')
        self.declare_parameter(
            'range_frames',
            ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right'],
        )

        robot_name = str(self.get_parameter('robot_name').value).strip() or (
            os.environ.get('USER') or os.environ.get('LOGNAME') or 'robot'
        )
        self._log_rate_hz = max(0.5, min(20.0, float(self.get_parameter('log_rate_hz').value)))
        self._stale_after_s = max(0.1, float(self.get_parameter('stale_after_s').value))
        self._flatline_epsilon_m = max(0.0001, float(self.get_parameter('flatline_epsilon_m').value))
        self._flatline_streak_threshold = max(3, int(self.get_parameter('flatline_streak_threshold').value))
        self._scan_compare_window_bins = max(1, int(self.get_parameter('scan_compare_window_bins').value))
        self._scan_fusion_delta_min_m = max(0.001, float(self.get_parameter('scan_fusion_delta_min_m').value))

        self._ns = self.get_namespace().strip('/')
        self._base_scan_frame = str(self.get_parameter('base_scan_frame').value).strip()
        if self._ns and '/' not in self._base_scan_frame:
            self._base_scan_frame = f'{self._ns}/{self._base_scan_frame}'

        output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        session_dir = Path(output_dir) / robot_name
        session_dir.mkdir(parents=True, exist_ok=True)
        session_name = datetime.now().strftime('ultrasonic-session-%Y%m%d-%H%M%S.jsonl')
        self._jsonl_path = session_dir / session_name
        self._jsonl = self._jsonl_path.open('a', encoding='utf-8')

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sensor_labels = ['l', 'f', 'r']
        frame_list = list(self.get_parameter('range_frames').value)
        if len(frame_list) != 3:
            frame_list = ['ultrasonic_link_left', 'ultrasonic_link_front', 'ultrasonic_link_right']
        self._topic_prefix = str(self.get_parameter('range_topic_prefix').value).strip()
        self._sensor_meta: Dict[str, Dict] = {}
        for idx, label in enumerate(sensor_labels):
            topic = f'{self._topic_prefix}{label}'
            frame = str(frame_list[idx]).strip()
            if self._ns and '/' not in frame:
                frame = f'{self._ns}/{frame}'
            self._sensor_meta[label] = {'topic': topic, 'frame': frame}

        # Per-sensor state
        self._last_range: Dict[str, Optional[Range]] = {k: None for k in self._sensor_meta}
        self._last_recv_s: Dict[str, Optional[float]] = {k: None for k in self._sensor_meta}
        self._last_value: Dict[str, Optional[float]] = {k: None for k in self._sensor_meta}
        self._flatline_streak: Dict[str, int] = {k: 0 for k in self._sensor_meta}
        self._msg_count: Dict[str, int] = {k: 0 for k in self._sensor_meta}
        self._rate_window: Dict[str, deque] = {k: deque(maxlen=100) for k in self._sensor_meta}
        self._tf_fail_count: Dict[str, int] = {k: 0 for k in self._sensor_meta}
        self._tf_last_ok_s: Dict[str, Optional[float]] = {k: None for k in self._sensor_meta}

        self._latest_scan: Optional[LaserScan] = None
        self._latest_scan_normalized: Optional[LaserScan] = None
        self._last_scan_s: Optional[float] = None
        self._last_scan_normalized_s: Optional[float] = None

        self._last_local_costmap_s: Optional[float] = None
        self._last_global_costmap_s: Optional[float] = None
        self._last_cmd_vel: Optional[Dict] = None
        self._last_cmd_vel_s: Optional[float] = None
        self._last_odom: Optional[Dict] = None
        self._last_odom_s: Optional[float] = None
        self._last_collision_ahead: Optional[bool] = None
        self._last_collision_ahead_s: Optional[float] = None
        self._rosout_range_warn_count = 0
        self._rosout_collision_ahead_warn_count = 0
        self._rosout_recent: deque = deque(maxlen=30)
        self._last_triangulation_debug: Optional[Dict] = None

        self._setup_subscriptions()

        self.create_timer(1.0 / self._log_rate_hz, self._on_tick)
        self._write_event('session_start', {
            'jsonl_path': str(self._jsonl_path),
            'namespace': self.get_namespace(),
            'base_scan_frame': self._base_scan_frame,
            'sensor_meta': self._sensor_meta,
        })
        self.get_logger().info(f'Ultrasonic debug logger started: {self._jsonl_path}')

    def _setup_subscriptions(self) -> None:
        qos_map = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_default = QoSProfile(depth=20)

        for label, meta in self._sensor_meta.items():
            self.create_subscription(
                Range,
                meta['topic'],
                lambda msg, sensor_label=label: self._on_range(sensor_label, msg),
                qos_profile_sensor_data,
            )
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_normalized', self._on_scan_normalized, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'ultrasonic_blob_scan', self._on_ultrasonic_blob_scan, qos_profile_sensor_data)
        self.create_subscription(
            String,
            str(self.get_parameter('triangulation_debug_topic').value),
            self._on_triangulation_debug,
            qos_default,
        )
        self.create_subscription(Costmap, 'local_costmap/costmap_raw', self._on_local_costmap, qos_map)
        self.create_subscription(Costmap, 'global_costmap/costmap_raw', self._on_global_costmap, qos_map)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, 'nav2_collision_ahead', self._on_collision_ahead, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self._on_rosout, qos_default)

    def _write_event(self, event: str, payload: Dict) -> None:
        obj = {'ts_iso': _now_iso(), 'event': event, **payload}
        self._jsonl.write(json.dumps(obj, separators=(',', ':')) + '\n')
        self._jsonl.flush()

    def _on_range(self, label: str, msg: Range) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_range[label] = msg
        self._last_recv_s[label] = now_s
        self._msg_count[label] += 1
        self._rate_window[label].append(now_s)

        value = float(msg.range)
        prev = self._last_value[label]
        if prev is None:
            self._flatline_streak[label] = 0
        elif abs(value - prev) <= self._flatline_epsilon_m:
            self._flatline_streak[label] += 1
        else:
            self._flatline_streak[label] = 0
        self._last_value[label] = value

        age_s = max(0.0, now_s - _stamp_to_s(msg.header.stamp))
        self._write_event('range_sample', {
            'sensor': label,
            'topic': self._sensor_meta[label]['topic'],
            'frame_id': msg.header.frame_id,
            'range_m': value,
            'min_range_m': float(msg.min_range),
            'max_range_m': float(msg.max_range),
            'age_s': age_s,
            'stale': age_s > self._stale_after_s,
            'flatline_streak': self._flatline_streak[label],
        })

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._last_scan_s = self.get_clock().now().nanoseconds * 1e-9

    def _on_scan_normalized(self, msg: LaserScan) -> None:
        self._latest_scan_normalized = msg
        self._last_scan_normalized_s = self.get_clock().now().nanoseconds * 1e-9

    def _on_ultrasonic_blob_scan(self, msg: LaserScan) -> None:
        finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
        self._write_event('triangulation_blob_scan', {
            'finite_points': len(finite),
            'blob_min_m': min(finite) if finite else None,
            'blob_max_m': max(finite) if finite else None,
            'scan_len': len(msg.ranges),
        })

    def _on_triangulation_debug(self, msg: String) -> None:
        text = str(msg.data).strip()
        if not text:
            return
        try:
            data = json.loads(text)
            if not isinstance(data, dict):
                return
        except Exception:
            return
        # Keep logger event typing stable even if payload carries its own "event".
        data.pop('event', None)
        self._last_triangulation_debug = data
        self._write_event('triangulation_decision', data)

    def _on_local_costmap(self, _msg: Costmap) -> None:
        self._last_local_costmap_s = self.get_clock().now().nanoseconds * 1e-9

    def _on_global_costmap(self, _msg: Costmap) -> None:
        self._last_global_costmap_s = self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_cmd_vel = {
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'angular_z': float(msg.angular.z),
        }

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_s = self.get_clock().now().nanoseconds * 1e-9
        tw = msg.twist.twist
        self._last_odom = {
            'linear_x': float(tw.linear.x),
            'linear_y': float(tw.linear.y),
            'angular_z': float(tw.angular.z),
        }

    def _on_collision_ahead(self, msg: Bool) -> None:
        self._last_collision_ahead_s = self.get_clock().now().nanoseconds * 1e-9
        self._last_collision_ahead = bool(msg.data)

    def _on_rosout(self, msg: Log) -> None:
        text = msg.msg or ''
        if 'No range readings received' in text:
            self._rosout_range_warn_count += 1
            self._rosout_recent.append({
                'name': msg.name,
                'level': int(msg.level),
                'msg': text[:300],
            })
        if 'detected collision ahead' in text:
            self._rosout_collision_ahead_warn_count += 1

    def _rate_hz(self, label: str) -> float:
        win = self._rate_window[label]
        if len(win) < 2:
            return 0.0
        span = win[-1] - win[0]
        if span <= 0.0:
            return 0.0
        return float(len(win) - 1) / span

    def _lookup_tf(self, source_frame: str, target_frame: str) -> Tuple[bool, Optional[str]]:
        try:
            self._tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            return True, None
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    def _scan_index(self, scan: LaserScan, angle_rad: float) -> Optional[int]:
        if scan is None or not scan.ranges:
            return None
        if scan.angle_increment == 0.0:
            return None
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        return idx

    def _cone_min(self, scan: LaserScan, center_angle: float, window_bins: int) -> Optional[float]:
        idx = self._scan_index(scan, center_angle)
        if idx is None:
            return None
        start = max(0, idx - window_bins)
        end = min(len(scan.ranges) - 1, idx + window_bins)
        values = []
        for i in range(start, end + 1):
            v = float(scan.ranges[i])
            if math.isfinite(v):
                values.append(v)
        if not values:
            return None
        return min(values)

    def _on_tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        anomalies: List[str] = []
        tf_rows = []
        fusion_rows = []

        # Approximate cone centers from sensor layout (left/front/right).
        cone_center = {'l': 0.757, 'f': 0.0, 'r': -0.757}

        for label, meta in self._sensor_meta.items():
            last_recv = self._last_recv_s[label]
            age_s = None if last_recv is None else max(0.0, now_s - last_recv)
            stale = age_s is None or age_s > self._stale_after_s
            if stale:
                anomalies.append(f'range_stale_{label}')
            if self._flatline_streak[label] >= self._flatline_streak_threshold:
                anomalies.append(f'range_flatlined_{label}')

            rate = self._rate_hz(label)
            gap_s = None
            win = self._rate_window[label]
            if len(win) >= 2:
                gap_s = max(0.0, win[-1] - win[-2])

            self._write_event('range_health', {
                'sensor': label,
                'topic': meta['topic'],
                'message_count': self._msg_count[label],
                'rate_hz': rate,
                'last_gap_s': gap_s,
                'last_recv_age_s': age_s,
                'flatline_streak': self._flatline_streak[label],
            })

            ok, err = self._lookup_tf(self._base_scan_frame, meta['frame'])
            if ok:
                self._tf_last_ok_s[label] = now_s
            else:
                self._tf_fail_count[label] += 1
                anomalies.append(f'tf_lookup_failed_{label}')
            tf_rows.append({
                'sensor': label,
                'source_frame': self._base_scan_frame,
                'target_frame': meta['frame'],
                'ok': ok,
                'error': err[:300] if err else None,
                'tf_fail_count': self._tf_fail_count[label],
                'tf_last_ok_age_s': (
                    None if self._tf_last_ok_s[label] is None else max(0.0, now_s - self._tf_last_ok_s[label])
                ),
            })

            # Compare raw vs fused scan around each ultrasonic cone.
            raw_min = self._cone_min(self._latest_scan, cone_center[label], self._scan_compare_window_bins)
            norm_min = self._cone_min(
                self._latest_scan_normalized,
                cone_center[label],
                self._scan_compare_window_bins,
            )
            delta = None
            if raw_min is not None and norm_min is not None:
                delta = raw_min - norm_min
                if abs(delta) < self._scan_fusion_delta_min_m:
                    anomalies.append(f'fusion_no_effect_{label}')
            fusion_rows.append({
                'sensor': label,
                'raw_cone_min_m': raw_min,
                'normalized_cone_min_m': norm_min,
                'delta_raw_minus_norm_m': delta,
            })

        self._write_event('tf_snapshot', {'rows': tf_rows})
        self._write_event('scan_compare', {
            'scan_age_s': None if self._last_scan_s is None else max(0.0, now_s - self._last_scan_s),
            'scan_normalized_age_s': (
                None if self._last_scan_normalized_s is None else max(0.0, now_s - self._last_scan_normalized_s)
            ),
            'rows': fusion_rows,
        })
        self._write_event('fusion_effect', {
            'rows': fusion_rows,
            'estimated_overrides': sum(
                1 for row in fusion_rows
                if row['delta_raw_minus_norm_m'] is not None and row['delta_raw_minus_norm_m'] > self._scan_fusion_delta_min_m
            ),
        })

        local_age = None if self._last_local_costmap_s is None else max(0.0, now_s - self._last_local_costmap_s)
        global_age = None if self._last_global_costmap_s is None else max(0.0, now_s - self._last_global_costmap_s)
        if self._rosout_range_warn_count > 0:
            anomalies.append('range_layer_no_input_warn')
        self._write_event('nav_context', {
            'local_costmap_age_s': local_age,
            'global_costmap_age_s': global_age,
            'rosout_no_range_readings_count': self._rosout_range_warn_count,
            'rosout_collision_ahead_count': self._rosout_collision_ahead_warn_count,
            'rosout_recent': list(self._rosout_recent),
            'triangulation_cluster': (
                None if self._last_triangulation_debug is None
                else self._last_triangulation_debug.get('cluster')
            ),
            'cmd_vel': self._last_cmd_vel,
            'cmd_vel_age_s': (
                None if self._last_cmd_vel_s is None else max(0.0, now_s - self._last_cmd_vel_s)
            ),
            'odom_twist': self._last_odom,
            'odom_age_s': (
                None if self._last_odom_s is None else max(0.0, now_s - self._last_odom_s)
            ),
            'nav2_collision_ahead': self._last_collision_ahead,
            'nav2_collision_ahead_age_s': (
                None if self._last_collision_ahead_s is None
                else max(0.0, now_s - self._last_collision_ahead_s)
            ),
        })

        if anomalies:
            self._write_event('anomaly', {'flags': sorted(set(anomalies))})

    def destroy_node(self) -> bool:
        try:
            self._write_event('session_end', {})
            self._jsonl.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicDebugLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        try:
            node.get_logger().warn(f'ultrasonic_debug_logger stopping: {exc}')
        except Exception:
            pass
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
