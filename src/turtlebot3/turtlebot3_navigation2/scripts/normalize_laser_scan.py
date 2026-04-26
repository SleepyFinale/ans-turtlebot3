#!/usr/bin/env python3
"""
Laser scan normalizer to fix variable reading counts for slam_toolbox.
This node normalizes all scans to have a consistent number of readings (default: 228)
by interpolating or padding/truncating as needed.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import tf2_ros
import math
from collections import deque
from collections import Counter
from statistics import median


class LaserScanNormalizer(Node):
    def __init__(self):
        super().__init__('laser_scan_normalizer')
        
        # Fixed target fallback/override; auto-detect mode can lock to observed scan lengths.
        self.declare_parameter('target_readings', 228)
        self.declare_parameter('auto_target_readings', True)
        self.declare_parameter('auto_target_sample_scans', 30)
        self.declare_parameter('auto_target_min_samples', 12)
        self.declare_parameter('auto_target_outlier_tolerance', 12)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_normalized')
        # Publish every Nth scan to reduce SLAM Toolbox message filter queue overflow (1 = every scan)
        # Default 4 = ~2.5 Hz at 10 Hz lidar; prevents "queue is full" drops in SLAM
        self.declare_parameter('publish_every_n_scans', 1)
        # For multi-robot: prefix for frame_id (e.g. "blinky" -> "blinky/base_scan"). Empty = use original.
        self.declare_parameter('frame_id_prefix', '')
        
        # ultrasonic sensor topics
        self.declare_parameter('range_topics', [''])
        self.declare_parameter('range_frame_ids', [''])
        self.declare_parameter('ultrasonic_window_size', 3)
        self.declare_parameter('ultrasonic_max_age_sec', 0.40)
        self.declare_parameter('ultrasonic_min_valid_range', 0.02)
        self.declare_parameter('ultrasonic_max_valid_range', 3.0)
        self.declare_parameter('ultrasonic_fusion_enabled', True)
        self.declare_parameter('ultrasonic_lidar_min_override_delta', 0.10)
        self.declare_parameter('ultrasonic_max_delta_per_update', 0.45)
        self.declare_parameter('ultrasonic_hysteresis_m', 0.03)
        self.declare_parameter('ultrasonic_max_hold_sec', 1.2)
        self.declare_parameter('ultrasonic_hold_epsilon_m', 0.008)
        self.declare_parameter('ultrasonic_cone_scale', 1.0)
        self.declare_parameter('ultrasonic_use_left', True)
        self.declare_parameter('ultrasonic_use_front', True)
        self.declare_parameter('ultrasonic_use_right', True)
        self.declare_parameter('ultrasonic_overlap_arbitration_enabled', True)
        self.declare_parameter('ultrasonic_overlap_similarity_m', 0.10)
        self.declare_parameter('ultrasonic_overlap_side_pair_front_scale', 0.60)

        target_readings = int(self.get_parameter('target_readings').value)
        self._auto_target_enabled = bool(
            self.get_parameter('auto_target_readings').value)
        self._auto_target_sample_scans = max(
            1, int(self.get_parameter('auto_target_sample_scans').value))
        self._auto_target_min_samples = max(
            3, int(self.get_parameter('auto_target_min_samples').value))
        self._auto_target_outlier_tolerance = max(
            0, int(self.get_parameter('auto_target_outlier_tolerance').value))
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_every_n = self.get_parameter('publish_every_n_scans').value
        self._frame_id_prefix = self.get_parameter('frame_id_prefix').value
        self._range_topics = self.get_parameter('range_topics').value
        self._range_frame_ids = self.get_parameter('range_frame_ids').value
        self._ultrasonic_window_size = max(
            1, int(self.get_parameter('ultrasonic_window_size').value))
        self._ultrasonic_max_age_sec = float(
            self.get_parameter('ultrasonic_max_age_sec').value)
        self._ultrasonic_min_valid_range = float(
            self.get_parameter('ultrasonic_min_valid_range').value)
        self._ultrasonic_max_valid_range = float(
            self.get_parameter('ultrasonic_max_valid_range').value)
        self._ultrasonic_fusion_enabled = bool(
            self.get_parameter('ultrasonic_fusion_enabled').value)
        self._ultrasonic_lidar_min_override_delta = float(
            self.get_parameter('ultrasonic_lidar_min_override_delta').value)
        self._ultrasonic_max_delta_per_update = float(
            self.get_parameter('ultrasonic_max_delta_per_update').value)
        self._ultrasonic_hysteresis_m = float(
            self.get_parameter('ultrasonic_hysteresis_m').value)
        self._ultrasonic_max_hold_sec = max(
            0.0, float(self.get_parameter('ultrasonic_max_hold_sec').value))
        self._ultrasonic_hold_epsilon_m = max(
            0.0001, float(self.get_parameter('ultrasonic_hold_epsilon_m').value))
        self._ultrasonic_cone_scale = max(
            0.1, float(self.get_parameter('ultrasonic_cone_scale').value))
        self._ultrasonic_use = {
            0: bool(self.get_parameter('ultrasonic_use_left').value),
            1: bool(self.get_parameter('ultrasonic_use_front').value),
            2: bool(self.get_parameter('ultrasonic_use_right').value),
        }
        self._ultrasonic_overlap_arbitration_enabled = bool(
            self.get_parameter('ultrasonic_overlap_arbitration_enabled').value
        )
        self._ultrasonic_overlap_similarity_m = max(
            0.01, float(self.get_parameter('ultrasonic_overlap_similarity_m').value)
        )
        self._ultrasonic_overlap_side_pair_front_scale = max(
            0.10, min(1.0, float(self.get_parameter('ultrasonic_overlap_side_pair_front_scale').value))
        )
        self._scan_counter = 0
        self._scan_frame = (
            f'{self._frame_id_prefix}/base_scan'
            if self._frame_id_prefix else 'base_scan'
        )

        self._range_subs = []
        self._range_msgs = []
        self._range_windows = []
        self._range_previous_filtered = []
        self._range_last_change_time = []
        self._range_frame_mismatch_counts = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to the original scan with sensor QoS (best effort, volatile)
        from rclpy.qos import qos_profile_sensor_data
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        for index, topic in enumerate(self._range_topics):
            self._range_subs.append(
                self.create_subscription(
                    Range,
                    topic,
                    lambda msg, topic_index=index: self.range_callback(msg, topic_index),
                    qos_profile_sensor_data
                )
            )
            self._range_msgs.append(None)
            self._range_windows.append(deque(maxlen=self._ultrasonic_window_size))
            self._range_previous_filtered.append(None)
            self._range_last_change_time.append(None)

        # Publish the normalized scan with sensor QoS
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data
        )
        
        self._fixed_target_readings = target_readings
        self.target_readings = target_readings
        self._auto_target_locked = not self._auto_target_enabled
        self._auto_target_samples = []
        self._auto_scans_seen = 0

        if self._auto_target_enabled:
            self.get_logger().info(
                'Auto target selection enabled: collecting '
                f'{self._auto_target_sample_scans} scan lengths '
                f'(min valid samples={self._auto_target_min_samples}, '
                f'fallback fixed target={self._fixed_target_readings})'
            )
            self._auto_warmup_log_mod = 10
        else:
            self.get_logger().info(
                f'Auto target selection disabled; using fixed target '
                f'{self._fixed_target_readings}'
            )
        self.get_logger().info(
            'Ultrasonic fusion sensor mask: '
            f'left={self._ultrasonic_use[0]}, front={self._ultrasonic_use[1]}, right={self._ultrasonic_use[2]}'
        )
        self.get_logger().info(
            'Ultrasonic overlap arbitration: '
            f'enabled={self._ultrasonic_overlap_arbitration_enabled}, '
            f'similarity_m={self._ultrasonic_overlap_similarity_m:.3f}, '
            f'side_pair_front_scale={self._ultrasonic_overlap_side_pair_front_scale:.2f}'
        )
        self.get_logger().info(
            f'Laser scan normalizer started: {input_topic} -> {output_topic} '
            f'(normalizing to {target_readings} readings, publishing every {self.publish_every_n} scan(s))'
        )

    def _normalize_frame_id(self, frame_id):
        frame = str(frame_id).strip()
        if frame.startswith('/'):
            frame = frame[1:]
        return frame

    def _frame_ids_match(self, expected_frame, msg_frame):
        expected = self._normalize_frame_id(expected_frame)
        msg = self._normalize_frame_id(msg_frame)
        if not expected:
            return True
        if not msg:
            return False
        if msg == expected:
            return True
        return msg.endswith('/' + expected)

    def _record_frame_mismatch(self, topic_index, expected_frame, msg_frame):
        key = (topic_index, self._normalize_frame_id(expected_frame), self._normalize_frame_id(msg_frame))
        count = self._range_frame_mismatch_counts.get(key, 0) + 1
        self._range_frame_mismatch_counts[key] = count
        if count in (1, 10, 50, 100) or count % 200 == 0:
            self.get_logger().warn(
                'Ultrasonic frame mismatch '
                f'(topic_idx={topic_index}, expected="{key[1]}", received="{key[2]}", count={count}).'
            )

    def _lock_auto_target(self):
        """Select a robust target from observed scan lengths and lock it."""
        self._auto_target_locked = True
        samples = list(self._auto_target_samples)
        if len(samples) < self._auto_target_min_samples:
            self.target_readings = self._fixed_target_readings
            self.get_logger().warn(
                'Auto target lock fallback: insufficient valid scan samples '
                f'({len(samples)}/{self._auto_target_min_samples}). '
                f'Using fixed target {self._fixed_target_readings}.'
            )
            return

        center = int(round(median(samples)))
        filtered = [
            count for count in samples
            if abs(count - center) <= self._auto_target_outlier_tolerance
        ]

        if len(filtered) < self._auto_target_min_samples:
            self.target_readings = self._fixed_target_readings
            self.get_logger().warn(
                'Auto target lock fallback: outlier filtering removed too many samples '
                f'({len(filtered)}/{self._auto_target_min_samples}, center={center}, '
                f'tolerance={self._auto_target_outlier_tolerance}). '
                f'Using fixed target {self._fixed_target_readings}.'
            )
            return

        counts = Counter(filtered)
        selected = sorted(counts.items(), key=lambda kv: (-kv[1], abs(kv[0] - center), kv[0]))[0][0]
        self.target_readings = int(selected)

        observed_summary = ', '.join(
            f'{value}:{freq}'
            for value, freq in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:6]
        )
        self.get_logger().info(
            'Auto target lock complete: '
            f'selected target_readings={self.target_readings} '
            f'from {len(filtered)}/{len(samples)} filtered samples '
            f'(center={center}, tolerance={self._auto_target_outlier_tolerance}). '
            f'Observed counts: {observed_summary}'
        )
    
    def _stamp_to_seconds(self, stamp):
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def _is_valid_ultrasonic_range(self, value):
        if not math.isfinite(value):
            return False
        if value < self._ultrasonic_min_valid_range:
            return False
        if value > self._ultrasonic_max_valid_range:
            return False
        return True

    def _build_ultrasonic_candidate(self, sensor_index, range_msg, normalized_msg):
        # ignore stale data to avoid painting ghosts in front of the robot
        age_sec = self.get_clock().now().nanoseconds * 1e-9 - self._stamp_to_seconds(range_msg.header.stamp)
        if age_sec > self._ultrasonic_max_age_sec:
            return None
        if not self._is_valid_ultrasonic_range(range_msg.range):
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self._scan_frame,
                range_msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        sx = tf.transform.translation.x
        sy = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        r = range_msg.range
        ox = sx + r * math.cos(yaw)
        oy = sy + r * math.sin(yaw)
        angle_center = math.atan2(oy, ox)
        dist = math.sqrt(ox**2 + oy**2)
        fov = getattr(range_msg, "field_of_view", 0.2)
        return {
            'sensor_index': sensor_index,
            'dist': dist,
            'angle_center': angle_center,
            'fov': fov,
            'cone_scale': self._ultrasonic_cone_scale,
        }

    def _apply_overlap_arbitration(self, candidates):
        if not self._ultrasonic_overlap_arbitration_enabled:
            return candidates

        cand_by_sensor = {c['sensor_index']: c for c in candidates}
        left = cand_by_sensor.get(0)
        front = cand_by_sensor.get(1)
        right = cand_by_sensor.get(2)
        if front is None:
            return candidates

        similarity = self._ultrasonic_overlap_similarity_m
        left_sim = (
            left is not None and abs(left['dist'] - front['dist']) <= similarity
        )
        right_sim = (
            right is not None and abs(right['dist'] - front['dist']) <= similarity
        )

        if left_sim and right_sim and left is not None and right is not None:
            # All three similar => likely a front obstacle. Keep one centered cone.
            front['dist'] = min(front['dist'], left['dist'], right['dist'])
            return [front]

        if left_sim and not right_sim and left is not None:
            # Left+front pair => likely obstacle is on left side. Keep left dominant,
            # and keep front only as a narrow weak contribution for safety.
            front_copy = dict(front)
            front_copy['cone_scale'] *= self._ultrasonic_overlap_side_pair_front_scale
            front_copy['dist'] = max(front_copy['dist'], left['dist'])
            return [left, front_copy]

        if right_sim and not left_sim and right is not None:
            # Right+front pair => likely obstacle is on right side.
            front_copy = dict(front)
            front_copy['cone_scale'] *= self._ultrasonic_overlap_side_pair_front_scale
            front_copy['dist'] = max(front_copy['dist'], right['dist'])
            return [right, front_copy]

        return candidates

    def range_callback(self, msg, topic_index):
        if topic_index < 0 or topic_index >= len(self._range_msgs):
            return

        expected_frame = ''
        if topic_index < len(self._range_frame_ids):
            expected_frame = str(self._range_frame_ids[topic_index]).strip()
        msg_frame = str(msg.header.frame_id).strip()
        if expected_frame:
            # Enforce stable frame IDs so TF lookups stay deterministic across robots.
            if not msg_frame:
                msg.header.frame_id = expected_frame
            elif not self._frame_ids_match(expected_frame, msg_frame):
                self._record_frame_mismatch(topic_index, expected_frame, msg_frame)
                return

        if not self._is_valid_ultrasonic_range(msg.range):
            return

        self._range_windows[topic_index].append(float(msg.range))
        filtered_range = float(median(self._range_windows[topic_index]))
        previous_filtered = self._range_previous_filtered[topic_index]
        stamp_s = self._stamp_to_seconds(msg.header.stamp)
        if previous_filtered is not None:
            delta = filtered_range - previous_filtered
            last_change_s = self._range_last_change_time[topic_index]
            hold_age_s = (
                0.0 if last_change_s is None
                else max(0.0, stamp_s - last_change_s)
            )
            # Reject only implausible *increases* in one update (typical of multipath/echo
            # artifacts on glossy surfaces / corners). Always allow large *decreases*,
            # which is how "something just appeared" shows up in ultrasonics.
            if (
                abs(delta) > self._ultrasonic_max_delta_per_update
                and delta > 0.0
            ):
                if hold_age_s < self._ultrasonic_max_hold_sec:
                    return
                filtered_range = min(
                    filtered_range,
                    previous_filtered + self._ultrasonic_max_delta_per_update
                )
                delta = filtered_range - previous_filtered
            if abs(delta) < self._ultrasonic_hysteresis_m and hold_age_s < self._ultrasonic_max_hold_sec:
                filtered_range = previous_filtered

        if previous_filtered is None or abs(filtered_range - previous_filtered) > self._ultrasonic_hold_epsilon_m:
            self._range_last_change_time[topic_index] = stamp_s
        self._range_previous_filtered[topic_index] = filtered_range
        filtered_msg = Range()
        filtered_msg.header = msg.header
        filtered_msg.radiation_type = msg.radiation_type
        filtered_msg.field_of_view = msg.field_of_view
        filtered_msg.min_range = msg.min_range
        filtered_msg.max_range = msg.max_range
        filtered_msg.range = filtered_range
        self._range_msgs[topic_index] = filtered_msg
    
    def scan_callback(self, msg):
        # Throttle: only process and publish every Nth scan to reduce SLAM message filter queue overflow
        self._scan_counter += 1
        if self._scan_counter < self.publish_every_n:
            return
        self._scan_counter = 0

        actual_readings = len(msg.ranges)

        if self._auto_target_enabled and not self._auto_target_locked:
            self._auto_scans_seen += 1
            if actual_readings > 0:
                self._auto_target_samples.append(actual_readings)
            if self._auto_scans_seen >= self._auto_target_sample_scans:
                self._lock_auto_target()
            else:
                # Do not publish until the target is locked; slam_toolbox latches the
                # first observed beam count and warns forever if it later changes.
                if (
                    self._auto_scans_seen == 1
                    or self._auto_scans_seen % self._auto_warmup_log_mod == 0
                ):
                    self.get_logger().info(
                        'Auto target warmup: collected '
                        f'{self._auto_scans_seen}/{self._auto_target_sample_scans} '
                        f'scan lengths; delaying publish until lock.'
                    )
                return
        
        # Create a copy of the message
        normalized_msg = LaserScan()
        normalized_msg.header = msg.header
        if self._frame_id_prefix:
            normalized_msg.header.frame_id = f"{self._frame_id_prefix}/base_scan"
        normalized_msg.angle_min = msg.angle_min
        normalized_msg.angle_max = msg.angle_max
        normalized_msg.angle_increment = msg.angle_increment
        normalized_msg.time_increment = msg.time_increment
        normalized_msg.scan_time = msg.scan_time
        normalized_msg.range_min = msg.range_min
        normalized_msg.range_max = msg.range_max
        
        # Debug: log first few scans to verify normalization is working
        if not hasattr(self, '_scan_count'):
            self._scan_count = 0
        self._scan_count += 1
        if self._scan_count <= 5:
            self.get_logger().info(
                f'Scan {self._scan_count}: received {actual_readings} readings, '
                f'normalizing to {self.target_readings}'
            )
        
        if actual_readings == self.target_readings:
            # Already correct size, just copy
            normalized_msg.ranges = list(msg.ranges)
            normalized_msg.intensities = list(msg.intensities) if msg.intensities else []
        elif actual_readings > self.target_readings:
            # Too many readings - downsample by selecting evenly spaced indices
            step = (actual_readings - 1) / (self.target_readings - 1) if self.target_readings > 1 else 0
            normalized_msg.ranges = []
            normalized_msg.intensities = []
            
            for i in range(self.target_readings):
                idx = int(round(i * step))
                if idx >= actual_readings:
                    idx = actual_readings - 1
                normalized_msg.ranges.append(msg.ranges[idx])
                if msg.intensities and len(msg.intensities) > idx:
                    normalized_msg.intensities.append(msg.intensities[idx])
        else:
            # Too few readings - upsample by linear interpolation
            normalized_msg.ranges = []
            normalized_msg.intensities = []
            
            if actual_readings == 0:
                # No readings - fill with max range
                normalized_msg.ranges = [msg.range_max] * self.target_readings
                normalized_msg.intensities = [0.0] * self.target_readings if msg.intensities else []
            elif actual_readings == 1:
                # Single reading - replicate it
                normalized_msg.ranges = [msg.ranges[0]] * self.target_readings
                normalized_msg.intensities = [msg.intensities[0] if msg.intensities else 0.0] * self.target_readings
            else:
                # Linear interpolation
                step = (actual_readings - 1) / (self.target_readings - 1) if self.target_readings > 1 else 0
                for i in range(self.target_readings):
                    pos = i * step
                    idx_low = int(pos)
                    idx_high = min(idx_low + 1, actual_readings - 1)
                    alpha = pos - idx_low
                    
                    # Interpolate range
                    range_val = msg.ranges[idx_low] * (1 - alpha) + msg.ranges[idx_high] * alpha
                    normalized_msg.ranges.append(range_val)
                    
                    # Interpolate intensity if available
                    if msg.intensities and len(msg.intensities) > idx_high:
                        intensity_val = msg.intensities[idx_low] * (1 - alpha) + msg.intensities[idx_high] * alpha
                        normalized_msg.intensities.append(intensity_val)
                    elif msg.intensities:
                        normalized_msg.intensities.append(msg.intensities[0] if len(msg.intensities) > 0 else 0.0)
                    else:
                        normalized_msg.intensities.append(0.0)
        
        # Ensure intensities array matches ranges array length
        if msg.intensities and len(normalized_msg.intensities) != len(normalized_msg.ranges):
            # Pad or truncate intensities to match
            if len(normalized_msg.intensities) < len(normalized_msg.ranges):
                normalized_msg.intensities.extend([0.0] * (len(normalized_msg.ranges) - len(normalized_msg.intensities)))
            else:
                normalized_msg.intensities = normalized_msg.intensities[:len(normalized_msg.ranges)]
        elif not msg.intensities and len(normalized_msg.ranges) > 0:
            # Create empty intensities array if not present
            normalized_msg.intensities = []
        
        # Adjust angle_increment to match the new number of readings
        # This ensures the scan geometry is correct
        angle_span = msg.angle_max - msg.angle_min
        normalized_msg.angle_increment = angle_span / (self.target_readings - 1) if self.target_readings > 1 else msg.angle_increment
        
        # add in ultrasonic ranges
        if self._ultrasonic_fusion_enabled:
            fusion_candidates = []
            for sensor_index, range_msg in enumerate(self._range_msgs):
                if not self._ultrasonic_use.get(sensor_index, True):
                    continue
                if range_msg is None:
                    continue
                candidate = self._build_ultrasonic_candidate(sensor_index, range_msg, normalized_msg)
                if candidate is not None:
                    fusion_candidates.append(candidate)

            fusion_candidates = self._apply_overlap_arbitration(fusion_candidates)

            for candidate in fusion_candidates:
                angle_center = candidate['angle_center']
                dist = candidate['dist']
                half_fov = (candidate['fov'] * candidate['cone_scale']) * 0.5

                # convert angular spread → scan index spread
                angle_min = normalized_msg.angle_min
                angle_inc = normalized_msg.angle_increment
                scan_len = len(normalized_msg.ranges)

                # convert bounds in angle space
                start_angle = angle_center - half_fov
                end_angle = angle_center + half_fov

                start_idx = int((start_angle - angle_min) / angle_inc)
                end_idx = int((end_angle - angle_min) / angle_inc)

                # clamp
                start_idx = max(0, start_idx)
                end_idx = min(scan_len - 1, end_idx)

                # --- fill cone with conservative lidar-agreement gating ---
                for i in range(start_idx, end_idx + 1):
                    current = normalized_msg.ranges[i]
                    if not math.isfinite(current):
                        normalized_msg.ranges[i] = dist
                        continue
                    if dist + self._ultrasonic_lidar_min_override_delta < current:
                        normalized_msg.ranges[i] = dist

            

        # CRITICAL: Verify we have exactly the target number of readings
        # This is a safety check - the code above should always produce exactly target_readings
        if len(normalized_msg.ranges) != self.target_readings:
            self.get_logger().error(
                f'CRITICAL: Normalization failed! Expected {self.target_readings} readings, '
                f'got {len(normalized_msg.ranges)}. Fixing by padding/truncating...'
            )
            # Emergency fix: pad or truncate to exact count
            if len(normalized_msg.ranges) < self.target_readings:
                # Pad with last value
                last_val = normalized_msg.ranges[-1] if normalized_msg.ranges else msg.range_max
                normalized_msg.ranges.extend([last_val] * (self.target_readings - len(normalized_msg.ranges)))
                if normalized_msg.intensities:
                    last_int = normalized_msg.intensities[-1] if normalized_msg.intensities else 0.0
                    normalized_msg.intensities.extend([last_int] * (self.target_readings - len(normalized_msg.intensities)))
            else:
                # Truncate
                normalized_msg.ranges = normalized_msg.ranges[:self.target_readings]
                if normalized_msg.intensities:
                    normalized_msg.intensities = normalized_msg.intensities[:self.target_readings]
        
        # Debug: log first few normalized scans
        if self._scan_count <= 5:
            self.get_logger().info(
                f'Scan {self._scan_count}: published {len(normalized_msg.ranges)} readings '
                f'(target: {self.target_readings})'
            )
        
        # Publish the normalized scan
        self.publisher.publish(normalized_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNormalizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        # Avoid double shutdown: on Ctrl+C the context may already be shutting down
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()