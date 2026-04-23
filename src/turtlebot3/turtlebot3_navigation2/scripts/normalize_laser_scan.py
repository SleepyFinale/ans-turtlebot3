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
from statistics import median


class LaserScanNormalizer(Node):
    def __init__(self):
        super().__init__('laser_scan_normalizer')
        
        # Declare parameter for target number of readings
        # Default to 228 to match slam_toolbox's expected count
        self.declare_parameter('target_readings', 228)
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
        self.declare_parameter('ultrasonic_cone_scale', 1.0)

        target_readings = self.get_parameter('target_readings').value
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
        self._ultrasonic_cone_scale = max(
            0.1, float(self.get_parameter('ultrasonic_cone_scale').value))
        self._scan_counter = 0
        self._scan_frame = (
            f'{self._frame_id_prefix}/base_scan'
            if self._frame_id_prefix else 'base_scan'
        )

        self._range_subs = []
        self._range_msgs = []
        self._range_windows = []
        self._range_previous_filtered = []
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

        # Publish the normalized scan with sensor QoS
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data
        )
        
        self.target_readings = target_readings
        self.get_logger().info(
            f'Laser scan normalizer started: {input_topic} -> {output_topic} '
            f'(normalizing to {target_readings} readings, publishing every {self.publish_every_n} scan(s))'
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
            elif msg_frame != expected_frame:
                return

        if not self._is_valid_ultrasonic_range(msg.range):
            return

        self._range_windows[topic_index].append(float(msg.range))
        filtered_range = float(median(self._range_windows[topic_index]))
        previous_filtered = self._range_previous_filtered[topic_index]
        if previous_filtered is not None:
            delta = filtered_range - previous_filtered
            # Reject only implausible *increases* in one update (typical of multipath/echo
            # artifacts on glossy surfaces / corners). Always allow large *decreases*,
            # which is how "something just appeared" shows up in ultrasonics.
            if (
                abs(delta) > self._ultrasonic_max_delta_per_update
                and delta > 0.0
            ):
                return
            if abs(delta) < self._ultrasonic_hysteresis_m:
                filtered_range = previous_filtered

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
            for range_msg in self._range_msgs:
                if range_msg is None:
                    continue

                # ignore stale data to avoid painting ghosts in front of the robot
                age_sec = self.get_clock().now().nanoseconds * 1e-9 - self._stamp_to_seconds(range_msg.header.stamp)
                if age_sec > self._ultrasonic_max_age_sec:
                    continue

                # ignore invalid readings
                if not self._is_valid_ultrasonic_range(range_msg.range):
                    continue

                try:
                    tf = self.tf_buffer.lookup_transform(
                        self._scan_frame,
                        range_msg.header.frame_id,
                        rclpy.time.Time()
                    )
                except Exception as e:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                    continue

                # --- sensor position ---
                sx = tf.transform.translation.x
                sy = tf.transform.translation.y

                # --- yaw from quaternion ---
                q = tf.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )

                # --- range ---
                r = range_msg.range

                # --- center of detection in base_scan frame ---
                ox = sx + r * math.cos(yaw)
                oy = sy + r * math.sin(yaw)

                angle_center = math.atan2(oy, ox)
                dist = math.sqrt(ox**2 + oy**2)

                # --- FOV spreading ---
                fov = getattr(range_msg, "field_of_view", 0.2)  # fallback if missing
                half_fov = (fov * self._ultrasonic_cone_scale) * 0.5

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
    except (KeyboardInterrupt, ExternalShutdownException):
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