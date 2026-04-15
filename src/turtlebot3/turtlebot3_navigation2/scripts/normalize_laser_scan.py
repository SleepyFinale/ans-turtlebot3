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

        target_readings = self.get_parameter('target_readings').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_every_n = self.get_parameter('publish_every_n_scans').value
        self._frame_id_prefix = self.get_parameter('frame_id_prefix').value
        self._range_topics = self.get_parameter('range_topics').value
        self._range_frame_ids = self.get_parameter('range_frame_ids').value
        self._scan_counter = 0

        self._range_subs = []
        self._range_msgs = []
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

        for topic in self._range_topics:
            self._range_subs.append(
                self.create_subscription(
                Range,
                topic,
                self.range_callback,
                qos_profile_sensor_data
                )
            )
            self._range_msgs.append(None)
        
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
    
    def range_callback(self, msg):
        topic_index = self._range_frame_ids.index(msg.header.frame_id.replace(self._frame_id_prefix + '/',''))
        if topic_index != -1:
            self._range_msgs[topic_index] = msg
    
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
        for range_msg in self._range_msgs:
            if range_msg is None:
                continue

            # ignore invalid readings
            if range_msg.range < range_msg.min_range or range_msg.range > range_msg.max_range:
                continue

            try:
                tf = self.tf_buffer.lookup_transform(
                    self._frame_id_prefix + '/base_scan',
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
            half_fov = fov * 0.5

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

            # --- fill cone ---
            # for i in range(start_idx, end_idx + 1):
            #     # optional: taper edges (more realistic)
            #     normalized_msg.ranges[i] = min(
            #         normalized_msg.ranges[i],
            #         dist
            #     ) if not math.isinf(normalized_msg.ranges[i]) else dist

            

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