#!/usr/bin/env python3
"""Mark the trustworthy prefix of over-range lidar beams as free space.

slam_toolbox drops ranges beyond max_laser_range (8 m on the LDS-02) and does
not paint those rays. A finite reading longer than that, including the ~19 m
corridor beams, means no surface inside the range the lidar can trust. This
node copies each SLAM occupancy grid and writes unknown cells to free along
those rays out to 8 m.

Occupied cells stop a ray, so a miss cannot punch through a wall already seen
from another angle. The scan published to slam_toolbox is unchanged, so scan
matching never sees a fake hit at 8 m.
"""

from __future__ import annotations

import math
import threading
from array import array

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def clear_open_rays(
    data: array,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    resolution: float,
    ranges,
    angle_min: float,
    angle_increment: float,
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    clear_range: float,
) -> int:
    """Write unknown cells to free along finite beams longer than clear_range.

    Each ray travels at most clear_range metres and stops at the first occupied
    cell. Returns how many unknown cells were changed.
    """
    if resolution <= 0.0 or width <= 0 or height <= 0 or clear_range <= 0.0:
        return 0
    if len(data) != width * height:
        return 0

    # Grid axes are the map frame rotated by the occupancy origin yaw.
    cos_o = math.cos(origin_yaw)
    sin_o = math.sin(origin_yaw)
    step = resolution * 0.5
    changed = 0
    for i, raw in enumerate(ranges):
        try:
            reading = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(reading) or reading <= clear_range:
            continue
        heading = sensor_yaw + angle_min + i * angle_increment
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        dist = step
        prev_idx = -1
        while dist <= clear_range + 1e-6:
            wx = sensor_x + cos_h * dist
            wy = sensor_y + sin_h * dist
            dist += step
            dx = wx - origin_x
            dy = wy - origin_y
            gx = cos_o * dx + sin_o * dy
            gy = -sin_o * dx + cos_o * dy
            mx = int(math.floor(gx / resolution))
            my = int(math.floor(gy / resolution))
            if mx < 0 or my < 0 or mx >= width or my >= height:
                break
            idx = my * width + mx
            if idx == prev_idx:
                continue
            prev_idx = idx
            cell = data[idx]
            if cell > 0:
                break
            if cell < 0:
                data[idx] = 0
                changed += 1
    return changed


class OpenRayMapClear(Node):
    def __init__(self) -> None:
        super().__init__('open_ray_map_clear')
        self.declare_parameter('input_map_topic', 'map_slam')
        self.declare_parameter('output_map_topic', 'map')
        self.declare_parameter('scan_topic', 'scan_normalized')
        self.declare_parameter('clear_range', 8.0)
        self.declare_parameter('tf_timeout_sec', 0.05)

        in_map = self.get_parameter('input_map_topic').get_parameter_value().string_value
        out_map = self.get_parameter('output_map_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self._clear_range = float(self.get_parameter('clear_range').get_parameter_value().double_value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)
        )

        self._lock = threading.Lock()
        self._scan: LaserScan | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(OccupancyGrid, out_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, in_map, self._on_map, MAP_QOS)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Open-ray map clear: {in_map} + {scan_topic} -> {out_map}, '
            f'free out to {self._clear_range:.1f} m on ranges longer than that'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            scan = self._scan
        if scan is None or msg.info.resolution <= 0.0:
            self._pub.publish(msg)
            return

        pose = self._sensor_pose(msg.header.frame_id, scan)
        if pose is None:
            self._pub.publish(msg)
            return

        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        data = array('b', msg.data)
        origin = msg.info.origin
        oq = origin.orientation
        changed = clear_open_rays(
            data,
            int(msg.info.width),
            int(msg.info.height),
            float(origin.position.x),
            float(origin.position.y),
            quat_yaw(oq.x, oq.y, oq.z, oq.w),
            float(msg.info.resolution),
            scan.ranges,
            float(scan.angle_min),
            float(scan.angle_increment),
            pose[0],
            pose[1],
            pose[2],
            self._clear_range,
        )
        out.data = data
        self._pub.publish(out)
        if changed:
            self.get_logger().info(
                f'Cleared {changed} unknown cells along open rays '
                f'(limit {self._clear_range:.1f} m)',
                throttle_duration_sec=5.0,
            )

    def _sensor_pose(self, map_frame: str, scan: LaserScan):
        frame = scan.header.frame_id
        if not map_frame or not frame:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                map_frame,
                frame,
                Time.from_msg(scan.header.stamp),
                timeout=self._tf_timeout,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f'TF {map_frame} <- {frame} unavailable ({exc}); publishing map unchanged',
                throttle_duration_sec=5.0,
            )
            return None
        q = tf.transform.rotation
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            quat_yaw(q.x, q.y, q.z, q.w),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: OpenRayMapClear | None = None
    try:
        node = OpenRayMapClear()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
