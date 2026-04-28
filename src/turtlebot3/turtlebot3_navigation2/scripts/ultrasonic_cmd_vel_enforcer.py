#!/usr/bin/env python3
"""Publish zero cmd_vel when ultrasonic hazard clusters are active."""

import json
from typing import Optional, Set

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class UltrasonicCmdVelEnforcer(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_cmd_vel_enforcer')

        self.declare_parameter('triangulation_debug_topic', 'ultrasonic_triangulation_debug')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('hold_sec', 0.7)
        self.declare_parameter(
            'hazard_clusters',
            [
                'front_guard',
                'front_guard_held',
                'front_emergency',
                'front_emergency_held',
                'front_emergency_replay',
                'front_single',
            ],
        )
        self.declare_parameter(
            'always_stop_clusters',
            [
                'front_emergency',
                'front_emergency_held',
                'front_emergency_replay',
                'front_single',
                'front_single_held',
            ],
        )
        self.declare_parameter(
            'guarded_stop_clusters',
            ['front_guard', 'front_guard_held'],
        )
        self.declare_parameter('guarded_stop_max_blob_dist_m', 0.32)

        tri_topic = str(self.get_parameter('triangulation_debug_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._publish_hz = max(2.0, float(self.get_parameter('publish_hz').value))
        self._hold_sec = max(0.0, float(self.get_parameter('hold_sec').value))
        self._hazard_clusters: Set[str] = {
            str(x).strip() for x in list(self.get_parameter('hazard_clusters').value) if str(x).strip()
        }
        self._always_stop_clusters: Set[str] = {
            str(x).strip() for x in list(self.get_parameter('always_stop_clusters').value) if str(x).strip()
        }
        self._guarded_stop_clusters: Set[str] = {
            str(x).strip() for x in list(self.get_parameter('guarded_stop_clusters').value) if str(x).strip()
        }
        self._guarded_stop_max_blob_dist_m = max(
            0.05, float(self.get_parameter('guarded_stop_max_blob_dist_m').value)
        )

        self._last_hazard_time_s: Optional[float] = None
        self._last_state: Optional[bool] = None
        self._last_cluster: Optional[str] = None

        self.create_subscription(String, tri_topic, self._on_triangulation_debug, 20)
        self._pub = self.create_publisher(Twist, self._cmd_vel_topic, 20)
        self.create_timer(1.0 / self._publish_hz, self._tick)

        self.get_logger().info(
            f'Ultrasonic cmd_vel enforcer started: topic={self._cmd_vel_topic} hold_sec={self._hold_sec:.2f} '
            f'hazard_clusters={sorted(self._hazard_clusters)}'
        )

    def _on_triangulation_debug(self, msg: String) -> None:
        text = (msg.data or '').strip()
        if not text:
            return
        try:
            data = json.loads(text)
            if not isinstance(data, dict):
                return
        except Exception:
            return
        cluster = str(data.get('cluster') or '').strip()
        if not cluster:
            return
        self._last_cluster = cluster
        blob_dist = data.get('blob_dist_m')
        dist_m = None
        if blob_dist is not None:
            try:
                dist_m = float(blob_dist)
            except Exception:
                dist_m = None

        should_stop = False
        if cluster in self._always_stop_clusters:
            should_stop = True
        elif cluster in self._guarded_stop_clusters:
            should_stop = dist_m is not None and dist_m <= self._guarded_stop_max_blob_dist_m
        elif cluster in self._hazard_clusters:
            should_stop = True

        if should_stop:
            self._last_hazard_time_s = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        active = (
            self._last_hazard_time_s is not None
            and (now_s - self._last_hazard_time_s) <= self._hold_sec
        )
        if active:
            # Hard stop override: repeatedly publish zero twist while hazard active.
            self._pub.publish(Twist())
        if self._last_state is None or self._last_state != active:
            self.get_logger().info(
                f'ultrasonic_stop_active={active} cluster={self._last_cluster or "unknown"}'
            )
        self._last_state = active


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicCmdVelEnforcer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
