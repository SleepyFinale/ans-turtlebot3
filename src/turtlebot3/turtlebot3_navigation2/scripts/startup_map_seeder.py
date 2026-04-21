#!/usr/bin/env python3
"""One-shot startup motion to improve early SLAM map quality."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


@dataclass
class Segment:
    duration_sec: float
    linear_x: float
    angular_z: float
    label: str


class StartupMapSeeder(Node):
    def __init__(self) -> None:
        super().__init__('startup_map_seeder')
        self.declare_parameter('controller_server_node', 'controller_server')
        self.declare_parameter('wait_timeout_sec', 60.0)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('rotate_speed_rad_s', 0.40)
        self.declare_parameter('forward_speed_m_s', 0.11)
        self.declare_parameter('reverse_speed_m_s', -0.08)

        rotate_speed = float(self.get_parameter('rotate_speed_rad_s').value)
        forward_speed = float(self.get_parameter('forward_speed_m_s').value)
        reverse_speed = float(self.get_parameter('reverse_speed_m_s').value)

        self._segments: List[Segment] = [
            Segment(4.0, 0.0, rotate_speed, 'rotate-left'),
            Segment(3.0, forward_speed, 0.0, 'forward'),
            Segment(4.0, 0.0, -rotate_speed, 'rotate-right'),
            Segment(2.0, reverse_speed, 0.0, 'reverse'),
            Segment(2.0, 0.0, 0.25, 'final-align'),
            Segment(1.0, 0.0, 0.0, 'stop'),
        ]

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        controller_node = str(self.get_parameter('controller_server_node').value).strip('/')
        self._wait_timeout_sec = max(1.0, float(self.get_parameter('wait_timeout_sec').value))
        self._publish_hz = max(2.0, float(self.get_parameter('publish_hz').value))

        # cmd_vel consumers (controller/velocity_smoother) generally use RELIABLE QoS.
        # Using sensor-data (best-effort) here can silently drop all commands.
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Twist, cmd_topic, cmd_qos)
        self._service_name = f'/{self.get_namespace().strip("/")}/{controller_node}/get_state'
        self._service_name = self._service_name.replace('//', '/')
        self._state_client = self.create_client(GetState, self._service_name)
        self._start_time = self.get_clock().now()
        self._segment_index = -1
        self._segment_started = self.get_clock().now()
        self._state = 'wait_service'

        self._timer = self.create_timer(1.0 / self._publish_hz, self._tick)
        self.get_logger().info(
            f'Startup map seeder active (ns={self.get_namespace()!r}, cmd_topic={cmd_topic}, '
            f'controller_state_service={self._service_name})'
        )

    def _timeout_hit(self) -> bool:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        return elapsed >= self._wait_timeout_sec

    def _publish_twist(self, x: float, z: float) -> None:
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self._pub.publish(msg)

    def _tick(self) -> None:
        if self._state == 'wait_service':
            if self._state_client.wait_for_service(timeout_sec=0.0):
                self.get_logger().info('controller_server lifecycle service is available; waiting for ACTIVE')
                self._state = 'wait_active'
                return
            if self._timeout_hit():
                self.get_logger().warn('Timeout waiting for controller_server service; ending seeding')
                self._finish()
            return

        if self._state == 'wait_active':
            req = GetState.Request()
            future = self._state_client.call_async(req)
            future.add_done_callback(self._on_state_response)
            self._state = 'wait_active_result'
            return

        if self._state == 'run':
            if self._segment_index >= len(self._segments):
                self._finish()
                return
            seg = self._segments[self._segment_index]
            seg_elapsed = (self.get_clock().now() - self._segment_started).nanoseconds / 1e9
            if seg_elapsed >= seg.duration_sec:
                self._segment_index += 1
                if self._segment_index >= len(self._segments):
                    self._finish()
                    return
                self._segment_started = self.get_clock().now()
                seg = self._segments[self._segment_index]
                self.get_logger().info(f'Running seeding segment: {seg.label}')
            self._publish_twist(seg.linear_x, seg.angular_z)
            return

    def _on_state_response(self, future) -> None:
        if self._state != 'wait_active_result':
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Lifecycle get_state failed: {exc}')
            if self._timeout_hit():
                self.get_logger().warn('Timeout while polling ACTIVE state; ending seeding')
                self._finish()
            else:
                self._state = 'wait_active'
            return

        if response.current_state.label.lower() == 'active':
            self.get_logger().info('controller_server is ACTIVE; beginning startup map seeding motion')
            self._segment_index = 0
            self._segment_started = self.get_clock().now()
            self._state = 'run'
            self.get_logger().info(f'Running seeding segment: {self._segments[0].label}')
        else:
            if self._timeout_hit():
                self.get_logger().warn(
                    f'Timeout waiting for ACTIVE (last state={response.current_state.label}); ending seeding'
                )
                self._finish()
            else:
                self._state = 'wait_active'

    def _finish(self) -> None:
        self._publish_twist(0.0, 0.0)
        self.get_logger().info('Startup map seeding complete')
        self.destroy_timer(self._timer)
        self.create_timer(0.2, self._shutdown_once)
        self._state = 'done'

    def _shutdown_once(self) -> None:
        self.destroy_node()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = StartupMapSeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
