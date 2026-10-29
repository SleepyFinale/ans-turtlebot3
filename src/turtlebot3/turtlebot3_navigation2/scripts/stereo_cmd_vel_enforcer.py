#!/usr/bin/env python3
"""Stop forward motion when stereo reports a close obstacle or a drop.

Only obstacles in the robot's own path count (path_obstacle_m: marks two
frames agree on, within path_half_width_m of the centre line). The gate waits
for a second consecutive frame so one bad match does not stop the robot. It
never stops while the depth node is uncalibrated.

The gate is a Bool (stereo/forward_block), true while a hazard was seen within
hold_sec. cmd_vel_mux clamps Nav2's forward speed to zero while it is true;
turning and reversing still pass, so a robot stopped facing an obstacle can
turn away from it. (This node used to publish zero cmd_vel next to Nav2's own
commands, and the base alternated between the two: a stutter, not a stop.)
"""

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class StereoCmdVelEnforcer(Node):
    def __init__(self) -> None:
        super().__init__('stereo_cmd_vel_enforcer')
        self.declare_parameter('debug_topic', 'stereo/debug')
        self.declare_parameter('gate_topic', 'stereo/forward_block')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('hold_sec', 0.50)
        self.declare_parameter('close_obstacle_m', 0.45)
        self.declare_parameter('required_streak', 2)

        self._publish_hz = max(2.0, float(self.get_parameter('publish_hz').value))
        self._hold_sec = max(0.0, float(self.get_parameter('hold_sec').value))
        self._close_m = float(self.get_parameter('close_obstacle_m').value)
        self._required_streak = max(1, int(self.get_parameter('required_streak').value))
        self._last_hazard_time_s: Optional[float] = None

        self.create_subscription(
            String, str(self.get_parameter('debug_topic').value), self._on_debug, 20)
        self._pub = self.create_publisher(Bool, str(self.get_parameter('gate_topic').value), 10)
        # The timer exists only while the gate is closed: a 10 Hz timer that
        # almost always found nothing to do cost CPU in rclpy.
        self._timer = None
        self.get_logger().info(
            f'Stereo stop gate started: close_m={self._close_m:.2f} streak={self._required_streak}')

    def _on_debug(self, msg: String) -> None:
        try:
            data = json.loads((msg.data or '').strip())
        except ValueError:
            return
        if not isinstance(data, dict) or not data.get('calibrated') or data.get('status') != 'ok':
            return
        try:
            obstacle_streak = int(data.get('obstacle_streak') or 0)
            drop_streak = int(data.get('drop_streak') or 0)
        except (TypeError, ValueError):
            return
        # Confirmed marks inside the robot's path only; the nearest hit
        # anywhere in view (obstacle_range_m) includes door jambs beside it.
        try:
            range_m = float(data['path_obstacle_m']) if data.get('path_obstacle_m') is not None else None
        except (TypeError, ValueError):
            range_m = None

        reason = ''
        if data.get('drop') and drop_streak >= self._required_streak:
            reason = 'drop'
        elif range_m is not None and range_m <= self._close_m and obstacle_streak >= self._required_streak:
            reason = 'close_obstacle'
        if not reason:
            return
        self._last_hazard_time_s = self._now()
        if self._timer is None:
            self.get_logger().info(f'stereo stop: forward blocked ({reason})')
            self._timer = self.create_timer(1.0 / self._publish_hz, self._tick)
        self._tick()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        active = (self._last_hazard_time_s is not None
                  and self._now() - self._last_hazard_time_s <= self._hold_sec)
        self._pub.publish(Bool(data=active))
        if not active and self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
            self.get_logger().info('stereo stop: released')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StereoCmdVelEnforcer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
