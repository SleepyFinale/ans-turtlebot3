#!/usr/bin/env python3
"""
Wait for a usable TF tree before starting Nav2.

Why:
- Nav2 costmaps/controllers are lifecycle nodes and may fail to activate if the
  base/odom TF isn't available yet at startup.
- In SLAM mode, you typically need both:
    * map -> odom   (from slam_toolbox)
    * odom -> base_* (from robot bringup / odometry / robot_state_publisher)

This script blocks until those transforms are available (or times out).

Env: TF_WAIT_ODOM_ONLY=true to only wait for odom->base_* (robot). Use when
the launch starts SLAM itself so map->odom appears after SLAM Toolbox starts.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TfWaiter(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_tf")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

    def wait_for(self, target: str, source: str, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            if self._buffer.can_transform(target, source, rclpy.time.Time()):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def main() -> int:
    # Defaults chosen for TB3 + SLAM Toolbox exploration workflow
    map_frame = os.environ.get("TF_WAIT_MAP_FRAME", "map")
    odom_frame = os.environ.get("TF_WAIT_ODOM_FRAME", "odom")
    base_candidates = os.environ.get("TF_WAIT_BASE_FRAMES", "base_footprint,base_link").split(",")
    timeout = float(os.environ.get("TF_WAIT_TIMEOUT_SEC", "30.0"))
    # When true, only wait for odom->base_* (robot). Use when this launch starts SLAM itself.
    odom_only = os.environ.get("TF_WAIT_ODOM_ONLY", "false").lower() in ("1", "true", "yes")

    rclpy.init()
    node = TfWaiter()

    try:
        if odom_only:
            node.get_logger().info(
                f"Waiting for TF (odom only). Need {odom_frame}->(one of {base_candidates}). "
                f"Timeout: {timeout:.1f}s"
            )
        else:
            node.get_logger().info(
                f"Waiting for TF. Need {map_frame}->{odom_frame} and {odom_frame}->(one of {base_candidates}). "
                f"Timeout: {timeout:.1f}s"
            )

        if not odom_only:
            if not node.wait_for(map_frame, odom_frame, timeout_sec=timeout):
                node.get_logger().error(f"Timed out waiting for TF {map_frame} -> {odom_frame}")
                return 1

        ok_base = False
        for base in base_candidates:
            base = base.strip()
            if not base:
                continue
            if node.wait_for(odom_frame, base, timeout_sec=timeout):
                node.get_logger().info(f"TF ready: {odom_frame} -> {base}")
                ok_base = True
                break

        if not ok_base:
            node.get_logger().error(
                f"Timed out waiting for TF {odom_frame} -> one of {base_candidates}. "
                "Check robot bringup is publishing base TF and ROS_DOMAIN_ID matches."
            )
            return 2

        node.get_logger().info("TF tree looks ready.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

