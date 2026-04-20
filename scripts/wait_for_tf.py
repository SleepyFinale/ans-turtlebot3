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

Fleet (central map_merge): after odom->base succeeds, optionally wait for
world_frame -> <robot>/map on global /tf so Nav2's ``map`` frame exists before
lifecycle activation::

  TF_WAIT_FLEET_WORLD_MAP_FRAME=pinky/map
  TF_WAIT_FLEET_MAP_TIMEOUT_SEC=120   # optional; default 120

For namespaced setups, set:
  TF_WAIT_ODOM_FRAME=blinky/odom
  TF_WAIT_BASE_FRAMES=blinky/base_footprint,blinky/base_link
  TF_WAIT_NAMESPACE=blinky   (subscribes to /blinky/tf in addition to /tf)
  TF_WAIT_MAP_FRAME=blinky/map   (only used when TF_WAIT_ODOM_ONLY is false)
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage


TF_STATIC_QOS = QoSProfile(
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class TfWaiter(Node):
    def __init__(self, namespace='') -> None:
        super().__init__("wait_for_tf")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        if namespace:
            self.create_subscription(
                TFMessage, f'/{namespace}/tf',
                lambda msg: self._relay_tf(msg, static=False), 100)
            self.create_subscription(
                TFMessage, f'/{namespace}/tf_static',
                lambda msg: self._relay_tf(msg, static=True), TF_STATIC_QOS)

    def _relay_tf(self, msg, static=False):
        """Forward namespaced TF into the local buffer so lookups succeed."""
        for t in msg.transforms:
            if static:
                self._buffer.set_transform_static(t, 'wait_for_tf_relay')
            else:
                self._buffer.set_transform(t, 'wait_for_tf_relay')

    def wait_for(self, target: str, source: str, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            if self._buffer.can_transform(target, source, rclpy.time.Time()):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_fresh(
        self,
        target: str,
        source: str,
        timeout_sec: float,
        max_age_sec: float,
        stable_samples: int,
    ) -> bool:
        """Wait until transform exists and is fresh for N consecutive checks."""
        start = time.time()
        fresh_ok = 0
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self._buffer.lookup_transform(
                    target, source, rclpy.time.Time())
            except Exception:
                fresh_ok = 0
                continue

            stamp = float(t.header.stamp.sec) + (float(t.header.stamp.nanosec) / 1e9)
            now = self.get_clock().now().nanoseconds / 1e9
            age = max(0.0, now - stamp)
            if age <= max_age_sec:
                fresh_ok += 1
                if fresh_ok >= max(1, stable_samples):
                    return True
            else:
                fresh_ok = 0
        return False


def main() -> int:
    map_frame = os.environ.get("TF_WAIT_MAP_FRAME", "map")
    odom_frame = os.environ.get("TF_WAIT_ODOM_FRAME", "odom")
    base_candidates = os.environ.get("TF_WAIT_BASE_FRAMES", "base_footprint,base_link").split(",")
    timeout = float(os.environ.get("TF_WAIT_TIMEOUT_SEC", "30.0"))
    odom_only = os.environ.get("TF_WAIT_ODOM_ONLY", "false").lower() in ("1", "true", "yes")
    namespace = os.environ.get("TF_WAIT_NAMESPACE", "")
    max_age_sec = float(os.environ.get("TF_WAIT_MAX_AGE_SEC", "0.0"))
    stable_samples = int(os.environ.get("TF_WAIT_STABLE_SAMPLES", "1"))

    rclpy.init()
    node = TfWaiter(namespace=namespace)

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
            if max_age_sec > 0.0:
                ok = node.wait_for_fresh(
                    map_frame,
                    odom_frame,
                    timeout_sec=timeout,
                    max_age_sec=max_age_sec,
                    stable_samples=stable_samples,
                )
            else:
                ok = node.wait_for(map_frame, odom_frame, timeout_sec=timeout)
            if not ok:
                node.get_logger().error(
                    f"Timed out waiting for fresh TF {map_frame} -> {odom_frame} "
                    f"(max_age_sec={max_age_sec}, stable_samples={stable_samples})"
                )
                return 1

        ok_base = False
        for base in base_candidates:
            base = base.strip()
            if not base:
                continue
            if max_age_sec > 0.0:
                ok = node.wait_for_fresh(
                    odom_frame,
                    base,
                    timeout_sec=timeout,
                    max_age_sec=max_age_sec,
                    stable_samples=stable_samples,
                )
            else:
                ok = node.wait_for(odom_frame, base, timeout_sec=timeout)
            if ok:
                node.get_logger().info(f"TF ready: {odom_frame} -> {base}")
                ok_base = True
                break

        if not ok_base:
            node.get_logger().error(
                f"Timed out waiting for TF {odom_frame} -> one of {base_candidates}. "
                "Check robot bringup is publishing base TF and ROS_DOMAIN_ID matches."
            )
            return 2

        fleet_map_child = os.environ.get("TF_WAIT_FLEET_WORLD_MAP_FRAME", "").strip()
        if fleet_map_child:
            fleet_timeout = float(
                os.environ.get("TF_WAIT_FLEET_MAP_TIMEOUT_SEC", "120.0"))
            node.get_logger().info(
                f"Waiting for fleet world TF: map -> {fleet_map_child} "
                f"(map_merge on central). Timeout: {fleet_timeout:.1f}s"
            )
            ok_map = node.wait_for("map", fleet_map_child, timeout_sec=fleet_timeout)
            if not ok_map:
                node.get_logger().error(
                    f"Timed out waiting for map -> {fleet_map_child}. "
                    "Start central start_central.sh (map_merge) before Nav2, or use "
                    "fleet_mode:=auto to block until the chain appears."
                )
                return 3

        node.get_logger().info("TF tree looks ready.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
