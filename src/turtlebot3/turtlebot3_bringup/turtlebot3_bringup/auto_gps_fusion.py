#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
import copy


class AutoGPSFusion(Node):

    def __init__(self):
        super().__init__('auto_gps_fusion')

        # Publisher (namespaced-safe topic)
        self.declare_parameter('output_topic', 'fix')
        output_topic = self.get_parameter('output_topic').value
        self.declare_parameter('namespace','')
        self.namespace = self.get_parameter('namespace').value
        self.declare_parameter('stale_timeout_sec', 1.5)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)

        # Storage for incoming GPS messages (index -> (msg, receive_time_sec))
        self.gps_msgs = {}

        self.get_logger().info("GPS fusion node started")

        # Subscriptions (fixed 2 inputs max)
        self.create_subscription(NavSatFix, 'gps1/fix', self.cb1, 10)
        self.create_subscription(NavSatFix, 'gps2/fix', self.cb2, 10)

    # -------------------------
    # Callbacks
    # -------------------------
    def cb1(self, msg):
        self.gps_msgs[0] = (msg, self.get_clock().now().nanoseconds / 1e9)
        self.publish_fused()

    def cb2(self, msg):
        self.gps_msgs[1] = (msg, self.get_clock().now().nanoseconds / 1e9)
        self.publish_fused()

    # -------------------------
    # Fusion logic
    # -------------------------
    def publish_fused(self):

        if len(self.gps_msgs) == 0:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        fresh_msgs = []
        for idx, (msg, recv_sec) in list(self.gps_msgs.items()):
            if (now_sec - recv_sec) <= self.stale_timeout_sec:
                fresh_msgs.append(msg)
            else:
                # Drop stale source so a dead/noisy receiver does not keep biasing fusion.
                self.gps_msgs.pop(idx, None)

        msgs = [m for m in fresh_msgs if m.status.status >= 0]
        if len(msgs) == 0:
            # Fallback: if both are reporting NO_FIX, pass through freshest anyway.
            msgs = fresh_msgs
        if len(msgs) == 0:
            return

        # 1 GPS → passthrough
        if len(msgs) == 1:
            self.publisher.publish(copy.deepcopy(msgs[0]))
            return

        # 2 GPS → weighted average
        m1, m2 = msgs[:2]

        fused = NavSatFix()
        newest = max(msgs[:2], key=lambda m: Time.from_msg(m.header.stamp).nanoseconds)
        fused.header.stamp = newest.header.stamp
        fused.header.frame_id = newest.header.frame_id or f"{self.namespace}/gps_link"

        # simple covariance weighting
        try:
            w1 = 1.0 / (m1.position_covariance[0] + 1e-6)
            w2 = 1.0 / (m2.position_covariance[0] + 1e-6)
        except:
            w1 = w2 = 1.0

        total = w1 + w2

        fused.latitude = (m1.latitude * w1 + m2.latitude * w2) / total
        fused.longitude = (m1.longitude * w1 + m2.longitude * w2) / total
        fused.altitude = (m1.altitude * w1 + m2.altitude * w2) / total

        fused.position_covariance = [
            (a + b) / 2.0 for a, b in zip(
                m1.position_covariance,
                m2.position_covariance
            )
        ]

        fused.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.publisher.publish(fused)


def main():
    rclpy.init()
    node = AutoGPSFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()