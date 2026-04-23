#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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

        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)

        # Storage for incoming GPS messages
        self.gps_msgs = {}

        self.get_logger().info("GPS fusion node started")

        # Subscriptions (fixed 2 inputs max)
        self.create_subscription(NavSatFix, 'gps1/fix', self.cb1, 10)
        self.create_subscription(NavSatFix, 'gps2/fix', self.cb2, 10)

    # -------------------------
    # Callbacks
    # -------------------------
    def cb1(self, msg):
        self.gps_msgs[0] = msg
        self.publish_fused()

    def cb2(self, msg):
        self.gps_msgs[1] = msg
        self.publish_fused()

    # -------------------------
    # Fusion logic
    # -------------------------
    def publish_fused(self):

        if len(self.gps_msgs) == 0:
            return

        msgs = list(self.gps_msgs.values())

        # 1 GPS → passthrough
        if len(msgs) == 1:
            self.publisher.publish(copy.deepcopy(msgs[0]))
            return

        # 2 GPS → weighted average
        m1, m2 = msgs[:2]

        fused = NavSatFix()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.header.frame_id = f"{self.namespace}/gps_link"

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