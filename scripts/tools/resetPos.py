import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
import math


class CmdVelIntegrator(Node):

    def __init__(self):
        super().__init__('cmd_vel_integrator')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/cmd_vel_intended_pose',
            10)

        # Reset service
        self.reset_service = self.create_service(
            Empty,
            '/reset_intended_pose',
            self.reset_callback)

        self.reset_pose()

    def reset_pose(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.get_logger().info("Intended pose reset to zero.")

    def reset_callback(self, request, response):
        self.reset_pose()
        return response

    def cmd_callback(self, msg: Twist):

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        v = msg.linear.x
        omega = msg.angular.z

        # Integrate
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        pose_msg = Float64MultiArray()
        pose_msg.data = [self.x, self.y, self.theta]
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
