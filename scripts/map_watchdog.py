import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time as RosTime

class MapWatchdog(Node):
    def __init__(self, robot_name: str = "pinky"):
        super().__init__("map_watchdog")
        self.robot_name = robot_name
        self.map_topic = f"/{robot_name}/map"
        self.scan_topic = f"/{robot_name}/scan_normalized"
        self.cmd_vel_topic = f"/{robot_name}/cmd_vel"

        self.last_map_stamp: RosTime | None = None
        self.last_scan_stamp: RosTime | None = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(1.0, self._check_stall)
        self.stall_threshold_s = 2.0  # consider stalled if no map for > 2s

    def _on_map(self, msg: OccupancyGrid):
        self.last_map_stamp = msg.header.stamp

    def _on_scan(self, msg: LaserScan):
        self.last_scan_stamp = msg.header.stamp

    def _check_stall(self):
        now = self.get_clock().now().to_msg()
        if self.last_map_stamp is None or self.last_scan_stamp is None:
            return

        dt_map = (now.sec - self.last_map_stamp.sec) + \
                 (now.nanosec - self.last_map_stamp.nanosec) * 1e-9
        dt_scan = (now.sec - self.last_scan_stamp.sec) + \
                  (now.nanosec - self.last_scan_stamp.nanosec) * 1e-9

        # Only care if scans are recent but map is not
        if dt_scan < 1.0 and dt_map > self.stall_threshold_s:
            self.get_logger().warn(
                f"Map on {self.map_topic} has not updated for "
                f"{dt_map:.2f}s while scans are active (dt_scan={dt_scan:.2f}s). "
                "Sending a small spin to nudge SLAM."
            )
            twist = Twist()
            twist.angular.z = 0.2  # gentle spin in place
            self.cmd_pub.publish(twist)