from rclpy.node import Node
import rclpy
import socket
import json
import time
import threading
import queue
from datetime import datetime
import math

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# -----------------------------
# Config
# -----------------------------
PI4_LAN_IP = "192.168.0.194"
LOGSTASH_IP = "192.168.0.76"
LOGSTASH_PORT = 5045

ROBOT_NAME = "pinky"

QUEUE_SIZE = 5000

THROTTLE = {
    "/odom": 0.1,
    "/battery_state": 5.0,
    "/cmd_vel": 0.1,
}

# -----------------------------
def quaternion_to_euler(o):
    sinr_cosp = 2 * (o.w * o.x + o.y * o.z)
    cosr_cosp = 1 - 2 * (o.x * o.x + o.y * o.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (o.w * o.y - o.z * o.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2 * (o.w * o.z + o.x * o.y)
    cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]

# -----------------------------
class LogstashPublisher(Node):
    def __init__(self):
        super().__init__('ros2_logstash_bridge')

        # ✅ UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Allow reuse (safe for ROS2 coexistence)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to LAN interface only (prevents DDS issues)
        self.sock.bind((PI4_LAN_IP, 0))  # 0 = OS chooses source port

        self.get_logger().info(
            f"UDP socket bound to {PI4_LAN_IP}, sending to {LOGSTASH_IP}:{LOGSTASH_PORT}"
        )

        self.queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.last_sent = {}

        self.sender_thread = threading.Thread(
            target=self.sender_loop,
            daemon=True
        )
        self.sender_thread.start()

        # Subscriptions
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    # -----------------------------
    def allowed(self, topic):
        now = time.time()
        last = self.last_sent.get(topic, 0)
        if now - last >= THROTTLE[topic]:
            self.last_sent[topic] = now
            return True
        return False

    def enqueue(self, topic, data):
        if not self.allowed(topic):
            return

        event = {
            "@timestamp": datetime.utcnow().isoformat() + "Z",
            "robot": ROBOT_NAME,
            "topic": topic,
            "data": data
        }

        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self.get_logger().warn("Queue full — dropping message")

    def sender_loop(self):
        while True:
            event = self.queue.get()
            try:
                payload = (json.dumps(event) + "\n").encode()
                self.sock.sendto(payload, (LOGSTASH_IP, LOGSTASH_PORT))
            except Exception as e:
                self.get_logger().error(f"UDP send failed: {e}")

    # -----------------------------
    # Callbacks
    def battery_cb(self, msg):
        self.enqueue("/battery_state", {
            "voltage": msg.voltage,
            "current": msg.current,
            "percentage": msg.percentage,
            "present": msg.present
        })

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t = msg.twist.twist.linear
        r = msg.twist.twist.angular

        euler = quaternion_to_euler(o)

        self.enqueue("/odom", {
            "pos": [p.x, p.y, p.z],
            "ori": euler,
            "lin_vel": [t.x, t.y, t.z],
            "ang_vel": [r.x, r.y, r.z]
        })

    def cmd_vel_cb(self, msg):
        self.enqueue("/cmd_vel", {
            "linear": [msg.linear.x, msg.linear.y, msg.linear.z],
            "angular": [msg.angular.x, msg.angular.y, msg.angular.z]
        })

# -----------------------------
def main():
    rclpy.init()
    node = LogstashPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


