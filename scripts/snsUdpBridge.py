from rclpy.node import Node
import rclpy
import socket
import json
import time
import threading
import queue
from datetime import datetime
import math
import os
import getpass

from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# -----------------------------
# Config
# -----------------------------
LOGSTASH_IP = "192.168.0.76"

# ROS topic events
ROS_LOGSTASH_PORT = 5045

# ARP cache events
ARP_LOGSTASH_PORT = 5000
ARP_INTERVAL = 5.0

QUEUE_SIZE = 5000

def get_robot_name():
    return os.getenv("SUDO_USER") or getpass.getuser()

ROBOT_NAME = get_robot_name()

# Real ROS topics on the robot
BATTERY_TOPIC = f'/{ROBOT_NAME}/battery_state'
ODOM_TOPIC = f'/{ROBOT_NAME}/odom'
CMD_VEL_TOPIC = f'/{ROBOT_NAME}/cmd_vel'

THROTTLE = {
    BATTERY_TOPIC: 5.0,
    ODOM_TOPIC: 0.1,
    CMD_VEL_TOPIC: 0.1,
}

# -----------------------------
def get_local_ip_for_destination(dest_ip, dest_port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dest_ip, dest_port))
        return s.getsockname()[0]
    finally:
        s.close()

def strip_namespace(topic, robot_name):
    prefix = f'/{robot_name}'
    if topic.startswith(prefix):
        cleaned = topic[len(prefix):]
        return cleaned if cleaned else "/"
    return topic

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

def get_arp_table():
    arp_entries = []
    try:
        with open("/proc/net/arp") as f:
            next(f)
            for line in f:
                parts = line.split()
                if len(parts) >= 6:
                    arp_entries.append({
                        "ip": parts[0],
                        "mac": parts[3],
                        "interface": parts[5]
                    })
    except Exception:
        pass
    return arp_entries

# -----------------------------
class LogstashPublisher(Node):
    def __init__(self):
        super().__init__('ros2_logstash_bridge')

        self.robot_name = ROBOT_NAME
        self.local_ip = get_local_ip_for_destination(LOGSTASH_IP, ROS_LOGSTASH_PORT)

        self.ros_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ros_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ros_sock.bind((self.local_ip, 0))

        self.arp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.arp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.arp_sock.bind((self.local_ip, 0))

        self.get_logger().info(f"Robot name: {self.robot_name}")
        self.get_logger().info(f"Subscribing to: {BATTERY_TOPIC}")
        self.get_logger().info(f"Subscribing to: {ODOM_TOPIC}")
        self.get_logger().info(f"Subscribing to: {CMD_VEL_TOPIC}")
        self.get_logger().info(
            f"ROS UDP socket bound to {self.local_ip}, sending to {LOGSTASH_IP}:{ROS_LOGSTASH_PORT}"
        )
        self.get_logger().info(
            f"ARP UDP socket bound to {self.local_ip}, sending to {LOGSTASH_IP}:{ARP_LOGSTASH_PORT}"
        )

        self.queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.last_sent = {}
        self.running = True

        self.sender_thread = threading.Thread(
            target=self.sender_loop,
            daemon=True
        )
        self.sender_thread.start()

        self.arp_thread = threading.Thread(
            target=self.arp_loop,
            daemon=True
        )
        self.arp_thread.start()

        self.create_subscription(BatteryState, BATTERY_TOPIC, self.battery_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_cb, 10)

    # -----------------------------
    # ROS sender path
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

        clean_topic = strip_namespace(topic, self.robot_name)

        event = {
            "@timestamp": datetime.utcnow().isoformat() + "Z",
            "robot": self.robot_name,
            "topic": clean_topic,
            "data": data
        }

        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self.get_logger().warn("Queue full - dropping ROS message")

    def sender_loop(self):
        while self.running:
            event = self.queue.get()
            try:
                payload = (json.dumps(event) + "\n").encode()
                self.ros_sock.sendto(payload, (LOGSTASH_IP, ROS_LOGSTASH_PORT))
            except Exception as e:
                self.get_logger().error(f"ROS UDP send failed: {e}")

    # -----------------------------
    # ARP sender path
    def send_arp_snapshot(self):
        payload = {
            "robot": self.robot_name,
            "arp_host": self.robot_name,
            "source_ip": self.local_ip,
            "timestamp": time.time(),
            "arp_table": get_arp_table()
        }

        try:
            data = json.dumps(payload).encode()
            self.arp_sock.sendto(data, (LOGSTASH_IP, ARP_LOGSTASH_PORT))
            self.get_logger().info("Sent ARP data over UDP")
        except Exception as e:
            self.get_logger().error(f"ARP UDP send failed: {e}")

    def arp_loop(self):
        while self.running:
            self.send_arp_snapshot()
            time.sleep(ARP_INTERVAL)

    # -----------------------------
    # ROS callbacks
    def battery_cb(self, msg):
        self.enqueue(BATTERY_TOPIC, {
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

        self.enqueue(ODOM_TOPIC, {
            "pos": [p.x, p.y, p.z],
            "ori": euler,
            "lin_vel": [t.x, t.y, t.z],
            "ang_vel": [r.x, r.y, r.z]
        })

    def cmd_vel_cb(self, msg):
        self.enqueue(CMD_VEL_TOPIC, {
            "linear": [msg.linear.x, msg.linear.y, msg.linear.z],
            "angular": [msg.angular.x, msg.angular.y, msg.angular.z]
        })

    # -----------------------------
    def close(self):
        self.running = False
        try:
            self.ros_sock.close()
        except Exception:
            pass
        try:
            self.arp_sock.close()
        except Exception:
            pass

# -----------------------------
def main():
    rclpy.init()
    node = LogstashPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
