#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
import time
import threading
import queue
from datetime import datetime

# Message Types
from std_msgs.msg import Float64

# -----------------------------
# Config
# -----------------------------
INKY_IP = "192.168.0.139" 
LOGSTASH_IP = "192.168.0.76"  
LOGSTASH_PORT = 5045

ROBOT_NAME = "inky"
QUEUE_SIZE = 5000

# Set throttle to 0.0 because every single watermark value is critical
THROTTLE = {
    "/inky/watermark_delay": 0.0,
}

# -----------------------------
class InkyWatermarkBridge(Node):
    def __init__(self):
        super().__init__('inky_watermark_bridge')

        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.sock.bind((INKY_IP, 0))
        except Exception as e:
            self.get_logger().error(f"Could not bind to {INKY_IP}: {e}")

        self.get_logger().info(f"Watermark-only bridge sending to {LOGSTASH_IP}:{LOGSTASH_PORT}")

        self.queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.last_sent = {}

        # Background sender thread
        self.sender_thread = threading.Thread(target=self.sender_loop, daemon=True)
        self.sender_thread.start()

        # ONLY subscribing to the watermark delay
        self.create_subscription(Float64, '/inky/watermark_delay', self.delay_cb, 10)

    def allowed(self, topic):
        now = time.time()
        last = self.last_sent.get(topic, 0)
        if now - last >= THROTTLE.get(topic, 0.0):
            self.last_sent[topic] = now
            return True
        return False

    def enqueue(self, topic, data):
        if not self.allowed(topic):
            return

        clean_topic = topic.replace(f"/{ROBOT_NAME}/", "", 1)
        
        event = {
            "@timestamp": datetime.utcnow().isoformat() + "Z",
            "robot": ROBOT_NAME,
            "topic": clean_topic,
            "data": data
        }

        try:
            self.queue.put_nowait(event)
        except queue.Full:
            self.get_logger().warn("Queue full — dropping watermark message")

    def sender_loop(self):
        while True:
            event = self.queue.get()
            try:
                payload = (json.dumps(event) + "\n").encode()
                self.sock.sendto(payload, (LOGSTASH_IP, LOGSTASH_PORT))
            except Exception as e:
                self.get_logger().error(f"UDP send failed: {e}")

    def delay_cb(self, msg):
        # Package just the delay value
        self.enqueue("/inky/watermark_delay", {"delay_seconds": msg.data})

def main():
    rclpy.init()
    node = InkyWatermarkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
