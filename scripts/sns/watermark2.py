import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque

class WatermarkRelay(Node):
    def __init__(self):
        super().__init__('watermark_relay')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
        self.subscription = self.create_subscription(
            LaserScan, 
            '/inky/scan', 
            self.listener_callback, 
            qos_profile)
    
        self.publisher = self.create_publisher(
            LaserScan, 
            '/inky/scan_watermarked', 
            qos_profile)

        # Gaussian parameters
        self.mu = 0.100  # 100ms average delay
        self.sigma = 0.010 # 10ms jitter
        
        # Buffer to hold messages with their scheduled release time
        self.message_queue = deque()
        
        # Timer to check the queue every 10ms (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def listener_callback(self, msg):
        # Calculate delay and scheduled release time using ROS clock
        delay = max(0, random.gauss(self.mu, self.sigma))
        release_time = self.get_clock().now() + rclpy.duration.Duration(seconds=delay)
        
        # Store message and target time in queue
        self.message_queue.append((release_time, msg))

    def timer_callback(self):
        # Check if there are messages ready to be released
        now = self.get_clock().now()
        
        while self.message_queue and self.message_queue[0][0] <= now:
            release_time, msg = self.message_queue.popleft()
            self.publisher.publish(msg)
            
            if self.publisher.get_subscription_count() > 0:
                self.get_logger().info(f'Published delayed scan')

def main(args=None):
    rclpy.init(args=args)
    node = WatermarkRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
