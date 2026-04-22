
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class WatermarkRelay(Node):
    def __init__(self):
        super().__init__('watermark_relay')
# 1. Define a QoS profile that matches the LiDAR
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This matches the 'incompatible' policy
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    # 2. Use that profile in your subscription
        self.subscription = self.create_subscription(
            LaserScan, 
            '/inky/scan', 
            self.listener_callback, 
            qos_profile) # Pass the profile here
    
    # 3. Use it for the publisher too so the VM can hear it easily
        self.publisher = self.create_publisher(
            LaserScan, 
            '/inky/scan_watermarked', 
            qos_profile)
        # Gaussian parameters
        self.mu = 0.100000  # 100ms average delay
        self.sigma = 0.050 # 50ms jitter

    def listener_callback(self, msg):
        delay = max(0, random.gauss(self.mu, self.sigma))

        if self.publisher.get_subscription_count() > 0:
            self.get_logger().info(f'Controller is active! Applying Gaussian watermark of {delay:.8f} s')
        
        # Add the "Timing Signature"
        time.sleep(delay)
        
        # Send it out on the new channel
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WatermarkRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
