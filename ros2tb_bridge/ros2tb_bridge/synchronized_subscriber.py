#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import json

class AsyncSynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('thread_synchronized_subscriber')

        print("Node initialization...")

        # ROS2 토픽 구독 설정
        self.velocity_sub = Subscriber(self, Twist, '/twist_mux/cmd_vel')
        self.gps_sub = Subscriber(self, NavSatFix, '/ublox_gps_node/fix')

        # ApproximateTimeSynchronizer 설정
        self.ats = ApproximateTimeSynchronizer([self.velocity_sub, self.gps_sub], queue_size=10, slop=1, allow_headerless=True)
        self.ats.registerCallback(self.callback)

        print("Subscribers and synchronizer setup completed.")

    def callback(self, velocity_msg, gps_msg):
        # 동기화된 데이터를 로깅
        self.get_logger().info(f"Synchronized Data - Speed: {velocity_msg.linear.x}, Latitude: {gps_msg.latitude}, Longitude: {gps_msg.longitude}")

def main(args=None):
    rclpy.init(args=args)
    node = AsyncSynchronizedSubscriber()
    print("Node is spinning...")
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    print("Node shutdown.")

if __name__ == '__main__':
    main()
