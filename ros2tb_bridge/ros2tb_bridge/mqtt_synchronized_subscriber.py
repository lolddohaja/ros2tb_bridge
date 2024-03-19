#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

import json
import threading
import paho.mqtt.client as mqtt

class CustomSyncSubscriber(Node):
    def __init__(self):
        super().__init__('thingsboard_bridge')

        self.latest_twist_msg = None
        self.latest_gps_msg = None
        self.latest_battery_msg = None

        # 각 토픽에 대한 구독자 설정
        self.twist_subscriber = self.create_subscription(Twist, '/twist_mux/cmd_vel', self.twist_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        self.battery_subscriber = self.create_subscription(Int32MultiArray, '/a012_power', self.battery_callback, 10)

        # 타이머를 설정하여 주기적으로 최신 메시지를 처리
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_broker = 'mqtt.thingsboard.cloud'
        self.mqtt_port = 1883
        self.mqtt_topic = 'v1/devices/me/telemetry'
        self.mqtt_client.username_pw_set(username="HDR001", password='zetabank')
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

    def twist_callback(self, msg):
        # Twist 메시지를 받을 때마다 최신 상태를 업데이트
        self.latest_twist_msg = msg

    def gps_callback(self, msg):
        # GPS 메시지를 받을 때마다 최신 상태를 업데이트
        self.latest_gps_msg = msg

    def battery_callback(self, msg):
        # BatteryStateArray 메시지를 받을 때마다 최신 상태를 업데이트
        self.latest_battery_msg = msg

    def timer_callback(self):
        # 모든 최신 메시지가 유효할 때 JSON 로깅
        if self.latest_twist_msg and self.latest_gps_msg and self.latest_battery_msg:
            # JSON 문자열로 로깅할 데이터 구성
            log_data = {
                "speed": self.latest_twist_msg.linear.x,
                "latitude": self.latest_gps_msg.latitude,
                "longitude": self.latest_gps_msg.longitude,
                "battery_level": self.latest_battery_msg.data[0]  # 예를 들어 첫 번째 배터리 레벨만 로깅
            }
            log_message = json.dumps(log_data)
            self.mqtt_client.publish(self.mqtt_topic, log_message)
            self.get_logger().info(log_message)
        else:
            self.get_logger().info("Waiting for Twist, GPS, and Battery messages...")

def main(args=None):
    rclpy.init(args=args)
    node = CustomSyncSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        node.mqtt_client.loop_stop()  # 비동기 처리 정지 및 정리
        rclpy.shutdown()

if __name__ == '__main__':
    main()
