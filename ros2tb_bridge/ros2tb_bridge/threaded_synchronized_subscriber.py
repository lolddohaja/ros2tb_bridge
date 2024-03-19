#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import threading
import subprocess

class AsyncSynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('thread_synchronized_subscriber')
        # MQTT 클라이언트 설정 및 비동기 루프 시작
        # self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        # self.mqtt_broker = 'mqtt.thingsboard.cloud'
        # self.mqtt_port = '1883'
        # self.mqtt_topic = 'v1/devices/me/telemetry'
        
        #self.mqtt_client.username_pw_set(username="a81afcd0-e03a-11ee-aedd-33f1c759bdb2", password='nl5qoluv3Pz2FhT1Hipk')
        print(1)
        # self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        print(2)
        # self.mqtt_client.loop_start()
        print(3)

        # ROS2 토픽 구독 설정
        self.velocity_sub = Subscriber(self, Twist, '/twist_mux/cmd_vel')
        self.gps_sub = Subscriber(self, NavSatFix, '/ublox_gps_node/fix')
        print(4)
        self.ats = ApproximateTimeSynchronizer([self.velocity_sub, self.gps_sub], queue_size=10, slop=1, allow_headerless=True)
        print(5)
        self.ats.registerCallback(self.callback)
        print(6)

    def callback(self, velocity_msg, gps_msg):
        print(7)
        msg_to_publish = json.dumps({
            'speed': velocity_msg.linear.x,
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude
        })
        
        # 로깅은 메인 스레드에서 실행
        self.get_logger().info(msg_to_publish)
        
        # MQTT 메시지 발행을 별도의 스레드로 실행
        threading.Thread(target=self.publish_message, args=(msg_to_publish,)).start()

    def publish_message(self, message):
        # 이 메소드는 별도의 스레드에서 실행됩니다.
        command = [
            'mosquitto_pub',
            '-h', self.mqtt_broker,  # MQTT 서버 호스트
            '-p', str(self.mqtt_port),  # 포트
            '-t', self.mqtt_topic,  # 토픽
            '-u', "a81afcd0-e03a-11ee-aedd-33f1c759bdb2",  # 사용자 이름
            '-P', 'nl5qoluv3Pz2FhT1Hipk',  # 비밀번호
            '-m', message  # 메시지
        ]
        subprocess.run(command)

def main(args=None):
    rclpy.init(args=args)
    print(8)
    node = AsyncSynchronizedSubscriber()
    print(9)
    rclpy.spin(node)
    print(10)
    # Clean up
    node.destroy_node()
    print(11)
    # node.mqtt_client.loop_stop()  # 비동기 처리 정지 및 정리
    print(11)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
