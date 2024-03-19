import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twists
import paho.mqtt.client as mqtt

class AsyncSynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('async_synchronized_subscriber')
        # MQTT 클라이언트 설정 및 비동기 루프 시작
        self.mqtt_client = mqtt.Client()
        self.mqtt_host = 'mqtt.thingsboard.cloud'
        self.mqtt_port = 1883
        self.mqtt_topic = 'v1/devices/me/telemetry'
        
        self.mqtt_client.username_pw_set(username="a81afcd0-e03a-11ee-aedd-33f1c759bdb2", password='nl5qoluv3Pz2FhT1Hipk')

        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()  # 비동기 처리 시작

        # ROS2 토픽 구독 설정
        self.velocity_sub = Subscriber(self, Twists, 'twist_mux/cmd_vel')
        self.gps_sub = Subscriber(self, NavSatFix, 'ublox_gps_node/fix')
        # self.battery_sub = Subscriber(self, String, 'battery_topic')
        self.ats = ApproximateTimeSynchronizer([self.velocity_sub, self.gps_sub], queue_size=10, slop=1)
        self.ats.registerCallback(self.callback)

    def callback(self, velocity_msg, gps_msg):
        msg_to_publish = {
            'speed': {velocity_msg.linear.x},
            'latitude': {gps_msg.latitude},
            'longitude': {gps_msg.longitude}
            
        }
        self.get_logger().info(msg_to_publish)
        self.mqtt_client.publish(self.mqtt_topic, msg_to_publish)

def main(args=None):
    rclpy.init(args=args)
    node = AsyncSynchronizedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    node.mqtt_client.loop_stop()  # 비동기 처리 정지 및 정리
    rclpy.shutdown()

if __name__ == '__main__':
    main()