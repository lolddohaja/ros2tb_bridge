import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String  # 실제 사용할 메시지 타입으로 변경 필요
import paho.mqtt.client as mqtt
import threading

class ThreadedSynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('threaded_synchronized_subscriber')
        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_host = 'localhost'
        self.mqtt_port = 1883
        self.mqtt_topic = 'synchronized_data'
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)

        # ROS2 토픽 구독 설정
        self.velocity_sub = Subscriber(self, String, 'velocity_topic')
        self.gps_sub = Subscriber(self, String, 'gps_topic')
        self.battery_sub = Subscriber(self, String, 'battery_topic')
        self.ats = ApproximateTimeSynchronizer([self.velocity_sub, self.gps_sub, self.battery_sub], queue_size=10, slop=0.5)
        self.ats.registerCallback(self.callback)

    def callback(self, velocity_msg, gps_msg, battery_msg):
        # 동기화된 메시지 처리 후 MQTT로 메시지 발행
        msg_to_publish = f'Synchronized Messages:\nVelocity: {velocity_msg.data}\nGPS: {gps_msg.data}\nBattery: {battery_msg.data}'
        self.get_logger().info(msg_to_publish)
        threading.Thread(target=self.publish_mqtt_message, args=(msg_to_publish,)).start()

    def publish_mqtt_message(self, message):
        # 별도 스레드에서 MQTT 메시지 발행
        self.mqtt_client.publish(self.mqtt_topic, message)

def main(args=None):
    rclpy.init(args=args)
    node = ThreadedSynchronizedSubscriber()
    rclpy.spin(node)
    # Clean up
    node.destroy_node()
    node.mqtt_client.disconnect()
    rclpy.shutdown()

if __name__ == '__main__':
    main()