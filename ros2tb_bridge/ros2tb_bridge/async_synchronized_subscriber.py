import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json

class AsyncSynchronizedSubscriber(Node):
    def __init__(self):
        super().__init__('async_synchronized_subscriber')
        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_host = 'mqtt.thingsboard.cloud'
        self.mqtt_port = 1883
        self.mqtt_topic = 'v1/devices/me/telemetry'
        
        self.mqtt_client.username_pw_set(username="a81afcd0-e03a-11ee-aedd-33f1c759bdb2", password='nl5qoluv3Pz2FhT1Hipk')
        self.mqtt_client.connect_async(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()
        print(1)
        
        # ROS2 토픽 구독 설정
        self.velocity_sub = Subscriber(self, Twist, 'twist_mux/cmd_vel')
        self.gps_sub = Subscriber(self, NavSatFix, 'ublox_gps_node/fix')
        self.ats = ApproximateTimeSynchronizer([self.velocity_sub, self.gps_sub], queue_size=10, slop=1, allow_headerless=True)
        self.ats.registerCallback(self.callback)
        print(2)

    def callback(self, velocity_msg, gps_msg):
        print(3)
        msg_to_publish = json.dumps({
            'speed': velocity_msg.linear.x,
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude
        })
        self.get_logger().info(msg_to_publish)
        # Use asyncio to run the publish_message function in an event loop
        asyncio.run(self.publish_message(msg_to_publish))

    async def publish_message(self, message):
        # This method is now an async function.
        # It allows us to use asynchronous methods within it.
        self.mqtt_client.publish(self.mqtt_topic, message)

def main(args=None):
    rclpy.init(args=args)
    node = AsyncSynchronizedSubscriber()
    
    # Use MultiThreadedExecutor to enable concurrent handling of callbacks
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    node.mqtt_client.loop_stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
