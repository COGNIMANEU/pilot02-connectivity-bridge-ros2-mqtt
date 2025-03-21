#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosSubscriber(Node):
    def __init__(self):
        super().__init__('ros_subscriber')
        self.get_logger().info('RosSubscriber started!')
        self.subscription = self.create_subscription(String, '/from_mqtt', self.callback, 10)

        # Create a timer to shut down the node after 30 seconds
        self.timer = self.create_timer(30.0, self.shutdown_node)

    def callback(self, msg):
        self.get_logger().info(f'RosSubscriber received via MQTT → ROS: "{msg.data}"')

    def shutdown_node(self):
        self.get_logger().info('RosSubscriber shutting down after 30 seconds.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RosSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
