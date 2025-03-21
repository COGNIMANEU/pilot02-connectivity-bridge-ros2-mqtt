#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosPublisher(Node):
    def __init__(self):
        super().__init__('ros_publisher')
        self.get_logger().info('RosPublisher started!')
        self.publisher = self.create_publisher(String, '/ros_to_mqtt', 10)
        self.counter = 0
        self.max_messages = 10  # Publish 10 times
        timer_period = 1.0  # 1 second
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        if self.counter < self.max_messages:
            msg = String()
            msg.data = f'Hello from ROS Publisher! Message {self.counter + 1}'
            self.publisher.publish(msg)
            self.get_logger().info(f'RosPublisher Published: "{msg.data}"')
            self.counter += 1
        else:
            self.get_logger().info('RosPublisher finished publishing.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RosPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
