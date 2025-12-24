#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Counter

class CounterPub(Node):
    def __init__(self):
        super().__init__("manual_mode")

        # SUBSCRIBERS
        self.counter_pub = self.create_publisher(Counter, "/counter", 10)
        self.count = 0.0
        self.create_timer(1.0, self.counter_cb)

    def counter_cb(self):
        self.count+=1

        msg = Counter()

        msg.a = int(self.count)
        msg.b = int(self.count)

        msg.c = self.count
        msg.d = self.count

        self.counter_pub.publish(msg)
        self.get_logger().info(f"{msg.b}  - pubbing count")

def main(args=None):
    rclpy.init(args=args)
    node = CounterPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()