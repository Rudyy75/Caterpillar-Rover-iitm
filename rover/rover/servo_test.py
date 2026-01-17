#!/usr/bin/env python3
"""
Servo Test Node - Simple servo position control via ROS topic

Publish desired angle to test servo movement.

Usage:
    ros2 run rover servo_test
    
Then publish angles:
    ros2 topic pub /servo_test_angle std_msgs/msg/Int32 "data: 0"    # Go to 0°
    ros2 topic pub /servo_test_angle std_msgs/msg/Int32 "data: 90"   # Go to 90° (center)
    ros2 topic pub /servo_test_angle std_msgs/msg/Int32 "data: 180"  # Go to 180°
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from robot_interfaces.msg import ServoCmd


class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test')
        
        # Subscribe to test angle commands
        self.create_subscription(Int32, '/servo_test_angle', self.angle_callback, 10)
        
        # Publisher for servo command
        self.servo_pub = self.create_publisher(ServoCmd, '/servo_cmd', 10)
        
        # Current angle
        self.current_angle = 90
        
        # Publish at 10Hz to keep servo updated
        self.create_timer(0.1, self.publish_angle)
        
        self.get_logger().info('=== Servo Test Node Started ===')
        self.get_logger().info('Publish to /servo_test_angle (0-360)')
        self.get_logger().info('  0-180: normal range')
        self.get_logger().info('  181-360: flipped range (270 = center flipped)')
    
    def angle_callback(self, msg: Int32):
        angle = msg.data
        
        # Clamp to valid range
        if angle < 0:
            angle = 0
        if angle > 360:
            angle = 360
        
        self.current_angle = angle
        self.get_logger().info(f'Servo angle set to: {angle}°')
    
    def publish_angle(self):
        cmd = ServoCmd()
        cmd.tilt_angle = self.current_angle
        self.servo_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
