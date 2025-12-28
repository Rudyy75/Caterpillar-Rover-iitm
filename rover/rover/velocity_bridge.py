#!/usr/bin/env python3
"""
Velocity Bridge Node for Caterpillar Rover (Nav2 Compatible)

Bridges between Nav2's geometry_msgs/Twist and the custom Velocity message
expected by the Blitz MCU interface.

Subscribes:
    /cmd_vel (geometry_msgs/Twist) - Standard Nav2 velocity commands

Publishes:
    /velocity (robot_interfaces/Velocity) - Custom message for Blitz/MCU
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robot_interfaces.msg import Velocity


class VelocityBridge(Node):
    def __init__(self):
        super().__init__('velocity_bridge')
        
        # Parameters
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/velocity')
        self.declare_parameter('max_linear_vel', 1.0)   # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        
        # Subscriber to Nav2's cmd_vel
        self.twist_sub = self.create_subscription(
            Twist, input_topic, self.twist_callback, 10)
        
        # Publisher to Blitz/MCU
        self.vel_pub = self.create_publisher(Velocity, output_topic, 10)
        
        self.get_logger().info(f'Velocity Bridge: {input_topic} -> {output_topic}')
    
    def twist_callback(self, msg: Twist):
        """Convert geometry_msgs/Twist to robot_interfaces/Velocity."""
        vel = Velocity()
        
        # Nav2 Twist convention (standard ROS):
        # linear.x > 0 = forward (robot moves away from scoop = +X in our frame)
        # angular.z > 0 = counter-clockwise
        
        # Our custom coordinate frame:
        # +X = south (away from scoop) = robot's navigational "forward"
        # +Y = east (right)
        # No negation needed - positive linear.x = forward = +X
        
        vel.vx = 0.0  # Differential drive can't strafe
        vel.vy = self.clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vel.vw = self.clamp(msg.angular.z, -self.max_angular, self.max_angular)
        
        self.vel_pub.publish(vel)
    
    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
