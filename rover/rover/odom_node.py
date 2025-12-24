#!/usr/bin/env python3
"""
Odometry Node for Caterpillar Rover
Computes robot pose from encoder ticks and BNO055 heading.

Subscribes:
    /encoder_raw (EncoderRaw) - Hall-effect encoder ticks
    /bno (BnoReading) - IMU heading for theta

Publishes:
    /odom (Odom) - Computed robot pose (x, y, theta)
"""

import rclpy
from rclpy.node import Node
import math

from robot_interfaces.msg import EncoderRaw, BnoReading, Odom


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # ============ Robot Parameters ============
        # Modify these to match your robot!
        self.declare_parameter('wheel_diameter', 0.1)      # meters
        self.declare_parameter('wheel_base', 0.3)          # meters (between left/right wheels)
        self.declare_parameter('ticks_per_rev', 360)       # encoder ticks per wheel revolution
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        # Derived constants
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev
        
        # ============ State Variables ============
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # From BNO
        
        # Previous encoder values for delta calculation
        self.prev_fl = 0
        self.prev_fr = 0
        self.prev_bl = 0
        self.prev_br = 0
        self.first_encoder_msg = True
        
        # ============ Subscribers ============
        self.encoder_sub = self.create_subscription(
            EncoderRaw,
            '/encoder_raw',
            self.encoder_callback,
            10
        )
        
        self.bno_sub = self.create_subscription(
            BnoReading,
            '/bno',
            self.bno_callback,
            10
        )
        
        # ============ Publisher ============
        self.odom_pub = self.create_publisher(Odom, '/odom', 10)
        
        self.get_logger().info('Odom Node started')
        self.get_logger().info(f'  Wheel diameter: {self.wheel_diameter} m')
        self.get_logger().info(f'  Wheel base: {self.wheel_base} m')
        self.get_logger().info(f'  Ticks per rev: {self.ticks_per_rev}')
    
    def bno_callback(self, msg: BnoReading):
        """Update theta from BNO055 yaw reading."""
        # Convert degrees to radians if needed
        self.theta = math.radians(msg.yaw)
    
    def encoder_callback(self, msg: EncoderRaw):
        """Compute odometry from encoder deltas."""
        
        if self.first_encoder_msg:
            # Initialize previous values on first message
            self.prev_fl = msg.fl_ticks
            self.prev_fr = msg.fr_ticks
            self.prev_bl = msg.bl_ticks
            self.prev_br = msg.br_ticks
            self.first_encoder_msg = False
            return
        
        # Calculate deltas
        d_fl = msg.fl_ticks - self.prev_fl
        d_fr = msg.fr_ticks - self.prev_fr
        d_bl = msg.bl_ticks - self.prev_bl
        d_br = msg.br_ticks - self.prev_br
        
        # Update previous values
        self.prev_fl = msg.fl_ticks
        self.prev_fr = msg.fr_ticks
        self.prev_bl = msg.bl_ticks
        self.prev_br = msg.br_ticks
        
        # Average left and right wheel deltas
        left_delta = (d_fl + d_bl) / 2.0
        right_delta = (d_fr + d_br) / 2.0
        
        # Convert ticks to meters
        left_dist = left_delta * self.meters_per_tick
        right_dist = right_delta * self.meters_per_tick
        
        # Forward distance (average of both sides)
        forward_dist = (left_dist + right_dist) / 2.0
        
        # Update position using BNO theta
        # x = forward (robot's Y in local frame)
        # y = lateral (robot's X in local frame, but we're diff drive so lateral = 0)
        self.x += forward_dist * math.cos(self.theta)
        self.y += forward_dist * math.sin(self.theta)
        
        # Publish odometry
        odom = Odom()
        odom.x = self.x
        odom.y = self.y
        odom.theta = self.theta
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
