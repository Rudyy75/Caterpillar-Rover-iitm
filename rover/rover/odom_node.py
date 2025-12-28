#!/usr/bin/env python3
"""
Odometry Node for Caterpillar Rover (Nav2 Compatible)

Computes robot pose from encoder ticks and BNO055 heading.
Publishes standard nav_msgs/Odometry and TF transform for SLAM/Nav2 compatibility.

Subscribes:
    /encoder_raw (EncoderRaw) - Hall-effect encoder ticks
    /bno (BnoReading) - IMU heading for theta

Publishes:
    /odom (nav_msgs/Odometry) - Standard ROS2 odometry message
    TF: odom -> base_link transform
"""

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from robot_interfaces.msg import EncoderRaw, BnoReading


def euler_to_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> Quaternion:
    """Convert Euler angles (radians) to Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # ============ Robot Parameters ============
        self.declare_parameter('wheel_diameter', 0.1)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('ticks_per_rev', 360)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Derived constants
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev
        
        # ============ State Variables ============
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()
        
        # Previous encoder values
        self.prev_fl = 0
        self.prev_fr = 0
        self.prev_bl = 0
        self.prev_br = 0
        self.first_encoder_msg = True
        
        # ============ TF Broadcaster ============
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ============ Subscribers ============
        self.encoder_sub = self.create_subscription(
            EncoderRaw, '/encoder_raw', self.encoder_callback, 10)
        
        self.bno_sub = self.create_subscription(
            BnoReading, '/bno', self.bno_callback, 10)
        
        # ============ Publisher ============
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.get_logger().info('Nav2-compatible Odom Node started')
        self.get_logger().info(f'  Frames: {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info(f'  TF broadcast: {self.publish_tf}')
    
    def bno_callback(self, msg: BnoReading):
        """Update theta from BNO055 yaw reading (degrees to radians)."""
        self.theta = math.radians(msg.yaw)
    
    def encoder_callback(self, msg: EncoderRaw):
        """Compute and publish odometry from encoder deltas."""
        current_time = self.get_clock().now()
        
        if self.first_encoder_msg:
            self.prev_fl = msg.fl_ticks
            self.prev_fr = msg.fr_ticks
            self.prev_bl = msg.bl_ticks
            self.prev_br = msg.br_ticks
            self.last_time = current_time
            self.first_encoder_msg = False
            return
        
        # Time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        
        # Calculate deltas
        d_fl = msg.fl_ticks - self.prev_fl
        d_fr = msg.fr_ticks - self.prev_fr
        d_bl = msg.bl_ticks - self.prev_bl
        d_br = msg.br_ticks - self.prev_br
        
        self.prev_fl = msg.fl_ticks
        self.prev_fr = msg.fr_ticks
        self.prev_bl = msg.bl_ticks
        self.prev_br = msg.br_ticks
        self.last_time = current_time
        
        # Average left and right
        left_delta = (d_fl + d_bl) / 2.0
        right_delta = (d_fr + d_br) / 2.0
        
        # Convert to meters
        left_dist = left_delta * self.meters_per_tick
        right_dist = right_delta * self.meters_per_tick
        
        # Forward distance and turn
        forward_dist = (left_dist + right_dist) / 2.0
        
        # Update position (Standard ROS/Nav2 convention)
        # +X = forward (away from scoop)
        # +Y = left
        self.x += forward_dist * math.cos(self.theta)
        self.y += forward_dist * math.sin(self.theta)
        
        # Velocities for twist
        self.vx = forward_dist / dt
        self.vth = (right_dist - left_dist) / (self.wheel_base * dt)
        
        # ============ Publish Odometry ============
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)
        
        # Pose covariance (x, y, z, roll, pitch, yaw)
        # Lower values = more confident
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,   # z unknown
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,   # roll unknown
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,   # pitch unknown
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03   # yaw from BNO
        ]
        
        # Twist (velocities in child_frame_id = base_link)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        # Twist covariance
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,   # vy unknown (diff drive)
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]
        
        self.odom_pub.publish(odom)
        
        # ============ Broadcast TF ============
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = euler_to_quaternion(self.theta)
            
            self.tf_broadcaster.sendTransform(t)


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
