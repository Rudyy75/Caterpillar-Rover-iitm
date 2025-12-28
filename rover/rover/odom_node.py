#!/usr/bin/env python3
"""
Odometry Node for Caterpillar Rover (Nav2 + slam_toolbox Compatible)

Computes robot pose from encoder ticks and BNO055 heading.
Publishes nav_msgs/Odometry and dynamic TF (odom -> base_link).

Static TFs are published ONLY from the launch file.
"""

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from robot_interfaces.msg import EncoderRaw, BnoReading


def euler_to_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> Quaternion:
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

        # Parameters
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

        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.prev_fl = 0
        self.prev_fr = 0
        self.prev_bl = 0
        self.prev_br = 0
        self.first_encoder_msg = True

        # TF broadcaster (dynamic only)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS interfaces
        self.create_subscription(EncoderRaw, '/encoder_raw', self.encoder_callback, 10)
        self.create_subscription(BnoReading, '/bno', self.bno_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info('Odom node started (dynamic TF only)')

    def bno_callback(self, msg: BnoReading):
        self.theta = math.radians(msg.yaw)

    def encoder_callback(self, msg: EncoderRaw):
        current_time = self.get_clock().now()

        if self.first_encoder_msg:
            self.prev_fl = msg.fl_ticks
            self.prev_fr = msg.fr_ticks
            self.prev_bl = msg.bl_ticks
            self.prev_br = msg.br_ticks
            self.last_time = current_time
            self.first_encoder_msg = False
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        d_fl = msg.fl_ticks - self.prev_fl
        d_fr = msg.fr_ticks - self.prev_fr
        d_bl = msg.bl_ticks - self.prev_bl
        d_br = msg.br_ticks - self.prev_br

        self.prev_fl = msg.fl_ticks
        self.prev_fr = msg.fr_ticks
        self.prev_bl = msg.bl_ticks
        self.prev_br = msg.br_ticks
        self.last_time = current_time

        left_dist = ((d_fl + d_bl) / 2.0) * self.meters_per_tick
        right_dist = ((d_fr + d_br) / 2.0) * self.meters_per_tick
        forward_dist = (left_dist + right_dist) / 2.0

        self.x += forward_dist * math.cos(self.theta)
        self.y += forward_dist * math.sin(self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
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
