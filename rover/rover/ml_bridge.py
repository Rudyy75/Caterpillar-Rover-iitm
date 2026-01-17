#!/usr/bin/env python3
"""
ML Bridge Node - Receives UDP from ML pipeline and publishes to ROS2

The ML pipeline runs in a Python 3.10 venv (Hailo) and can't use rclpy.
This bridge receives UDP messages and publishes to /ml_pipeline ROS topic.

Usage: ros2 run rover ml_bridge
"""

import rclpy
from rclpy.node import Node
import socket
import threading

from std_msgs.msg import String


class MLBridgeNode(Node):
    def __init__(self):
        super().__init__('ml_bridge')
        
        # UDP socket to receive from ML pipeline
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 5555))
        self.sock.setblocking(False)  # Non-blocking
        
        # Buffer for latest detection
        self.latest_detection = "NO_CRATER"
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(String, '/ml_pipeline', 10)
        
        # Timer to receive UDP messages (fast, just buffers)
        self.create_timer(0.01, self.receive_udp)  # 100Hz receive
        
        # Timer to publish at 10Hz
        self.create_timer(0.1, self.publish_detection)  # 10Hz publish
        
        self.get_logger().info('=== ML Bridge Node Started ===')
        self.get_logger().info('Listening on UDP port 5555, publishing at 10Hz')
    
    def receive_udp(self):
        """Receive from UDP and buffer (does NOT publish)."""
        try:
            data, addr = self.sock.recvfrom(1024)
            self.latest_detection = data.decode()
        except BlockingIOError:
            pass  # No data available
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def publish_detection(self):
        """Publish latest detection at 10Hz."""
        msg = String()
        msg.data = self.latest_detection
        self.detection_pub.publish(msg)
        self.get_logger().debug(msg.data)
    
    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MLBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
