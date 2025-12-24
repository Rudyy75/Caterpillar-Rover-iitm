#!/usr/bin/env python3
"""
Mode Manager Node for Caterpillar Rover
Listens to /mode_switch and dynamically launches/kills autonomous nodes.

Subscribes:
    /mode_switch (ModeSwitch) - Toggle from controller limit switch

Behavior:
    - autonomous=False → Manual mode (just Blitz + Odom running)
    - autonomous=True → Launch autonomous stack (ML, SLAM, Nav)
"""

import rclpy
from rclpy.node import Node
import subprocess
import signal
import os

from robot_interfaces.msg import ModeSwitch


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        
        self.autonomous_process = None
        self.is_autonomous = False
        
        # Subscribe to mode switch
        self.mode_sub = self.create_subscription(
            ModeSwitch,
            '/mode_switch',
            self.mode_callback,
            10
        )
        
        self.get_logger().info('Mode Manager started')
        self.get_logger().info('Waiting for mode switch signal...')
    
    def mode_callback(self, msg: ModeSwitch):
        """Handle mode switch signal from controller."""
        
        if msg.autonomous == self.is_autonomous:
            # No change, ignore
            return
        
        self.is_autonomous = msg.autonomous
        
        if self.is_autonomous:
            self.start_autonomous()
        else:
            self.stop_autonomous()
    
    def start_autonomous(self):
        """Launch autonomous stack."""
        self.get_logger().info('AUTONOMOUS MODE - Launching stack...')
        
        try:
            # Launch autonomous nodes
            self.autonomous_process = subprocess.Popen(
                ['ros2', 'launch', 'rover', 'autonomous.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean kill
            )
            self.get_logger().info('Autonomous stack launched')
        except Exception as e:
            self.get_logger().error(f'Failed to launch autonomous stack: {e}')
    
    def stop_autonomous(self):
        """Kill autonomous stack."""
        self.get_logger().info('MANUAL MODE - Stopping autonomous stack...')
        
        if self.autonomous_process is not None:
            try:
                # Kill entire process group
                os.killpg(os.getpgid(self.autonomous_process.pid), signal.SIGTERM)
                self.autonomous_process.wait(timeout=5)
                self.get_logger().info('Autonomous stack stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping autonomous stack: {e}')
                # Force kill if graceful shutdown failed
                try:
                    os.killpg(os.getpgid(self.autonomous_process.pid), signal.SIGKILL)
                except:
                    pass
            finally:
                self.autonomous_process = None
    
    def destroy_node(self):
        """Clean up on shutdown."""
        self.stop_autonomous()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
