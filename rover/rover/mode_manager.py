#!/usr/bin/env python3
"""
Mode Manager Node for Caterpillar Rover

Manual → Autonomous transition:
  1. Trigger map save
  2. Wait for map saver to finish
  3. Launch autonomous stack

Autonomous → Manual:
  - Kill autonomous stack only
"""

import rclpy
from rclpy.node import Node
import subprocess
import signal
import os

from robot_interfaces.msg import ModeSwitch
from std_srvs.srv import Trigger


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.autonomous_process = None
        self.is_autonomous = False

        # Prevent re-entrant mode transitions
        self._transition_in_progress = False

        # Service client to trigger map saving
        self.map_save_client = self.create_client(
            Trigger, '/launch_map_saver'
        )

        # Subscribe to mode switch
        self.mode_sub = self.create_subscription(
            ModeSwitch,
            '/mode_switch',
            self.mode_callback,
            10
        )

        self.get_logger().info('Mode Manager started')
        self.get_logger().info('Waiting for mode switch signal...')

    # ================= MODE SWITCH =================

    def mode_callback(self, msg: ModeSwitch):
        if self._transition_in_progress:
            self.get_logger().warn(
                'Mode transition already in progress, ignoring request'
            )
            return

        if msg.autonomous == self.is_autonomous:
            return

        self._transition_in_progress = True
        self.is_autonomous = msg.autonomous

        if self.is_autonomous:
            self.start_autonomous()
        else:
            self.stop_autonomous()

        self._transition_in_progress = False

    # ================= AUTONOMOUS START =================

    def start_autonomous(self):
        self.get_logger().info('AUTONOMOUS MODE REQUESTED')

        # 1. Trigger map save
        if not self._trigger_map_save():
            self.get_logger().error(
                'Map saving failed or timed out. Autonomous launch aborted.'
            )
            self.is_autonomous = False
            return

        # 2. Launch autonomous stack
        self._launch_autonomous_stack()

    def _trigger_map_save(self) -> bool:
        self.get_logger().info(
            'Requesting map save before autonomous start...'
        )

        if not self.map_save_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/launch_map_saver service not available'
            )
            return False

        req = Trigger.Request()
        future = self.map_save_client.call_async(req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            self.get_logger().error(
                'Map save service call timed out'
            )
            return False

        resp = future.result()
        if not resp.success:
            self.get_logger().error(
                f'Map save rejected: {resp.message}'
            )
            return False

        self.get_logger().info('Map save completed successfully')
        return True

    def _launch_autonomous_stack(self):
        self.get_logger().info('Launching autonomous stack...')

        try:
            self.autonomous_process = subprocess.Popen(
                ['ros2', 'launch', 'rover', 'autonomous.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.get_logger().info('Autonomous stack launched')

        except Exception as e:
            self.get_logger().error(
                f'Failed to launch autonomous stack: {e}'
            )
            self.autonomous_process = None
            self.is_autonomous = False
            self.get_logger().error(
                'Reverting to MANUAL mode'
            )

    # ================= AUTONOMOUS STOP =================

    def stop_autonomous(self):
        self.get_logger().info(
            'MANUAL MODE - stopping autonomous stack'
        )

        if self.autonomous_process is None:
            return

        try:
            os.killpg(
                os.getpgid(self.autonomous_process.pid),
                signal.SIGTERM
            )
            self.autonomous_process.wait(timeout=5)
            self.get_logger().info(
                'Autonomous stack stopped cleanly'
            )

        except Exception as e:
            self.get_logger().warn(
                f'Graceful stop failed: {e}'
            )
            try:
                os.killpg(
                    os.getpgid(self.autonomous_process.pid),
                    signal.SIGKILL
                )
            except Exception:
                pass
        finally:
            self.autonomous_process = None

    # ================= CLEANUP =================

    def destroy_node(self):
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
