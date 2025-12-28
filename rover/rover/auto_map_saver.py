#!/usr/bin/env python3
"""
auto_map_saver.py

ROS 2 service-driven map saver for slam_toolbox.

Service:
  /launch_map_saver (std_srvs/Trigger)

Behavior:
- Saves maps ONLY when service is called
- Stores all maps in one directory
- Auto-increments filenames:
    1.yaml / 1.pgm
    2.yaml / 2.pgm
- Uses slam_toolbox SaveMap service if available
- Falls back to nav2_map_server map_saver_cli
"""

import rclpy
from rclpy.node import Node
import os
import subprocess
import re

from std_srvs.srv import Trigger
from std_msgs.msg import String as StdString

try:
    from slam_toolbox.srv import SaveMap
except Exception:
    SaveMap = None


class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')

        # Parameters
        self.declare_parameter(
            'save_directory',
            os.path.join(os.path.expanduser('~'), 'maps')
        )
        self.declare_parameter(
            'slam_service_name',
            '/slam_toolbox/save_map'
        )

        self.save_directory = os.path.expanduser(
            self.get_parameter('save_directory').value
        )
        self.slam_service_name = self.get_parameter(
            'slam_service_name'
        ).value

        os.makedirs(self.save_directory, exist_ok=True)

        # Service
        self.create_service(
            Trigger,
            '/launch_map_saver',
            self._handle_save_request
        )

        self.get_logger().info('AutoMapSaver ready')
        self.get_logger().info(f'Map directory: {self.save_directory}')
        self.get_logger().info('Waiting for /launch_map_saver service calls')

    # ================= SERVICE CALLBACK =================

    def _handle_save_request(self, request, response):
        next_index = self._get_next_map_index()
        base_name = str(next_index)
        full_base = os.path.join(self.save_directory, base_name)

        self.get_logger().info(
            f'Saving map #{next_index} as {base_name}.yaml / {base_name}.pgm'
        )

        # Try slam_toolbox first
        if SaveMap is not None:
            if self._call_slam_toolbox_save(base_name):
                if self._check_files_exist(full_base):
                    msg = f'Map #{next_index} saved via slam_toolbox'
                    self.get_logger().info(msg)
                    response.success = True
                    response.message = msg
                    return response
                else:
                    self.get_logger().warn(
                        'slam_toolbox reported success but files missing'
                    )

        # Fallback
        if self._call_map_saver_cli(full_base):
            if self._check_files_exist(full_base):
                msg = f'Map #{next_index} saved via map_saver_cli'
                self.get_logger().info(msg)
                response.success = True
                response.message = msg
                return response

        # Failure
        msg = f'Failed to save map #{next_index}'
        self.get_logger().error(msg)
        response.success = False
        response.message = msg
        return response

    # ================= HELPERS =================

    def _get_next_map_index(self) -> int:
        indices = []

        for filename in os.listdir(self.save_directory):
            match = re.match(r'^(\d+)\.yaml$', filename)
            if match:
                indices.append(int(match.group(1)))

        return max(indices) + 1 if indices else 1

    def _call_slam_toolbox_save(self, map_name: str) -> bool:
        try:
            client = self.create_client(SaveMap, self.slam_service_name)
        except Exception as e:
            self.get_logger().error(f'Create SaveMap client failed: {e}')
            return False

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('slam_toolbox save service unavailable')
            return False

        req = SaveMap.Request()
        try:
            req.name = StdString()
            req.name.data = map_name
        except Exception:
            req.name = map_name

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            return getattr(future.result(), 'result', None) == 0

        return False

    def _call_map_saver_cli(self, full_base_path: str) -> bool:
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', full_base_path
        ]

        self.get_logger().info(f'Running fallback: {" ".join(cmd)}')

        try:
            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=20
            )
            return proc.returncode == 0
        except Exception as e:
            self.get_logger().error(f'map_saver_cli failed: {e}')
            return False

    def _check_files_exist(self, base: str) -> bool:
        return (
            os.path.exists(base + '.yaml') and
            os.path.exists(base + '.pgm')
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
