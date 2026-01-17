#!/usr/bin/env python3
"""
Camera Servo Controller Node

Controls camera tilt servo based on crater detection.

Tilt Servo:
    - Default center: 90° (lidar side)
    - Search range: 80° to 100° (±10° from center)
    - Flipped center: 270° (dump side, after limit switch press)
    - Flipped range: 260° to 280° (±10° from flipped center)

When limit switch pressed: 
    - Flips from 90° zone to 270° zone (or vice versa)
    - Camera faces opposite direction

Subscribes:
    /ml_pipeline (String) - crater detection
    /controller_pan_trigger (Bool) - limit switch for 180° flip

Publishes:
    /servo_cmd (ServoCmd) - servo angle to ESP32

Usage: ros2 run rover camera_servo_node
"""

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String, Bool
from robot_interfaces.msg import ServoCmd


class CameraServoNode(Node):
    def __init__(self):
        super().__init__('camera_servo_node')
        
        # ============ Parameters ============
        self.declare_parameter('detection_timeout', 1.0)  # seconds
        self.declare_parameter('tilt_step', 10)           # degrees
        self.declare_parameter('sweep_delay', 0.5)        # delay between sweeps
        
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.tilt_step = self.get_parameter('tilt_step').value
        self.sweep_delay = self.get_parameter('sweep_delay').value
        
        # ============ State ============
        self.last_detection_time = time.time()
        self.crater_detected = False
        
        # Tilt state
        # Normal mode: center = 90, range = 80-100
        # Flipped mode: center = 270, range = 260-280
        self.is_flipped = False
        self.tilt_offset = 0  # -10 to +10
        self.sweep_direction = 1  # 1 = increasing, -1 = decreasing
        self.last_sweep_time = time.time()
        
        # ============ Subscribers ============
        self.ml_sub = self.create_subscription(
            String, '/ml_pipeline', self.ml_callback, 10)
        
        # Subscribe to controller flip trigger (limit switch)
        self.flip_sub = self.create_subscription(
            Bool, '/controller_pan_trigger', self.flip_callback, 10)
        
        # ============ Publishers ============
        self.servo_pub = self.create_publisher(ServoCmd, '/servo_cmd', 10)
        
        # ============ Timer ============
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('=== Camera Servo Node Started ===')
        self.get_logger().info(f'Detection timeout: {self.detection_timeout}s')
        self.get_logger().info(f'Tilt range: ±{self.tilt_step}° from center')
        self.get_logger().info(f'Mode: NORMAL (90°)')
    
    def get_center_angle(self) -> int:
        """Get the center angle based on flip state."""
        return 270 if self.is_flipped else 90
    
    def get_current_angle(self) -> int:
        """Get current absolute angle (center + offset)."""
        return self.get_center_angle() + self.tilt_offset
    
    def ml_callback(self, msg: String):
        """Handle crater detection from ML pipeline."""
        data = msg.data
        
        if data.startswith('CRATER'):
            # Crater detected
            self.crater_detected = True
            self.last_detection_time = time.time()
            self.get_logger().debug('Crater detected - holding tilt')
        elif data == 'NO_CRATER':
            # No crater in current frame
            self.crater_detected = False
    
    def flip_callback(self, msg: Bool):
        """Handle flip trigger from controller limit switch."""
        if msg.data:
            # Toggle flip state
            self.is_flipped = not self.is_flipped
            
            # Reset offset to center
            self.tilt_offset = 0
            
            mode = "FLIPPED (270°)" if self.is_flipped else "NORMAL (90°)"
            self.get_logger().info(f'Camera flip triggered! Mode: {mode}')
            
            # Publish immediately
            self.publish_servo_cmd()
    
    def control_loop(self):
        """Main control loop - runs at 10Hz."""
        now = time.time()
        
        # Check if crater detection timed out
        time_since_detection = now - self.last_detection_time
        
        if time_since_detection > self.detection_timeout:
            # No detection for too long - sweep tilt
            if now - self.last_sweep_time > self.sweep_delay:
                self.sweep_tilt()
                self.last_sweep_time = now
        
        # Publish current servo command
        self.publish_servo_cmd()
    
    def sweep_tilt(self):
        """Sweep tilt offset between -tilt_step and +tilt_step."""
        # Calculate next offset
        next_offset = self.tilt_offset + (self.sweep_direction * self.tilt_step)
        
        # Check bounds and reverse direction if needed
        if next_offset > self.tilt_step:
            next_offset = self.tilt_step
            self.sweep_direction = -1
        elif next_offset < -self.tilt_step:
            next_offset = -self.tilt_step
            self.sweep_direction = 1
        
        self.tilt_offset = next_offset
        self.get_logger().info(f'Tilt sweep: {self.get_current_angle()}° (offset: {self.tilt_offset})')
    
    def publish_servo_cmd(self):
        """Publish servo command to ESP32."""
        cmd = ServoCmd()
        cmd.tilt_angle = self.get_current_angle()
        self.servo_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CameraServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
