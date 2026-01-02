#!/usr/bin/env python3
"""
Simple Excavation Routine for Caterpillar Rover

Sequence:
1. Move BACK for 2 sec
2. Actuate motors (both move together) for 20 sec  
3. Move FORWARD for 3 sec

Note: Lead screw and tub motors are coupled in firmware - they move together.

Usage: ros2 run rover simple_excavation
"""

import rclpy
from rclpy.node import Node
from enum import Enum
import time

from geometry_msgs.msg import Twist
from robot_interfaces.msg import ActuatorCmd


class State(Enum):
    MOVE_BACK = 0
    ACTUATE_MOTORS = 1
    MOVE_FORWARD = 2
    DONE = 3


class SimpleExcavation(Node):
    def __init__(self):
        super().__init__('simple_excavation')
        
        # ============ Timing Parameters ============
        self.back_duration = 2.0         # seconds
        self.actuate_duration = 20.0     # seconds
        self.forward_duration = 3.0      # seconds
        
        # ============ Speed Parameters ============
        self.drive_speed = 0.3
        
        # ============ Publishers ============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actuator_pub = self.create_publisher(ActuatorCmd, '/actuator_cmd', 10)
        
        # ============ State Machine ============
        self.state = State.MOVE_BACK
        self.state_start_time = time.time()
        
        # ============ Timer ============
        self.timer = self.create_timer(0.05, self.tick)  # 20Hz
        
        self.get_logger().info('=== Simple Excavation Starting ===')
        self.get_logger().info(f'1. Move BACK: {self.back_duration}s')
        self.get_logger().info(f'2. Actuate motors: {self.actuate_duration}s')
        self.get_logger().info(f'3. Move FORWARD: {self.forward_duration}s')
    
    def elapsed(self) -> float:
        return time.time() - self.state_start_time
    
    def transition(self, new_state: State):
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()
    
    def stop_all(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        actuator = ActuatorCmd()
        actuator.lead_screw = 0
        actuator.tub_angle = 0
        self.actuator_pub.publish(actuator)
    
    def tick(self):
        if self.state == State.MOVE_BACK:
            if self.elapsed() < self.back_duration:
                twist = Twist()
                twist.linear.x = -self.drive_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.transition(State.ACTUATE_MOTORS)
        
        elif self.state == State.ACTUATE_MOTORS:
            # Both motors move together - firmware limitation
            if self.elapsed() < self.actuate_duration:
                actuator = ActuatorCmd()
                actuator.lead_screw = 2  # Actuates both motors
                actuator.tub_angle = 0
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition(State.MOVE_FORWARD)
        
        elif self.state == State.MOVE_FORWARD:
            if self.elapsed() < self.forward_duration:
                twist = Twist()
                twist.linear.x = self.drive_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.transition(State.DONE)
        
        elif self.state == State.DONE:
            self.get_logger().info('=== Excavation Complete! ===')
            self.stop_all()
            self.timer.cancel()
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleExcavation()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
