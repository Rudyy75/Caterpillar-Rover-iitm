#!/usr/bin/env python3
"""
Excavation Node for Caterpillar Rover

Autonomous excavation state machine triggered by limit switch (sand detection).

State Machine:
1. IDLE - Waiting for sand detection (limit switch)
2. BACK_UP - Detected sand, move backward (+X direction)
3. LOWER_SCREW - Lower lead screw by ~3cm
4. SCOOP - Move forward while rotating tub inward
5. LIFT - Raise lead screw with collected sand
6. TRANSPORT - Move to dump location (placeholder)
7. DUMP - Dump sand (placeholder)
8. DONE - Cycle complete, return to IDLE

Subscribes:
    /rover_limit_sw (LimitSwitches) - ls1 is sand detection

Publishes:
    /cmd_vel (geometry_msgs/Twist) - Drive commands
    /actuator_cmd (ActuatorCmd) - Lead screw and tub control
"""

import rclpy
from rclpy.node import Node
from enum import Enum
import time

from geometry_msgs.msg import Twist
from robot_interfaces.msg import LimitSwitches, ActuatorCmd


class ExcavationState(Enum):
    IDLE = 0
    BACK_UP = 1
    LOWER_SCREW = 2
    SCOOP = 3
    LIFT = 4
    TRANSPORT = 5  # Placeholder
    DUMP = 6       # Placeholder
    DONE = 7


class ExcavationNode(Node):
    def __init__(self):
        super().__init__('excavation_node')
        
        # ============ Parameters ============
        self.declare_parameter('backup_distance', 0.15)      # meters
        self.declare_parameter('backup_speed', 0.2)          # m/s
        self.declare_parameter('backup_duration', 0.75)      # seconds
        self.declare_parameter('lower_duration', 0.3)        # seconds (~3cm)
        self.declare_parameter('scoop_speed', 0.15)          # m/s
        self.declare_parameter('scoop_duration', 1.0)        # seconds
        self.declare_parameter('lift_duration', 2.0)         # seconds
        self.declare_parameter('enabled', True)              # Enable/disable excavation
        
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_duration = self.get_parameter('backup_duration').value
        self.lower_duration = self.get_parameter('lower_duration').value
        self.scoop_speed = self.get_parameter('scoop_speed').value
        self.scoop_duration = self.get_parameter('scoop_duration').value
        self.lift_duration = self.get_parameter('lift_duration').value
        self.enabled = self.get_parameter('enabled').value
        
        # ============ State Machine ============
        self.state = ExcavationState.IDLE
        self.state_start_time = 0.0
        
        # ============ Subscribers ============
        self.limit_sw_sub = self.create_subscription(
            LimitSwitches, '/rover_limit_sw', self.limit_switch_callback, 10)
        
        # ============ Publishers ============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actuator_pub = self.create_publisher(ActuatorCmd, '/actuator_cmd', 10)
        
        # ============ Timer for state machine ============
        self.timer = self.create_timer(0.05, self.state_machine_tick)  # 20Hz
        
        self.get_logger().info('Excavation Node started')
        self.get_logger().info(f'  Enabled: {self.enabled}')
    
    def limit_switch_callback(self, msg: LimitSwitches):
        """Handle limit switch trigger (sand detection)."""
        # ls1 is the sand detection limit switch
        if msg.ls1 and self.state == ExcavationState.IDLE and self.enabled:
            self.get_logger().info('Sand detected! Starting excavation sequence')
            self.transition_to(ExcavationState.BACK_UP)
    
    def transition_to(self, new_state: ExcavationState):
        """Transition to a new state."""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()
    
    def state_elapsed(self) -> float:
        """Get time elapsed in current state."""
        return time.time() - self.state_start_time
    
    def stop_all(self):
        """Stop all motors."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        actuator = ActuatorCmd()
        actuator.lead_screw = 0  # Stop
        actuator.tub_angle = 0   # Stop
        self.actuator_pub.publish(actuator)
    
    def state_machine_tick(self):
        """State machine tick - called at 20Hz."""
        
        if self.state == ExcavationState.IDLE:
            # Waiting for limit switch trigger
            pass
        
        elif self.state == ExcavationState.BACK_UP:
            # Move backward (+X direction) to clear the sand pile
            if self.state_elapsed() < self.backup_duration:
                twist = Twist()
                twist.linear.x = self.backup_speed  # Move backward (away from scoop)
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.transition_to(ExcavationState.LOWER_SCREW)
        
        elif self.state == ExcavationState.LOWER_SCREW:
            # Lower lead screw by ~3cm
            if self.state_elapsed() < self.lower_duration:
                actuator = ActuatorCmd()
                actuator.lead_screw = 2  # Down
                actuator.tub_angle = 0
                self.actuator_pub.publish(actuator)
            else:
                # Stop lead screw
                actuator = ActuatorCmd()
                actuator.lead_screw = 0
                actuator.tub_angle = 0
                self.actuator_pub.publish(actuator)
                self.transition_to(ExcavationState.SCOOP)
        
        elif self.state == ExcavationState.SCOOP:
            # Move forward while rotating tub inward to scoop sand
            if self.state_elapsed() < self.scoop_duration:
                # Move forward
                twist = Twist()
                twist.linear.x = -self.scoop_speed  # Move forward (toward scoop)
                self.cmd_vel_pub.publish(twist)
                
                # Rotate tub inward (CW)
                actuator = ActuatorCmd()
                actuator.lead_screw = 0
                actuator.tub_angle = 1  # CW to scoop
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition_to(ExcavationState.LIFT)
        
        elif self.state == ExcavationState.LIFT:
            # Raise lead screw with sand
            if self.state_elapsed() < self.lift_duration:
                actuator = ActuatorCmd()
                actuator.lead_screw = 1  # Up
                actuator.tub_angle = 0
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition_to(ExcavationState.TRANSPORT)
        
        elif self.state == ExcavationState.TRANSPORT:
            # TODO: Navigate to dump location
            # For now, just skip to dump
            self.get_logger().info('TRANSPORT: Placeholder - skipping to DUMP')
            self.transition_to(ExcavationState.DUMP)
        
        elif self.state == ExcavationState.DUMP:
            # TODO: Dump sand at construction zone
            # For now, just rotate tub outward
            self.get_logger().info('DUMP: Placeholder - rotating tub outward')
            actuator = ActuatorCmd()
            actuator.lead_screw = 0
            actuator.tub_angle = 2  # CCW to dump
            self.actuator_pub.publish(actuator)
            
            # Wait a bit then done
            if self.state_elapsed() > 1.0:
                self.stop_all()
                self.transition_to(ExcavationState.DONE)
        
        elif self.state == ExcavationState.DONE:
            self.get_logger().info('Excavation cycle complete!')
            self.transition_to(ExcavationState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = ExcavationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
