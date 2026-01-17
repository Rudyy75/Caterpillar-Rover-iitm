#!/usr/bin/env python3
"""
Autonomous Mission Node for Caterpillar Rover

Mission Sequence:
1. Go straight for 15m (using odom for distance, BNO for heading)
2. Execute excavation routine
3. Turn right 90 degrees (using BNO yaw)
4. Go forward 3m
5. Execute dump
6. RELAY LOOP:
   - Move back 1.5m
   - Excavate
   - Move forward 1.5m
   - Dump
   - Repeat until interrupted

Usage: ros2 run rover auto_mission
"""

import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_interfaces.msg import BnoReading, ActuatorCmd


class MissionState(Enum):
    IDLE = 0
    INITIAL_TURN = 1          # Turn +90 degrees at start
    GO_TO_EXCAVATION = 2      # Travel 6m (initial)
    EXCAVATING = 3            # Run excavation sequence
    TURN_RIGHT = 4            # Turn 90 degrees right
    GO_TO_DUMP = 5            # Travel 2m (initial)
    DUMPING = 6               # Execute dump
    # Relay loop states
    RELAY_BACK = 7            # Move back 1.5m
    RELAY_EXCAVATE = 8        # Run excavation
    RELAY_FORWARD = 9         # Move forward 1.5m
    RELAY_DUMP = 10           # Dump again
    DONE = 11


class ExcavationSubState(Enum):
    MOVE_FORWARD = 0
    SCOOP_UP_1 = 1
    LEAD_SCREW_UP = 2
    SCOOP_UP_2 = 3
    DONE = 4


class AutoMission(Node):
    def __init__(self):
        super().__init__('auto_mission')
        
        # ============ Parameters ============
        self.declare_parameter('excavation_distance', 15.0)  # meters
        self.declare_parameter('dump_distance', 3.0)         # meters
        self.declare_parameter('relay_distance', 1.5)        # meters
        self.declare_parameter('forward_speed', 0.75)        # m/s
        self.declare_parameter('turn_speed', 0.5)            # rad/s
        self.declare_parameter('heading_kp', 0.02)           # Heading correction P gain
        
        self.excavation_distance = self.get_parameter('excavation_distance').value
        self.dump_distance = self.get_parameter('dump_distance').value
        self.relay_distance = self.get_parameter('relay_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.heading_kp = self.get_parameter('heading_kp').value
        
        # ============ State Machine ============
        self.state = MissionState.IDLE
        self.excavation_substate = ExcavationSubState.MOVE_FORWARD
        self.state_start_time = 0.0
        self.relay_count = 0
        
        # ============ Navigation State ============
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.initial_yaw = 0.0
        self.odom_received = False
        self.bno_received = False
        
        # ============ Subscribers ============
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.bno_sub = self.create_subscription(
            BnoReading, '/bno', self.bno_callback, 10)
        
        # ============ Publishers ============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actuator_pub = self.create_publisher(ActuatorCmd, '/actuator_cmd', 10)
        
        # ============ Timer ============
        self.timer = self.create_timer(0.05, self.mission_tick)  # 20Hz
        
        self.get_logger().info('=== Auto Mission Node Started ===')
        self.get_logger().info('Waiting for odom and BNO data...')
    
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_received = True
    
    def bno_callback(self, msg: BnoReading):
        self.current_yaw = msg.yaw
        self.bno_received = True
    
    def elapsed(self) -> float:
        return time.time() - self.state_start_time
    
    def transition(self, new_state: MissionState):
        self.get_logger().info(f'Mission: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()
        
        # Record start position for distance-based states
        if new_state in [MissionState.GO_TO_EXCAVATION, MissionState.GO_TO_DUMP, 
                         MissionState.RELAY_BACK, MissionState.RELAY_FORWARD]:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.initial_yaw = self.current_yaw
        
        # Record target yaw for turning
        if new_state == MissionState.INITIAL_TURN:
            self.target_yaw = self.normalize_angle(self.current_yaw + 90.0)
        elif new_state == MissionState.TURN_RIGHT:
            self.target_yaw = self.normalize_angle(self.current_yaw - 90.0)
    
    def transition_excavation(self, new_substate: ExcavationSubState):
        self.get_logger().info(f'Excavation: {self.excavation_substate.name} -> {new_substate.name}')
        self.excavation_substate = new_substate
        self.state_start_time = time.time()
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to 180 range."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def get_distance_traveled(self) -> float:
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_yaw_error(self, target: float) -> float:
        """Get signed error from current yaw to target."""
        error = target - self.current_yaw
        return self.normalize_angle(error)
    
    def stop_all(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        actuator = ActuatorCmd()
        actuator.lead_screw = 0
        actuator.tub_angle = 0
        self.actuator_pub.publish(actuator)
    
    def drive_straight(self, speed: float):
        """Drive straight while correcting heading drift using BNO."""
        heading_error = self.get_yaw_error(self.initial_yaw)
        correction = heading_error * self.heading_kp
        
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = correction  # Correct drift
        self.cmd_vel_pub.publish(twist)
    
    def mission_tick(self):
        # Wait for sensor data before starting
        if self.state == MissionState.IDLE:
            if self.odom_received and self.bno_received:
                self.get_logger().info('Sensors ready - Starting mission!')
                self.transition(MissionState.INITIAL_TURN)
            return
        
        # ============ INITIAL TURN (+90°) ============
        if self.state == MissionState.INITIAL_TURN:
            yaw_error = self.get_yaw_error(self.target_yaw)
            
            if abs(yaw_error) > 3.0:  # 3 degree tolerance
                twist = Twist()
                twist.angular.z = self.turn_speed if yaw_error > 0 else -self.turn_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.get_logger().info(f'Initial turn complete. Current yaw: {self.current_yaw:.1f}°')
                self.transition(MissionState.GO_TO_EXCAVATION)
            return
        
        # ============ GO TO EXCAVATION (15m) ============
        if self.state == MissionState.GO_TO_EXCAVATION:
            distance = self.get_distance_traveled()
            
            if distance < self.excavation_distance:
                self.drive_straight(self.forward_speed)
                
                if int(self.elapsed()) % 5 == 0 and self.elapsed() % 1 < 0.1:
                    self.get_logger().info(f'Distance: {distance:.2f}m / {self.excavation_distance}m')
            else:
                self.stop_all()
                self.get_logger().info(f'Reached excavation point at {distance:.2f}m')
                self.excavation_substate = ExcavationSubState.MOVE_FORWARD
                self.transition(MissionState.EXCAVATING)
        
        # ============ EXCAVATING ============
        elif self.state == MissionState.EXCAVATING:
            self.run_excavation(next_state=MissionState.TURN_RIGHT)
        
        # ============ TURN RIGHT 90° ============
        elif self.state == MissionState.TURN_RIGHT:
            yaw_error = self.get_yaw_error(self.target_yaw)
            
            if abs(yaw_error) > 3.0:  # 3 degree tolerance
                twist = Twist()
                twist.angular.z = -self.turn_speed if yaw_error < 0 else self.turn_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.get_logger().info(f'Turn complete. Current yaw: {self.current_yaw:.1f}°')
                self.transition(MissionState.GO_TO_DUMP)
        
        # ============ GO TO DUMP (3m) ============
        elif self.state == MissionState.GO_TO_DUMP:
            distance = self.get_distance_traveled()
            
            if distance < self.dump_distance:
                self.drive_straight(self.forward_speed)
            else:
                self.stop_all()
                self.get_logger().info(f'Reached dump point at {distance:.2f}m')
                self.transition(MissionState.DUMPING)
        
        # ============ DUMPING ============
        elif self.state == MissionState.DUMPING:
            if self.elapsed() < 2.0:
                self.get_logger().info('Dumping... (placeholder)')
            else:
                self.stop_all()
                self.get_logger().info('Starting relay loop...')
                self.transition(MissionState.RELAY_BACK)
        
        # ============ RELAY: MOVE BACK (1.5m) ============
        elif self.state == MissionState.RELAY_BACK:
            distance = self.get_distance_traveled()
            
            if distance < self.relay_distance:
                self.drive_straight(-self.forward_speed)  # Negative = backward
            else:
                self.stop_all()
                self.relay_count += 1
                self.get_logger().info(f'Relay #{self.relay_count}: Moved back {distance:.2f}m')
                self.excavation_substate = ExcavationSubState.MOVE_FORWARD
                self.transition(MissionState.RELAY_EXCAVATE)
        
        # ============ RELAY: EXCAVATE ============
        elif self.state == MissionState.RELAY_EXCAVATE:
            self.run_excavation(next_state=MissionState.RELAY_FORWARD)
        
        # ============ RELAY: MOVE FORWARD (1.5m) ============
        elif self.state == MissionState.RELAY_FORWARD:
            distance = self.get_distance_traveled()
            
            if distance < self.relay_distance:
                self.drive_straight(self.forward_speed)
            else:
                self.stop_all()
                self.get_logger().info(f'Relay #{self.relay_count}: Moved forward {distance:.2f}m')
                self.transition(MissionState.RELAY_DUMP)
        
        # ============ RELAY: DUMP ============
        elif self.state == MissionState.RELAY_DUMP:
            if self.elapsed() < 2.0:
                self.get_logger().info(f'Relay #{self.relay_count}: Dumping...')
            else:
                self.stop_all()
                self.get_logger().info(f'Relay #{self.relay_count} complete. Looping back...')
                self.transition(MissionState.RELAY_BACK)  # Loop forever!
        
        # ============ DONE (not reached in relay mode) ============
        elif self.state == MissionState.DONE:
            self.get_logger().info('=== Mission Complete! ===')
            self.stop_all()
            self.timer.cancel()
            raise SystemExit(0)
    
    def run_excavation(self, next_state: MissionState):
        """Run the excavation sub-state machine."""
        
        if self.excavation_substate == ExcavationSubState.MOVE_FORWARD:
            if self.elapsed() < 2.0:
                twist = Twist()
                twist.linear.x = self.forward_speed
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_all()
                self.transition_excavation(ExcavationSubState.SCOOP_UP_1)
        
        elif self.excavation_substate == ExcavationSubState.SCOOP_UP_1:
            if self.elapsed() < 2.0:
                actuator = ActuatorCmd()
                actuator.lead_screw = 0
                actuator.tub_angle = 1  # UP/scoop
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition_excavation(ExcavationSubState.LEAD_SCREW_UP)
        
        elif self.excavation_substate == ExcavationSubState.LEAD_SCREW_UP:
            if self.elapsed() < 17.0:
                actuator = ActuatorCmd()
                actuator.lead_screw = 2  # UP
                actuator.tub_angle = 0
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition_excavation(ExcavationSubState.SCOOP_UP_2)
        
        elif self.excavation_substate == ExcavationSubState.SCOOP_UP_2:
            if self.elapsed() < 2.0:
                actuator = ActuatorCmd()
                actuator.lead_screw = 0
                actuator.tub_angle = 1  # UP/scoop
                self.actuator_pub.publish(actuator)
            else:
                self.stop_all()
                self.transition_excavation(ExcavationSubState.DONE)
        
        elif self.excavation_substate == ExcavationSubState.DONE:
            self.get_logger().info('Excavation complete!')
            self.transition(next_state)


def main(args=None):
    rclpy.init(args=args)
    node = AutoMission()
    
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
