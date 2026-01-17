#!/usr/bin/env python3
"""
Unified Blitz Node - Handles BOTH serial read and write on a single connection.

ROBUST VERSION for RPi5:
- Buffered state machine parser with proper resync
- No in_waiting checks (causes race condition on RPi5)
- Automatic reconnection on serial errors
- Buffer size limit to prevent memory leaks
- DTR disabled to prevent ESP32 reset on connect
- Short non-blocking reads (no timer backlog)

Usage:
    ros2 run blitz blitz_node.py
"""

import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from blitz import Blitz
from interfaces import blitz_interfaces
from std_msgs.msg import String


# Configuration constants
MAX_BUFFER_SIZE = 4096          # Max bytes to buffer before discarding
MAX_DEBUG_MSG_LEN = 256         # Max debug message length from MCU
RECONNECT_COOLDOWN_S = 2.0      # Seconds between reconnect attempts
SERIAL_TIMEOUT_S = 0.005        # 5ms timeout (shorter than 10ms timer)


class BlitzNode(Node):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__("blitz_node")
        
        # Declare parameters
        self.declare_parameter('port', port)
        self.declare_parameter('baud', baud)
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True
        
        # Receive buffer for stateful parsing
        self.rx_buffer = bytearray()
        
        # Reconnect state
        self._last_reconnect_attempt = 0.0
        
        # Try to open serial port
        self._open_serial()
        
        # Callback group for concurrent execution
        self.cb_group = ReentrantCallbackGroup()
        
        # ============ Publishers (MCU → ROS) ============
        self.schema_by_id = {}
        for name, schema in blitz_interfaces.items():
            if schema.from_mcu:
                schema.pub = self.create_publisher(schema.ros_msg, schema.topic, 10)
                self.schema_by_id[schema.id] = schema
                self.get_logger().info(f"[PUB] {schema.topic}")
        
        # Debug publisher
        self.debug_pub = self.create_publisher(String, "/debug", 10)
        
        # ============ Subscribers (ROS → MCU) ============
        for name, schema in blitz_interfaces.items():
            if not schema.from_mcu:
                self.create_subscription(
                    schema.ros_msg,
                    schema.topic,
                    self._make_sub_callback(schema),
                    10,
                    callback_group=self.cb_group
                )
                self.get_logger().info(f"[SUB] {schema.topic}")
        
        # ============ Serial Read Timer ============
        self.create_timer(0.01, self._serial_read_tick, callback_group=self.cb_group)
        
        self.get_logger().info(f"BlitzNode started on {self.port} @ {self.baud}")
    
    def _open_serial(self):
        """Open serial port with robust settings for ESP32."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=1.0,
                # CRITICAL: Disable DTR to prevent ESP32 auto-reset on connect
                dsrdtr=False,
                rtscts=False
            )
            
            # Some systems need explicit DTR/RTS control
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except:
                pass
            
            # Flush buffers (discard boot garbage from ESP32)
            time.sleep(0.1)  # Brief delay for ESP32 boot
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.rx_buffer.clear()
            self.get_logger().info(f"Opened serial port {self.port}")
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {self.port}: {e}")
            self.ser = None
            return False
    
    def _try_reconnect(self):
        """Attempt reconnection with cooldown to prevent rapid retries."""
        now = time.time()
        if now - self._last_reconnect_attempt < RECONNECT_COOLDOWN_S:
            return False
        
        self._last_reconnect_attempt = now
        self.get_logger().warn(f"Attempting to reconnect to {self.port}...")
        
        # Close existing connection if any
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
        
        return self._open_serial()
    
    def _make_sub_callback(self, schema: Blitz):
        """Create a callback for a ROS subscription that writes to serial."""
        def callback(msg):
            with self.ser_lock:
                if not self.ser or not self.ser.is_open:
                    return
                try:
                    packet = schema.pack(msg)
                    self.ser.write(packet)
                    self.get_logger().debug(f"TX: {schema.topic}")
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial write error: {e}")
                    # Schedule reconnection
                    try:
                        self.ser.close()
                    except:
                        pass
                    self.ser = None
        return callback
    
    def _get_packet_payload_size(self, id_byte):
        """Get expected payload size for a packet ID.
        
        Returns:
            int: Payload size in bytes
            -1: Variable length (debug message)
            None: Unknown ID
        """
        if id_byte == 99:  # Debug message - variable length
            return -1
        if id_byte == 85:  # Heartbeat/sync - no payload (ID 0x55)
            return 0
        if id_byte in self.schema_by_id:
            return struct.calcsize("=" + self.schema_by_id[id_byte].struct)
        return None
    
    def _serial_read_tick(self):
        """Timer callback to read from serial port."""
        with self.ser_lock:
            # Handle disconnected state
            if not self.ser or not self.ser.is_open:
                self._try_reconnect()
                return
            
            try:
                # Non-blocking read with very short timeout
                new_data = self.ser.read(256)
                if new_data:
                    self.rx_buffer.extend(new_data)
                
                # Prevent buffer overflow (memory protection)
                if len(self.rx_buffer) > MAX_BUFFER_SIZE:
                    self.get_logger().warn(f"Buffer overflow ({len(self.rx_buffer)} bytes), clearing")
                    self.rx_buffer.clear()
                    return
                
                # Process buffer
                self._process_buffer()
                
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                self.rx_buffer.clear()
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None  # Will trigger reconnect on next tick
                
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
    
    def _process_buffer(self):
        """Process receive buffer and extract complete packets.
        
        State machine that:
        1. Scans for 0xAA header
        2. Validates packet ID
        3. Waits for complete payload
        4. Publishes to ROS
        5. Removes processed data from buffer
        """
        max_iterations = 100
        iterations = 0
        
        while len(self.rx_buffer) >= 2 and iterations < max_iterations:
            iterations += 1
            
            # ===== STEP 1: Find header byte 0xAA =====
            if self.rx_buffer[0] != 0xAA:
                # Not a header - discard and continue scanning
                self.rx_buffer.pop(0)
                continue
            
            # ===== STEP 2: Read packet ID =====
            id_byte = self.rx_buffer[1]
            payload_size = self._get_packet_payload_size(id_byte)
            
            # ===== STEP 3: Handle unknown IDs =====
            if payload_size is None:
                # Unknown ID - likely stream corruption
                # Discard the 0xAA (the ID byte might be a valid header in disguise)
                self.rx_buffer.pop(0)
                continue
            
            # ===== STEP 4: Handle debug messages (ID=99) =====
            if id_byte == 99:
                if len(self.rx_buffer) < 3:
                    return  # Wait for length byte
                
                length = self.rx_buffer[2]
                
                # Sanity check length
                if length > MAX_DEBUG_MSG_LEN:
                    # Invalid length - likely corruption
                    self.rx_buffer.pop(0)
                    continue
                
                total_size = 3 + length  # header + id + length + data
                
                if len(self.rx_buffer) < total_size:
                    return  # Wait for complete message
                
                # Extract and publish debug message
                data = bytes(self.rx_buffer[3:total_size])
                msg = String()
                msg.data = data.decode('utf-8', errors='replace')
                self.debug_pub.publish(msg)
                self.get_logger().info(f"[MCU] {msg.data}")
                
                del self.rx_buffer[:total_size]
                continue
            
            # ===== STEP 5: Handle heartbeat (ID=85, no payload) =====
            if id_byte == 85:
                del self.rx_buffer[:2]  # header + id
                continue
            
            # ===== STEP 6: Handle schema-defined packets =====
            total_size = 2 + payload_size  # header + id + payload
            
            if len(self.rx_buffer) < total_size:
                return  # Wait for complete packet
            
            payload = bytes(self.rx_buffer[2:total_size])
            
            try:
                schema = self.schema_by_id[id_byte]
                ros_msg = schema.unpack(payload)
                schema.pub.publish(ros_msg)
            except Exception as e:
                # Unpack failed - could be corrupted data
                self.get_logger().debug(f"Unpack failed for ID {id_byte}: {e}")
            
            del self.rx_buffer[:total_size]
    
    def destroy_node(self):
        """Clean up serial port on shutdown."""
        self.running = False
        with self.ser_lock:
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
        super().destroy_node()


def main():
    rclpy.init()
    
    node = BlitzNode()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
