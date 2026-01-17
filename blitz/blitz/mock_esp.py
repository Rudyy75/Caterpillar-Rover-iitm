#!/usr/bin/env python3
"""
Mock ESP32 Simulator for testing Blitz communication without hardware.

Creates a virtual serial port pair using socat, then simulates the ESP32:
- Sends periodic sensor data (EncoderRaw, BnoReading, LimitSwitches)
- Receives and parses velocity commands
- Sends debug messages

Usage:
    # Terminal 1: Start the mock ESP
    python3 mock_esp.py
    
    # Terminal 2: Launch blitz with the virtual port
    ros2 launch blitz blitz.launch.py port:=/tmp/ttyVESP0
    
Requirements:
    sudo apt install socat
"""

import os
import subprocess
import time
import struct
import threading
import sys

# Packet IDs (must match blitz_interfaces.hpp)
VELOCITY = 1
ENCODER_RAW = 2
LIMIT_SWITCHES = 3
MODE_SWITCH = 4
BNO_READING = 5
ACTUATOR_CMD = 6
DEBUG_MSG = 99

HEADER = 0xAA


class MockESP32:
    def __init__(self):
        self.virtual_port = "/tmp/ttyVESP0"
        self.link_port = "/tmp/ttyVESP1"
        self.socat_proc = None
        self.serial_fd = None
        self.running = False
        
        # Simulated sensor values
        self.encoder_ticks = [0, 0, 0, 0]  # fl, fr, bl, br
        self.bno_yaw = 0.0
        self.limit_switches = [False, False, False]
        
    def start(self):
        """Start the virtual serial port and ESP simulation."""
        print("Starting Mock ESP32 Simulator...")
        
        # Create virtual serial port pair using socat
        self._start_socat()
        time.sleep(1)  # Wait for socat to create ports
        
        # Open the link port (we write/read from this end)
        try:
            self.serial_fd = os.open(self.link_port, os.O_RDWR | os.O_NOCTTY)
            print(f"✓ Mock ESP connected to {self.link_port}")
            print(f"✓ ROS should connect to: {self.virtual_port}")
        except Exception as e:
            print(f"✗ Failed to open {self.link_port}: {e}")
            return False
        
        self.running = True
        
        # Start threads
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._transmit_loop, daemon=True)
        
        self.rx_thread.start()
        self.tx_thread.start()
        
        print("\n" + "="*50)
        print("Mock ESP32 is running!")
        print(f"Connect ROS to: {self.virtual_port}")
        print("="*50 + "\n")
        
        # Send initial debug message
        self._send_debug("Mock ESP32 Started!")
        self._send_debug("BNO055 OK (simulated)")
        self._send_debug("Encoders OK (simulated)")
        
        return True
    
    def _start_socat(self):
        """Create virtual serial port pair."""
        # Kill any existing socat for these ports
        os.system(f"pkill -f 'socat.*{self.virtual_port}'")
        time.sleep(0.2)
        
        # Create the virtual port pair
        cmd = [
            "socat",
            "-d", "-d",
            f"pty,raw,echo=0,link={self.virtual_port}",
            f"pty,raw,echo=0,link={self.link_port}"
        ]
        
        self.socat_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"✓ Created virtual serial ports")
    
    def _send_packet(self, packet_id: int, data: bytes):
        """Send a Blitz packet."""
        if not self.serial_fd:
            return
        
        packet = bytes([HEADER, packet_id]) + data
        try:
            os.write(self.serial_fd, packet)
        except Exception as e:
            print(f"TX Error: {e}")
    
    def _send_debug(self, message: str):
        """Send a debug message."""
        msg_bytes = message.encode('utf-8')
        length = len(msg_bytes)
        data = bytes([length]) + msg_bytes
        self._send_packet(DEBUG_MSG, data)
        print(f"[TX DEBUG] {message}")
    
    def _send_encoder_raw(self):
        """Send encoder tick data."""
        # Simulate some movement
        self.encoder_ticks[2] += 10  # BL
        self.encoder_ticks[3] += 10  # BR
        
        data = struct.pack("<iiii", *self.encoder_ticks)
        self._send_packet(ENCODER_RAW, data)
    
    def _send_bno_reading(self):
        """Send IMU orientation."""
        # Simulate slow rotation
        self.bno_yaw = (self.bno_yaw + 0.5) % 360.0
        
        data = struct.pack("<fff", self.bno_yaw, 0.0, 0.0)  # yaw, pitch, roll
        self._send_packet(BNO_READING, data)
    
    def _send_limit_switches(self):
        """Send limit switch states."""
        data = struct.pack("???", *self.limit_switches)
        self._send_packet(LIMIT_SWITCHES, data)
    
    def _transmit_loop(self):
        """Periodically send sensor data (20Hz like real ESP)."""
        while self.running:
            self._send_encoder_raw()
            self._send_bno_reading()
            self._send_limit_switches()
            time.sleep(0.05)  # 20Hz
    
    def _receive_loop(self):
        """Receive and parse commands from ROS."""
        buffer = b""
        
        while self.running:
            try:
                data = os.read(self.serial_fd, 256)
                if data:
                    buffer += data
                    buffer = self._parse_buffer(buffer)
            except Exception as e:
                time.sleep(0.01)
    
    def _parse_buffer(self, buffer: bytes) -> bytes:
        """Parse incoming Blitz packets."""
        while len(buffer) >= 2:
            # Find header
            if buffer[0] != HEADER:
                buffer = buffer[1:]
                continue
            
            packet_id = buffer[1]
            
            # Determine expected size
            if packet_id == VELOCITY:
                expected_size = 2 + 12  # header + id + 3 floats
                if len(buffer) >= expected_size:
                    payload = buffer[2:expected_size]
                    vx, vy, vw = struct.unpack("<fff", payload)
                    print(f"[RX VELOCITY] vx={vx:.2f} vy={vy:.2f} vw={vw:.2f}")
                    buffer = buffer[expected_size:]
                else:
                    break
                    
            elif packet_id == ACTUATOR_CMD:
                expected_size = 2 + 2  # header + id + 2 bytes
                if len(buffer) >= expected_size:
                    payload = buffer[2:expected_size]
                    lead_screw, tub_angle = struct.unpack("<BB", payload)
                    print(f"[RX ACTUATOR] lead_screw={lead_screw} tub_angle={tub_angle}")
                    buffer = buffer[expected_size:]
                else:
                    break
            else:
                # Unknown ID, skip
                buffer = buffer[1:]
        
        return buffer
    
    def stop(self):
        """Clean up."""
        self.running = False
        
        if self.serial_fd:
            try:
                os.close(self.serial_fd)
            except:
                pass
        
        if self.socat_proc:
            self.socat_proc.terminate()
            self.socat_proc.wait()
        
        print("\nMock ESP32 stopped.")


def main():
    mock = MockESP32()
    
    if not mock.start():
        sys.exit(1)
    
    try:
        print("Press Ctrl+C to stop...\n")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mock.stop()


if __name__ == "__main__":
    main()
