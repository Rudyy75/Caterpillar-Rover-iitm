#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import struct
import serial
from interfaces import blitz_interfaces
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

class SerialReceiver(Node):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__("serial_receiver")
        self.schema = blitz_interfaces 
        
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        # Create Publisher and Timer
        for name, schema in blitz_interfaces.items():
            if schema.from_mcu:
                schema.pub = self.create_publisher(schema.ros_msg, schema.topic, 10)
                self.get_logger().info(f"Publishing to {schema.topic}")

        self.debug_pub = self.create_publisher(String, "/debug", 10)
        # self.get_logger().info(f"Publishing to {schema.topic}")

        self.schema = {s.id: s for s in blitz_interfaces.values()}

    def serial_read(self):
        if not self.ser:
            return

        if self.ser.in_waiting >= 1:  
            header = self.ser.read(1)
            if header[0] != 0xAA:
                return  # resync
            
            id_byte = self.ser.read(1)[0]  # already got ID

            if id_byte == 99: # debug msg
                length = self.ser.read(1)[0]
                msg = String()
                data = self.ser.read(length)
                msg.data = struct.unpack(f'{length}s', data)[0].decode('utf-8')
                self.debug_pub.publish(msg)

            else:
                try :
                    self.schema[id_byte].payload_data = self.ser.read(struct.calcsize("="+self.schema[id_byte].struct))

                    data = self.schema[id_byte].payload_data
                    if data is not None:
                        msg = self.schema[id_byte].unpack(data)
                        # self.get_logger().info(f"msg received from mcu {msg}")
                        self.schema[id_byte].pub.publish(msg)
                
                except KeyError:
                    self.get_logger().error(f"Received msg ID {id_byte}, ID mismatch, dropping data check configuration")

def main():
    rclpy.init()

    receiver = SerialReceiver()
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    
    try:
        while True:
            receiver.serial_read()
        # executor.spin()  # either spin in multithreaded executor, or run separately
    except KeyboardInterrupt or serial.SerialException as e:
        receiver.get_logger().error(f"Serial error: {e}")
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()