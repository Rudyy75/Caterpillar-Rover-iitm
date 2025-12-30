#!/usr/bin/env python3
# subscribes from ros2 topics and packs data to the serial port
import rclpy
from rclpy.node import Node
import serial
from blitz import Blitz
from interfaces import blitz_interfaces
from rclpy.executors import SingleThreadedExecutor

class SerialSender(Node):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__("serial_sender")
        self.schema = blitz_interfaces
     
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        # Create subscriptions
        for name, schema in blitz_interfaces.items():
            if not schema.from_mcu:
                self.create_subscription(
                    schema.ros_msg,
                    schema.topic,
                    self.sub_callback(schema),
                    10
                )
                self.get_logger().info(f"Subscribed to {schema.topic}")

    def sub_callback(self, schema : Blitz):
        def callback(msg):
            if not self.ser:
                return
            packet = schema.pack(msg)
            self.ser.write(packet)
            self.get_logger().info(f"Writing to Serial {msg}")
        return callback
     
def main():
    rclpy.init()

    sender = SerialSender()

    executor = SingleThreadedExecutor()
    executor.add_node(sender)
    
    try:
        executor.spin()  # either spin in multithreaded executor, or run separately
    except KeyboardInterrupt or serial.SerialException as e:
        sender.get_logger().error(f"Serial error: {e}")
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()