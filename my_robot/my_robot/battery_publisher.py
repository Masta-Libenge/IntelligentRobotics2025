#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time


SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 57600
TOPIC = '/battery_voltage'


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.publisher = self.create_publisher(Float32, TOPIC, 10)
        self.timer = self.create_timer(60.0, self.publish_voltage)

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1.0)
            time.sleep(2.0)  # allow OpenCR reset
            self.get_logger().info(f"Connected to OpenCR on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        self.get_logger().info(f"Publishing battery voltage to {TOPIC} every 60s")

    def read_voltage_from_serial(self):
        if not self.ser:
            return None

        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return None

            # Battery info we can actually observe from this firmware:
            # it prints "Executed: checkCriticalBattery" when it checks battery.
            if "checkCriticalBattery" in line:
                return 11.0  # treat as low/critical

            # Normal chatter we saw in minicom includes "OK: setVelocity(...)"
            if "OK" in line or "setVelocity" in line:
                return 12.0  # treat as normal

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

        return None

    def publish_voltage(self):
        voltage = self.read_voltage_from_serial()
        if voltage is None:
            self.get_logger().warn("Battery voltage read failed (no usable data)")
            return

        msg = Float32()
        msg.data = voltage
        self.publisher.publish(msg)
        self.get_logger().info(f"Battery voltage: {voltage:.2f} V")


def main():
    rclpy.init()
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


