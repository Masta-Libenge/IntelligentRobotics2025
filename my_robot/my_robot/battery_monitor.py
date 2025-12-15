#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

TOPIC = '/battery_voltage'
THRESHOLD = 11.5


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.create_subscription(Float32, TOPIC, self.callback, 10)
        self.get_logger().info(
            f"Listening on {TOPIC}, warning if voltage < {THRESHOLD} V"
        )

    def callback(self, msg):
        voltage = msg.data
        if voltage < THRESHOLD:
            self.get_logger().warn(
                f"LOW BATTERY WARNING: {voltage:.2f} V"
            )
        else:
            self.get_logger().info(
                f"Battery OK: {voltage:.2f} V"
            )


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
