#!/usr/bin/env python3
"""INA219 battery monitor node. Publishes to /battery_state."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import board, busio
from adafruit_ina219 import INA219

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.declare_parameters(namespace='', parameters=[
            ('battery_full_voltage', 12.6), ('battery_empty_voltage', 9.6),
            ('battery_publish_rate', 1.0)])
        p = lambda n: self.get_parameter(n).value
        self.v_full, self.v_empty = p('battery_full_voltage'), p('battery_empty_voltage')
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina = INA219(i2c)
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_timer(1.0 / p('battery_publish_rate'), self._publish)

    def _publish(self):
        v = self.ina.bus_voltage + self.ina.shunt_voltage / 1000
        a = self.ina.current / 1000
        pct = max(0.0, min(1.0, (v - self.v_empty) / (self.v_full - self.v_empty)))
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(v); msg.current = float(a); msg.percentage = float(pct)
        msg.present = True
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
