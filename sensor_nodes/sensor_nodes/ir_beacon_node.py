#!/usr/bin/env python3
"""IR dock beacon detection node. Publishes /ir_dock_detected as Bool."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class IRBeaconNode(Node):
    def __init__(self):
        super().__init__('ir_beacon')
        self.declare_parameters(namespace='', parameters=[('ir_receiver_pin', 21)])
        self.PIN = self.get_parameter('ir_receiver_pin').value
        GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
        GPIO.setup(self.PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.pub = self.create_publisher(Bool, '/ir_dock_detected', 10)
        self.create_timer(0.05, self._read)  # 20 Hz

    def _read(self):
        # TSOP38238: output is LOW when IR signal detected
        detected = GPIO.input(self.PIN) == GPIO.LOW
        self.pub.publish(Bool(data=detected))

def main(args=None):
    rclpy.init(args=args)
    node = IRBeaconNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: GPIO.cleanup(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
