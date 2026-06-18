#!/usr/bin/env python3
"""HC-SR04 ultrasonic distance sensor node. Publishes to /ultrasonic/front."""
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic')
        self.declare_parameters(namespace='', parameters=[
            ('ultrasonic_trig_pin', 24), ('ultrasonic_echo_pin', 25),
            ('ultrasonic_max_range', 4.0), ('ultrasonic_publish_rate', 10.0)])
        p = lambda n: self.get_parameter(n).value
        self.TRIG, self.ECHO = p('ultrasonic_trig_pin'), p('ultrasonic_echo_pin')
        self.max_range = p('ultrasonic_max_range')
        GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
        GPIO.setup(self.TRIG, GPIO.OUT); GPIO.setup(self.ECHO, GPIO.IN)
        self.pub = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.create_timer(1.0 / p('ultrasonic_publish_rate'), self._measure)

    def _measure(self):
        GPIO.output(self.TRIG, True); time.sleep(0.00001); GPIO.output(self.TRIG, False)
        t0 = time.time()
        while GPIO.input(self.ECHO) == 0 and time.time() - t0 < 0.03: pass
        t1 = time.time()
        while GPIO.input(self.ECHO) == 1 and time.time() - t1 < 0.03: pass
        dist = (time.time() - t1) * 34300 / 2 / 100  # cm → meters
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02; msg.max_range = self.max_range
        msg.range = float(min(max(dist, 0.0), self.max_range))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: GPIO.cleanup(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
