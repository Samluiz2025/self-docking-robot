#!/usr/bin/env python3
"""
Motor controller node — converts /cmd_vel Twist messages to L298N GPIO PWM signals.

Subscriptions:
  /cmd_vel  (geometry_msgs/Twist)

Topics published: none
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters (defaults overridden by hardware_params.yaml)
        self.declare_parameters(namespace='', parameters=[
            ('pin_in1', 17), ('pin_in2', 18),
            ('pin_in3', 22), ('pin_in4', 23),
            ('pin_ena', 12), ('pin_enb', 13),
            ('pwm_frequency', 1000),
            ('max_speed', 0.3),
            ('watchdog_timeout', 0.5),
            ('wheel_separation', 0.20),
            ('wheel_radius', 0.0325),
        ])

        p = lambda name: self.get_parameter(name).value
        self.IN1, self.IN2 = p('pin_in1'), p('pin_in2')
        self.IN3, self.IN4 = p('pin_in3'), p('pin_in4')
        self.ENA, self.ENB = p('pin_ena'), p('pin_enb')
        self.max_speed     = p('max_speed')
        self.wheel_sep     = p('wheel_separation')
        self.wheel_rad     = p('wheel_radius')

        self._setup_gpio(p('pwm_frequency'))
        self._last_cmd = self.get_clock().now()
        self._watchdog_timeout = p('watchdog_timeout')

        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info('Motor controller ready')

    def _setup_gpio(self, freq):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB]:
            GPIO.setup(pin, GPIO.OUT)
        self.pwm_a = GPIO.PWM(self.ENA, freq)
        self.pwm_b = GPIO.PWM(self.ENB, freq)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def _cmd_cb(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        lin = msg.linear.x
        ang = msg.angular.z
        # Differential drive kinematics
        v_left  = (lin - ang * self.wheel_sep / 2.0)
        v_right = (lin + ang * self.wheel_sep / 2.0)
        self._set_motors(v_left, v_right)

    def _set_motors(self, v_left: float, v_right: float):
        def to_pwm(v):
            pct = abs(v) / self.max_speed * 100.0
            return min(max(pct, 0.0), 100.0)

        # Left motor direction
        GPIO.output(self.IN1, v_left >= 0)
        GPIO.output(self.IN2, v_left < 0)
        # Right motor direction
        GPIO.output(self.IN3, v_right >= 0)
        GPIO.output(self.IN4, v_right < 0)
        # PWM duty cycle
        self.pwm_a.ChangeDutyCycle(to_pwm(v_left))
        self.pwm_b.ChangeDutyCycle(to_pwm(v_right))

    def _watchdog(self):
        """Stop motors if no cmd_vel received within timeout."""
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout:
            self._set_motors(0.0, 0.0)

    def destroy_node(self):
        self._set_motors(0.0, 0.0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
