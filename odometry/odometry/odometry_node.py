#!/usr/bin/env python3
"""
Wheel encoder odometry node.

Reads quadrature encoder pulses from GPIO and publishes:
  /odom  (nav_msgs/Odometry)
  tf: odom → base_link
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.declare_parameters(namespace='', parameters=[
            ('wheel_separation',    0.20),
            ('wheel_radius',        0.0325),
            ('ticks_per_revolution', 480),
            ('pin_enc_left_a',  5),
            ('pin_enc_left_b',  6),
            ('pin_enc_right_a', 19),
            ('pin_enc_right_b', 26),
            ('publish_rate',    20.0),
        ])

        p = lambda n: self.get_parameter(n).value
        self.wheel_sep   = p('wheel_separation')
        self.wheel_rad   = p('wheel_radius')
        self.ticks_per_rev = p('ticks_per_revolution')

        # metres per tick
        self.dist_per_tick = (2 * math.pi * self.wheel_rad) / self.ticks_per_rev

        # Robot pose
        self.x = self.y = self.theta = 0.0

        # Encoder tick counters
        self._left_ticks  = 0
        self._right_ticks = 0
        self._prev_left   = 0
        self._prev_right  = 0

        self._setup_gpio(p)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / p('publish_rate'), self._publish)
        self.get_logger().info('Odometry node ready')

    def _setup_gpio(self, p):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [p('pin_enc_left_a'),  p('pin_enc_left_b'),
                    p('pin_enc_right_a'), p('pin_enc_right_b')]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Count rising edges on channel A
        GPIO.add_event_detect(p('pin_enc_left_a'),  GPIO.RISING,
                              callback=self._left_cb)
        GPIO.add_event_detect(p('pin_enc_right_a'), GPIO.RISING,
                              callback=self._right_cb)

    def _left_cb(self, channel):
        # Channel B determines direction
        if GPIO.input(self.get_parameter('pin_enc_left_b').value):
            self._left_ticks += 1
        else:
            self._left_ticks -= 1

    def _right_cb(self, channel):
        if GPIO.input(self.get_parameter('pin_enc_right_b').value):
            self._right_ticks -= 1   # right motor is mirrored
        else:
            self._right_ticks += 1

    def _publish(self):
        # How many ticks since last publish?
        dl = (self._left_ticks  - self._prev_left)  * self.dist_per_tick
        dr = (self._right_ticks - self._prev_right) * self.dist_per_tick
        self._prev_left  = self._left_ticks
        self._prev_right = self._right_ticks

        # Differential drive kinematics
        dist  = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_sep

        self.x     += dist * math.cos(self.theta + dtheta / 2)
        self.y     += dist * math.sin(self.theta + dtheta / 2)
        self.theta += dtheta

        now = self.get_clock().now().to_msg()

        # Publish TF transform odom → base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = math.sin(self.theta / 2)
        t.transform.rotation.w    = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        msg = Odometry()
        msg.header.stamp    = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x  = self.x
        msg.pose.pose.position.y  = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        dl_dt = dl * p('publish_rate') if (dl := dl) else 0.0
        msg.twist.twist.linear.x  = dist   * self.get_parameter('publish_rate').value
        msg.twist.twist.angular.z = dtheta * self.get_parameter('publish_rate').value
        self.odom_pub.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
