#!/usr/bin/env python3
"""
Robot state machine node.

States: IDLE, LOW_BATTERY, SEEKING_DOCK, APPROACHING_DOCK, DOCKED, UNDOCKING

Subscriptions:
  /battery_state      (sensor_msgs/BatteryState)
  /ir_dock_detected   (std_msgs/Bool)
  /docking_complete   (std_msgs/Bool)
  /trigger_dock       (std_msgs/Bool)  — dashboard button

Published:
  /robot_state        (std_msgs/String)
  /cmd_vel            (geometry_msgs/Twist) — during seek/undock
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from enum import Enum, auto


class State(str, Enum):
    IDLE             = 'IDLE'
    LOW_BATTERY      = 'LOW_BATTERY'
    SEEKING_DOCK     = 'SEEKING_DOCK'
    APPROACHING_DOCK = 'APPROACHING_DOCK'
    DOCKED           = 'DOCKED'
    UNDOCKING        = 'UNDOCKING'


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.declare_parameters(namespace='', parameters=[
            ('battery_low_threshold',     0.20),
            ('battery_charged_threshold', 0.80),
            ('dock_x', 2.0), ('dock_y', 0.5),
            ('dock_approach_x', 1.5), ('dock_approach_y', 0.5),
            ('seek_angular_speed', 0.3),
            ('approach_linear_speed', 0.1),
            ('undock_distance', 0.5),
        ])

        p = lambda n: self.get_parameter(n).value
        self.low_thresh     = p('battery_low_threshold')
        self.charged_thresh = p('battery_charged_threshold')
        self.seek_omega     = p('seek_angular_speed')
        self.approach_v     = p('approach_linear_speed')

        self.state = State.IDLE
        self.battery_pct = 1.0
        self.ir_detected = False

        # Publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.create_subscription(BatteryState, '/battery_state',    self._battery_cb, 10)
        self.create_subscription(Bool,         '/ir_dock_detected', self._ir_cb,      10)
        self.create_subscription(Bool,         '/docking_complete', self._dock_cb,    10)
        self.create_subscription(Bool,         '/trigger_dock',     self._trigger_cb, 10)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.1, self._update)
        self.get_logger().info('State machine ready — state: IDLE')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _battery_cb(self, msg: BatteryState):
        self.battery_pct = msg.percentage

    def _ir_cb(self, msg: Bool):
        self.ir_detected = msg.data

    def _dock_cb(self, msg: Bool):
        if msg.data and self.state == State.APPROACHING_DOCK:
            self._transition(State.DOCKED)

    def _trigger_cb(self, msg: Bool):
        if msg.data and self.state == State.IDLE:
            self._transition(State.LOW_BATTERY)

    # ── Main update loop ───────────────────────────────────────────────────

    def _update(self):
        if self.state == State.IDLE:
            if self.battery_pct < self.low_thresh:
                self._transition(State.LOW_BATTERY)

        elif self.state == State.LOW_BATTERY:
            self._send_nav_goal(self.get_parameter('dock_approach_x').value,
                                self.get_parameter('dock_approach_y').value)
            self._transition(State.SEEKING_DOCK)

        elif self.state == State.SEEKING_DOCK:
            if self.ir_detected:
                self._transition(State.APPROACHING_DOCK)
            else:
                # Rotate in place to find IR beacon
                twist = Twist()
                twist.angular.z = self.seek_omega
                self.cmd_pub.publish(twist)

        elif self.state == State.DOCKED:
            if self.battery_pct >= self.charged_thresh:
                self._transition(State.UNDOCKING)

        self.state_pub.publish(String(data=str(self.state.value)))

    def _transition(self, new_state: State):
        self.get_logger().info(f'State: {self.state} → {new_state}  (battery: {self.battery_pct:.0%})')
        self.state = new_state
        # Stop motors on any transition
        self.cmd_pub.publish(Twist())

    def _send_nav_goal(self, x: float, y: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self._nav_client.wait_for_server(timeout_sec=5.0)
        self._nav_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
