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
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from enum import Enum


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

        self.state       = State.IDLE
        self.battery_pct = 1.0
        self.ir_detected = False

        # Bug fix: without this flag, LOW_BATTERY sent a Nav2 goal every 100ms
        self._nav_goal_sent = False
        # Bug fix: tracks when undocking started so we can time the backup
        self._undock_start  = None

        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',     10)

        self.create_subscription(BatteryState, '/battery_state',    self._battery_cb, 10)
        self.create_subscription(Bool,         '/ir_dock_detected', self._ir_cb,      10)
        self.create_subscription(Bool,         '/docking_complete', self._dock_cb,    10)
        self.create_subscription(Bool,         '/trigger_dock',     self._trigger_cb, 10)

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
        """Dashboard 'send to dock' button."""
        if msg.data and self.state == State.IDLE:
            self._transition(State.LOW_BATTERY)

    # ── Main loop ──────────────────────────────────────────────────────────

    def _update(self):
        if self.state == State.IDLE:
            if self.battery_pct < self.low_thresh:
                self._transition(State.LOW_BATTERY)

        elif self.state == State.LOW_BATTERY:
            # Send Nav2 goal ONCE — flag prevents re-sending every 100ms
            if not self._nav_goal_sent:
                self._send_nav_goal(
                    self.get_parameter('dock_approach_x').value,
                    self.get_parameter('dock_approach_y').value)
                self._nav_goal_sent = True
                self._transition(State.SEEKING_DOCK)

        elif self.state == State.SEEKING_DOCK:
            if self.ir_detected:
                self._transition(State.APPROACHING_DOCK)
            else:
                twist = Twist()
                twist.angular.z = self.seek_omega
                self.cmd_pub.publish(twist)

        elif self.state == State.DOCKED:
            if self.battery_pct >= self.charged_thresh:
                self._transition(State.UNDOCKING)

        elif self.state == State.UNDOCKING:
            # Back up for (undock_distance / 0.1 m/s) seconds, then go idle
            if self._undock_start is None:
                self._undock_start = self.get_clock().now()

            undock_dist  = self.get_parameter('undock_distance').value
            backup_speed = 0.1
            duration_sec = undock_dist / backup_speed
            elapsed = (self.get_clock().now() - self._undock_start).nanoseconds / 1e9

            if elapsed < duration_sec:
                twist = Twist()
                twist.linear.x = -backup_speed
                self.cmd_pub.publish(twist)
            else:
                self._undock_start  = None
                self._nav_goal_sent = False   # reset so next dock cycle works
                self._transition(State.IDLE)

        self.state_pub.publish(String(data=str(self.state.value)))

    def _transition(self, new_state: State):
        self.get_logger().info(
            f'State: {self.state} → {new_state}  (battery: {self.battery_pct:.0%})')
        self.state = new_state
        self.cmd_pub.publish(Twist())  # zero velocity on every transition

    def _send_nav_goal(self, x: float, y: float):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
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
