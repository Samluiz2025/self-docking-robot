"""
Main bringup launch file — starts everything on the physical robot.
Usage: ros2 launch robot_bringup robot.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── Paths ──────────────────────────────────────────────────────────────
    bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    slam_params = PathJoinSubstitution([bringup_share, 'config', 'slam_params.yaml'])
    nav2_params = PathJoinSubstitution([bringup_share, 'config', 'nav2_params.yaml'])
    hw_params   = PathJoinSubstitution([bringup_share, 'config', 'hardware_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── Motor controller ───────────────────────────────────────────────
        Node(package='motor_controller', executable='motor_controller_node',
             name='motor_controller', parameters=[hw_params]),

        # ── Odometry ──────────────────────────────────────────────────────
        Node(package='odometry', executable='odometry_node',
             name='odometry', parameters=[hw_params]),

        # ── Sensor nodes ──────────────────────────────────────────────────
        Node(package='sensor_nodes', executable='ultrasonic_node',
             name='ultrasonic', parameters=[hw_params]),
        Node(package='sensor_nodes', executable='battery_node',
             name='battery_monitor', parameters=[hw_params]),
        Node(package='sensor_nodes', executable='ir_beacon_node',
             name='ir_beacon', parameters=[hw_params]),

        # ── Camera + ArUco ────────────────────────────────────────────────
        Node(package='usb_cam', executable='usb_cam_node_exe',
             name='camera', parameters=[hw_params]),
        Node(package='aruco_detector', executable='aruco_detector_node',
             name='aruco_detector', parameters=[hw_params]),

        # ── SLAM Toolbox ──────────────────────────────────────────────────
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[slam_params],
             output='screen'),

        # ── Nav2 ──────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py']),
            launch_arguments={'params_file': nav2_params, 'use_sim_time': use_sim_time}.items()
        ),

        # ── State machine ─────────────────────────────────────────────────
        Node(package='state_machine', executable='state_machine_node',
             name='state_machine', parameters=[hw_params]),

        # ── rosbridge WebSocket server (port 9090) ────────────────────────
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge', parameters=[{'port': 9090}]),
    ])
