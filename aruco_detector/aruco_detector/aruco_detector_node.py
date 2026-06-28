#!/usr/bin/env python3
"""
ArUco marker detection + PID visual servoing node.

Subscriptions:
  /camera/image_raw   (sensor_msgs/Image)

Published:
  /camera/aruco_view  (sensor_msgs/Image)   — annotated image
  /cmd_vel            (geometry_msgs/Twist) — PID correction during APPROACHING_DOCK
  /docking_complete   (std_msgs/Bool)
  /robot_state        (std_msgs/String)     — read to know when to activate
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameters(namespace='', parameters=[
            ('aruco_dict', 'DICT_4X4_50'),
            ('dock_marker_id', 0),
            ('dock_approach_distance', 0.15),
            ('pid_lateral_kp', 0.003),
            ('pid_lateral_ki', 0.0001),
            ('pid_lateral_kd', 0.001),
            ('approach_linear_speed', 0.1),
        ])

        p = lambda n: self.get_parameter(n).value
        self.dock_id    = p('dock_marker_id')
        self.stop_dist  = p('dock_approach_distance')
        self.kp         = p('pid_lateral_kp')
        self.ki         = p('pid_lateral_ki')
        self.kd         = p('pid_lateral_kd')
        self.approach_v = p('approach_linear_speed')

        aruco_dict_name   = p('aruco_dict')
        self.aruco_dict   = aruco.getPredefinedDictionary(getattr(aruco, aruco_dict_name))
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.active = False
        self._integral   = 0.0
        self._prev_error = 0.0

        # Camera calibration — replace with calibrated values after running camera_calibration
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs   = np.zeros((5, 1))
        self.marker_size   = 0.10  # metres — must match the physical marker

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub  = self.create_publisher(Image, '/camera/aruco_view', 10)
        self.dock_pub = self.create_publisher(Bool, '/docking_complete', 10)

        self.create_subscription(Image,  '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(String, '/robot_state',      self._state_cb, 10)

        self.get_logger().info('ArUco detector ready')

    def _state_cb(self, msg: String):
        self.active = (msg.data == 'APPROACHING_DOCK')
        if not self.active:
            self._integral   = 0.0
            self._prev_error = 0.0

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            # 3D object points for a square marker centred at origin
            half = self.marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.dock_id:
                    # estimatePoseSingleMarkers was removed in OpenCV 4.8+
                    # solvePnP is the correct replacement — same result, explicit API
                    img_pts = corners[i][0].astype(np.float32)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_pts, img_pts, self.camera_matrix, self.dist_coeffs)
                    if not ok:
                        continue
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                      rvec, tvec, 0.05)
                    z_dist = float(tvec[2][0])  # distance along camera Z axis
                    if self.active:
                        self._servo(corners[i], z_dist, frame.shape[1])

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    def _servo(self, corner, z_dist: float, img_width: int):
        """PID lateral correction. Error = pixels the marker centre is off-screen-centre."""
        cx    = int(np.mean(corner[0][:, 0]))
        error = cx - img_width / 2   # positive → marker is right of centre

        self._integral  += error * 0.033
        derivative       = (error - self._prev_error) / 0.033
        angular_z        = -(self.kp * error + self.ki * self._integral + self.kd * derivative)
        self._prev_error = error

        twist           = Twist()
        twist.angular.z = float(np.clip(angular_z, -1.0, 1.0))

        if z_dist > self.stop_dist:
            twist.linear.x = self.approach_v
        else:
            twist.linear.x = 0.04  # creep forward for final contact

        if z_dist < 0.05:  # ~5 cm — contact
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.dock_pub.publish(Bool(data=True))
            return

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
