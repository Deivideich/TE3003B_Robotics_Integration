#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Adjustable: side length of ArUco marker in meters
ARUCO_MARKER_LENGTH = 0.05  # e.g. 5 cm
CALIBRATION_FILE = "calibration.npz"


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        package_share_dir = get_package_share_directory('puzzlebot_navigation')
        # Load camera calibration
        self.calibration_file = os.path.join(package_share_dir, 'resources', 'calibration.npz')
        if not os.path.exists(self.calibration_file):
            self.get_logger().error(f"Calibration file '{self.calibration_file}' not found.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Found calibration file: '{self.calibration_file}'")
        with np.load(self.calibration_file) as X:
            self.camera_matrix = X["camMatrix"]
            self.dist_coeffs = X["distCoeff"]

        # Setup ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # Start video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            rclpy.shutdown()
            return

        self.get_logger().info("Aruco detector node started.")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame grab failed.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_MARKER_LENGTH, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[i], tvecs[i], ARUCO_MARKER_LENGTH * 0.5)

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                self.get_logger().info(f"Marker ID {ids[i][0]}: x={tvec[0]:.2f}, y={tvec[1]:.2f}, z={tvec[2]:.2f}")

        cv2.imshow("Aruco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quitting ArUco detector node.")
            self.destroy_node()
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
