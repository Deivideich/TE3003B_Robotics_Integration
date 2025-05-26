import cv2
import numpy as np
import os
import sys

class ArucoDetector:
    def __init__(self, marker_length=0.05):
        self.marker_length = marker_length
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None

        self._load_calibration()
        self._init_detector()

    def _load_calibration(self):
        # Get the path to the calibration file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        calib_path = os.path.abspath(os.path.join(script_dir, "../resources/calibration.npz"))

        if not os.path.exists(calib_path):
            print(f"❌ Calibration file '{calib_path}' not found.")
            sys.exit(1)

        print(f"✅ Found calibration file: '{calib_path}'")

        with np.load(calib_path) as X:
            self.camera_matrix = X["camMatrix"]
            self.dist_coeffs = X["distCoeff"]

    def _init_detector(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def detect(self, frame):
        """Detect markers in a BGR image frame. Returns detection results."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                result = {
                    'id': int(ids[i][0]),
                    'corner': corners[i],
                    'rvec': rvecs[i][0],
                    'tvec': tvecs[i][0]
                }
                results.append(result)

        return results
