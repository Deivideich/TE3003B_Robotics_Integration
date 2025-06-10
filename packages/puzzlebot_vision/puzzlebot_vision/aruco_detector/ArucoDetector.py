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
        self.object_points = self._get_marker_object_points()

    def _load_calibration(self):
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

    def _get_marker_object_points(self):
        """Return 3D object points of the marker in its own coordinate system."""
        half_len = self.marker_length / 2.0
        return np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []

        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i]
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints=self.object_points,
                    imagePoints=corner,
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs
                )

                if success:
                    result = {
                        'id': int(ids[i][0]),
                        'corner': corner,
                        'rvec': rvec.flatten(),
                        'tvec': tvec.flatten()
                    }
                    results.append(result)

        return results
