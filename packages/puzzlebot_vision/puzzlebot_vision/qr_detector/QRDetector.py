from puzzlebot_interfaces.msg import QRCode
from typing import List
import cv2
import numpy as np
from numpy.typing import NDArray
import os
import sys

class QRDetector:
    def __init__(self, qr_size=0.05):
        self.qr_size = qr_size
        self.camera_matrix = None
        self.dist_coeffs = None
        self.qr_detector = cv2.QRCodeDetector()
        self.object_points = self._get_qr_object_points()

        self._load_calibration()

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

    def _get_qr_object_points(self):
        """Return 3D object points of the QR code in its own coordinate system."""
        half_len = self.qr_size / 2.0
        return np.array([
            [-half_len,  half_len, 0],  # top-left
            [ half_len,  half_len, 0],  # top-right
            [ half_len, -half_len, 0],  # bottom-right
            [-half_len, -half_len, 0]   # bottom-left
        ], dtype=np.float32)

    def detect(self, frame: NDArray) -> List[QRCode]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = frame.shape[:2]
        
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray)
        detected_codes = []

        if retval:
            for content, corner in zip(decoded_info, points):
                if content and corner.shape[0] == 4:
                    image_points = corner.astype(np.float32)

                    success, rvec, tvec = cv2.solvePnP(
                        objectPoints=self.object_points,
                        imagePoints=image_points,
                        cameraMatrix=self.camera_matrix,
                        distCoeffs=self.dist_coeffs
                    )

                    if success:
                        # Get normalized coordinates
                        x_coords = corner[:, 0] / width
                        y_coords = corner[:, 1] / height
                        
                        
                        result = {
                            'content': content,
                            'corner': corner,
                            'rvec': rvec.flatten(),
                            'tvec': tvec.flatten()
                        }

                        qr = QRCode()
                        qr.x1 = float(min(x_coords))
                        qr.x2 = float(max(x_coords))
                        qr.y1 = float(min(y_coords))
                        qr.y2 = float(max(y_coords))
                        qr.content = content
                        qr.rvec = rvec.flatten().astype(np.float32).tolist()
                        qr.tvec = tvec.flatten().astype(np.float32).tolist()

                        detected_codes.append(qr)

        return detected_codes

