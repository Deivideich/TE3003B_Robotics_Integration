from puzzlebot_interfaces.msg import QRCode
from typing import List
import cv2
import numpy as np
from numpy.typing import NDArray
import os
import sys
from pyzbar import pyzbar
from PIL import Image
from qreader import QReader


class QRDetector:
    def __init__(self, qr_size=0.05):
        self.qr_size = qr_size
        self.camera_matrix = None
        self.dist_coeffs = None
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
        
        # Use pyzbar for QR detection
        pil_image = Image.fromarray(gray)
        detected_qrs = pyzbar.decode(pil_image)
        detected_codes = []

        for qr in detected_qrs:
            if qr.type == 'QRCODE':
                content = qr.data.decode('utf-8')
                
                # Extract corner points from pyzbar polygon
                points = qr.polygon
                if len(points) == 4:
                    # Convert to numpy array in the correct format
                    corner = np.array([[p.x, p.y] for p in points], dtype=np.float32)
                    
                    # Reorder points to match expected format: top-left, top-right, bottom-right, bottom-left
                    corner = self._reorder_corners(corner)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        objectPoints=self.object_points,
                        imagePoints=corner,
                        cameraMatrix=self.camera_matrix,
                        distCoeffs=self.dist_coeffs
                    )

                    if success:
                        # Get normalized coordinates
                        x_coords = corner[:, 0] / width
                        y_coords = corner[:, 1] / height

                        qr_msg = QRCode()
                        qr_msg.x1 = float(min(x_coords))
                        qr_msg.x2 = float(max(x_coords))
                        qr_msg.y1 = float(min(y_coords))
                        qr_msg.y2 = float(max(y_coords))
                        qr_msg.content = str(content).replace(" ", "_")
                        qr_msg.rvec = rvec.flatten().astype(np.float32).tolist()
                        qr_msg.tvec = tvec.flatten().astype(np.float32).tolist()

                        detected_codes.append(qr_msg)

        return detected_codes
    
    def _reorder_corners(self, corners):
        """Reorder corners to: top-left, top-right, bottom-right, bottom-left"""
        # Calculate center point
        center = np.mean(corners, axis=0)
        
        # Sort by angle from center
        def angle_from_center(point):
            return np.arctan2(point[1] - center[1], point[0] - center[0])
        
        # Sort corners by angle (counter-clockwise from right)
        sorted_corners = sorted(corners, key=angle_from_center)
        
        # Find top-left corner (smallest sum of x+y)
        sums = [p[0] + p[1] for p in sorted_corners]
        top_left_idx = np.argmin(sums)
        
        # Reorder starting from top-left, going clockwise
        reordered = []
        for i in range(4):
            reordered.append(sorted_corners[(top_left_idx + i) % 4])
        
        return np.array(reordered, dtype=np.float32)



class QRDetectorTorch:
    def _init_(self, qr_size=0.05):
        self.qr_size = qr_size
        self.camera_matrix = None
        self.dist_coeffs = None
        self.qr_detector = cv2.QRCodeDetector()
        self.qreader = QReader()
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
    
    def detect(self, frame: NDArray):
        detected_qrs, detected = self.qreader.detect_and_decode(image=frame, return_detections=True)
        detected_codes = []
        for i, qr in enumerate(detected_qrs):
            if (i > len(detected)) or qr == "" or qr is None or qr == "None": 
                continue

            x1, y1, x2, y2 = detected[i]["bbox_xyxy"]

            # Get the points of the QR code
            points = np.array([
                [x1, y1],
                [x2, y1],
                [x2, y2],
                [x1, y2]
            ], dtype=np.float32)

            # Estimate the pose of the QR code
            success, rvec, tvec = cv2.solvePnP(
                self.object_points, points, self.camera_matrix, self.dist_coeffs
            )
            
            if success:

                qr_msg = QRCode()
                qr_msg.x1 = float(x1)
                qr_msg.x2 = float(x2)
                qr_msg.y1 = float(y1)
                qr_msg.y2 = float(y2)
                qr_msg.content = str(qr).replace(" ", "_")
                qr_msg.rvec = rvec.flatten().astype(np.float32).tolist()
                qr_msg.tvec = tvec.flatten().astype(np.float32).tolist()

                detected_codes.append(qr_msg)

        return detected_codes