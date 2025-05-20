from puzzlebot_interfaces.msg import QRCode
from typing import List
import cv2
import numpy as np
from numpy.typing import NDArray

class QRDetector:
    def __init__(self):
        self.qr_detector = cv2.QRCodeDetector()

    def detect(self, frame: NDArray) -> List[QRCode]:
        detected_codes = []
        height, width = frame.shape[:2]
        
        # Detect QR codes in the frame
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
        
        if retval:
            for content, coords in zip(decoded_info, points):
                if content:  # Only process if content was successfully decoded
                    # Get normalized coordinates
                    x_coords = coords[:, 0] / width
                    y_coords = coords[:, 1] / height
                    
                    qr = QRCode()
                    qr.x1 = float(min(x_coords))
                    qr.x2 = float(max(x_coords))
                    qr.y1 = float(min(y_coords))
                    qr.y2 = float(max(y_coords))
                    qr.content = content
                    
                    detected_codes.append(qr)
                    
        return detected_codes