import cv2
import numpy as np
import os
import sys

# === Adjustable parameter: side length of ArUco marker in meters ===
ARUCO_MARKER_LENGTH = 0.05  # e.g. 5 cm

CALIBRATION_FILE = "calibration.npz"

def main():
    # Check for calibration file
    if not os.path.exists(CALIBRATION_FILE):
        print(f"❌ Calibration file '{CALIBRATION_FILE}' not found.")
        sys.exit(1)

    print(f"✅ Found calibration file: '{CALIBRATION_FILE}'")

    # Load camera calibration data
    with np.load(CALIBRATION_FILE) as X:
        camera_matrix = X["camMatrix"]
        dist_coeffs = X["distCoeff"]

    # Load ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Start video capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Could not open camera.")
        return

    print("🎥 Starting ArUco detection with pose estimation. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame grab failed.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_MARKER_LENGTH, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], ARUCO_MARKER_LENGTH * 0.5)


                # Output pose info
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                print(f"🟢 Marker ID {ids[i][0]}:")
                print(f"    Translation: x={tvec[0]:.2f}, y={tvec[1]:.2f}, z={tvec[2]:.2f}")
                print(f"    Rotation vector: {rvec}")

        # Show result
        cv2.imshow("ArUco Pose Estimation", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
