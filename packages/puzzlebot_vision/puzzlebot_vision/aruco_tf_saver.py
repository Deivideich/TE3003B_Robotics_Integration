#!/usr/bin/env python3

import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
import numpy as np
import json
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time

from puzzlebot_vision.aruco_detector.ArucoDetector import ArucoDetector

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        print("DEBUG")
        self.aruco_detector = ArucoDetector()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.compressed_image_callback,
            10
        )

        self.camera_frame = "camera_link"  # Change this to your actual camera frame name
        self.map_frame = "map"
        self.saved_ids = set()

    def compressed_image_callback(self, msg):
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            self.process_frame(frame, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def process_frame(self, frame, stamp: Time):
        detections = self.aruco_detector.detect(frame)

        for det in detections:
            marker_id = det['id']
            rvec = np.array(det['rvec'], dtype=np.float64)
            tvec = np.array(det['tvec'], dtype=np.float64)

            T_camera_to_aruco = self.rvec_tvec_to_matrix(rvec, tvec)
            pose_cam = self.matrix_to_pose_stamped(T_camera_to_aruco, self.camera_frame, stamp)

            try:
                # Lookup transform from map → camera_frame
                tf_map_to_camera = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rclpy.time.Time.from_msg(stamp),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                # Transform pose to map frame
                pose_map = do_transform_pose(pose_cam, tf_map_to_camera)

                self.broadcast_tf(pose_map, marker_id)
                if marker_id not in self.saved_ids:
                    self.save_pose_to_json(marker_id, pose_map)
                    self.saved_ids.add(marker_id)

            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

    def rvec_tvec_to_matrix(self, rvec, tvec):
        R_mat, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = tvec.flatten()
        return T

    def matrix_to_pose_stamped(self, T, frame_id, stamp):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp

        pose.pose.position.x = T[0, 3]
        pose.pose.position.y = T[1, 3]
        pose.pose.position.z = T[2, 3]

        rot = R.from_matrix(T[:3, :3])
        q = rot.as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def broadcast_tf(self, pose_stamped: PoseStamped, marker_id: int):
        t = TransformStamped()
        t.header = pose_stamped.header
        t.child_frame_id = f"aruco_{marker_id}"

        t.transform.translation = pose_stamped.pose.position
        t.transform.rotation = pose_stamped.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def save_pose_to_json(self, marker_id: int, pose_stamped: PoseStamped, filepath="aruco_poses.json"):
        pose_data = {
            "id": marker_id,
            "translation": {
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "z": pose_stamped.pose.position.z
            },
            "rotation": {
                "x": pose_stamped.pose.orientation.x,
                "y": pose_stamped.pose.orientation.y,
                "z": pose_stamped.pose.orientation.z,
                "w": pose_stamped.pose.orientation.w
            }
        }

        existing = []
        try:
            with open(filepath, 'r') as f:
                existing = json.load(f)
                existing = [e for e in existing if e["id"] != marker_id]
        except FileNotFoundError:
            pass

        existing.append(pose_data)
        with open(filepath, 'w') as f:
            json.dump(existing, f, indent=2)

        self.get_logger().info(f"✅ Saved ArUco marker {marker_id} pose to {filepath}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
