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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
from puzzlebot_vision.aruco_detector.ArucoDetector import ArucoDetector
import os
import threading
import sys
import importlib.resources


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.aruco_detector = ArucoDetector()
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.loaded_saved_poses = []

        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.compressed_image_callback,
            10
        )

        self.saved_tf_timer = self.create_timer(0.01, self.broadcast_saved_tfs)

        self.file_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),  # current file directory
         "aruco_detector", "aruco_map_poses", "aruco_poses.json"
        )
        self.file_path = os.path.abspath(self.file_path)

        # Ensure the directory for the file exists
        dir_path = os.path.dirname(self.file_path)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path, exist_ok=True)

        self.camera_frame = "camera_base_link"
        self.map_frame = "map"
        self.saved_ids = set()

        self.allow_updates = True
        self.allow_new_saves = True
        threading.Thread(target=self.listen_for_keys, daemon=True).start()

    def compressed_image_callback(self, msg):
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            self.process_frame(frame, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def broadcast_saved_tfs(self):
        if len(self.loaded_saved_poses) <= 0:
            return
        now = self.get_clock().now().to_msg()
        try:
            with open(self.file_path, 'r') as f:
                poses = json.load(f)
        except Exception as e:
            self.get_logger().warn(f"Could not load JSON from {self.file_path}: {e}")
            return

        for pose in poses:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = f"aruco_{pose['id']}"

            t.transform.translation.x = pose['translation']['x']
            t.transform.translation.y = pose['translation']['y']
            t.transform.translation.z = pose['translation']['z']
            t.transform.rotation.x = pose['rotation']['x']
            t.transform.rotation.y = pose['rotation']['y']
            t.transform.rotation.z = pose['rotation']['z']
            t.transform.rotation.w = pose['rotation']['w']

            self.tf_broadcaster.sendTransform(t)

    def process_frame(self, frame, stamp: Time):
        detections = self.aruco_detector.detect(frame)

        for det in detections:
            marker_id = det['id']
            rvec = np.array(det['rvec'], dtype=np.float64)
            tvec = np.array(det['tvec'], dtype=np.float64)
            T_camera_to_aruco = self.rvec_tvec_to_matrix(rvec, tvec)
            pose_cam = self.matrix_to_pose_stamped(T_camera_to_aruco, self.camera_frame, stamp)
            try:
                tf_map_to_camera = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    rclpy.time.Time.from_msg(stamp),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                pose_map = do_transform_pose(pose_cam.pose, tf_map_to_camera)

                pose_stamped_map = PoseStamped()
                pose_stamped_map.header.frame_id = self.map_frame
                pose_stamped_map.header.stamp = stamp
                pose_stamped_map.pose = pose_map

                # self.broadcast_tf(pose_stamped_map, marker_id)

                if marker_id in self.saved_ids:
                    if self.allow_updates:
                        self.save_pose_to_json(marker_id, pose_stamped_map, self.file_path)
                        # self.get_logger().info(f"♻️ Updated pose for marker {marker_id}")
                else:
                    if self.allow_new_saves:
                        self.save_pose_to_json(marker_id, pose_stamped_map, self.file_path)
                        self.saved_ids.add(marker_id)
                        # self.get_logger().info(f"🆕 Saved new pose for marker {marker_id}")

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

        try:
            rot = R.from_matrix(T[:3, :3])  # For scipy >= 1.4.0
        except AttributeError:
            rot = R.from_dcm(T[:3, :3])     # For scipy < 1.4.0
            
        q = rot.as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

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
        self.loaded_saved_poses.append(pose_data)
        existing = []
        try:
            if os.path.exists(filepath) and os.path.getsize(filepath) > 0:
                with open(filepath, 'r') as f:
                    existing = json.load(f)
                    existing = [e for e in existing if e["id"] != marker_id]
            else:
                self.get_logger().info(f"File {filepath} not found or empty. Creating new file.")
        except Exception as e:
            self.get_logger().warn(f"Could not load JSON from {filepath}: {e}")
            existing = []

        existing.append(pose_data)
        try:
            with open(filepath, 'w') as f:
                json.dump(existing, f, indent=2)
        except Exception as e:
            self.get_logger().info(f"Error: {e}")

    def listen_for_keys(self):
        self.get_logger().info("🎮 Press 'u' to toggle updates, 'n' to toggle new saves.")
        while True:
            key = sys.stdin.read(1)
            if key == 'u':
                self.allow_updates = not self.allow_updates
                self.get_logger().info(f"🛠️ Updates {'enabled' if self.allow_updates else 'disabled'}.")
            elif key == 'n':
                self.allow_new_saves = not self.allow_new_saves
                self.get_logger().info(f"💾 New saves {'enabled' if self.allow_new_saves else 'disabled'}.")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
