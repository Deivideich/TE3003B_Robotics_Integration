from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        Node(
            package='puzzlebot_vision',
            executable='camera_sim.py',
            name='camera_sim',
            parameters=[],
        ),

        # Camera compressed
        Node(
            package='puzzlebot_vision',
            executable='camera_compressed_pub.py',
            name='camera_compressed_pub',
            parameters=[],
        ),

        Node(
            package='puzzlebot_vision',
            executable='aruco_detector_node.py',
            name='aruco_detector_node',
            parameters=[],
        ),

        Node(
            package='puzzlebot_vision',
            executable='qr_detector_node.py',
            name='qr_detector_node',
            parameters=[],
        ),
    ])
