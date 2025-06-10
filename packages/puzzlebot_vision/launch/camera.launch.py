import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_puzzlebot_ros_name = "puzzlebot_ros"
    # Paths
    pkg_puzzlebot_ros_share = FindPackageShare(pkg_puzzlebot_ros_name).find(pkg_puzzlebot_ros_name)
    camera_launch_path = os.path.join(pkg_puzzlebot_ros_share, 'launch', 'camera_jetson.launch.py')

    return LaunchDescription([
        # Launch arguments

        # Camera Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("puzzlebot_ros"),
                    "camera_jetson.launch.py"
                ])
            ])
        ),

        # Camera compressed
        Node(
            package='puzzlebot_vision',
            executable='camera_compressed_pub.py',
            name='camera_compressed_pub',
            parameters=[],
        ),

    ])
