import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

port_micro = LaunchConfiguration('port_micro')
port_lidar = LaunchConfiguration('port_lidar')

def generate_launch_description():
    pkg_lidar_name = "rplidar_ros"
    pkg_kinematics_name = "puzzlebot_kinematics"
    # Paths
    pkg_kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_lidar_share = FindPackageShare(pkg_lidar_name).find(pkg_lidar_name)
    lidar_launch_path = os.path.join(pkg_lidar_share, 'launch', 'rplidar_a1_launch.py')
    real_kinematics_launch_path = os.path.join(pkg_kinematics_share, 'launch', 'puzzlebot_kinematic_real.launch.py')

    return LaunchDescription([
        # Launch arguments
        
        DeclareLaunchArgument(
            name="port_micro", default_value="/dev/stm32",
            description="Which USB port to use for Micro-ROS agent connection with Hackerboard"
        ),

        DeclareLaunchArgument(
            name="port_lidar", default_value="/dev/rplidar",
            description="Which USB port to use for Lidar connection"
        ),

        # Lidar Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={
                'serial_port': port_lidar
            }.items()
        ),

        # Micro ROS
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "-D", port_micro],
            parameters=[],
            output='screen'
        ),

        #Camera
        
        # Puzzlebot kinematics launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_kinematics_launch_path),
            launch_arguments={
                'rviz': 'false'
            }.items()
        ),
        
    ])
