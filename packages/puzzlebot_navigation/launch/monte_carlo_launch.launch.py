import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

mcl_args = {
    'useClustering': False,
    'numParticles': 1000,
    'minClusterDistance': 0.5,
    'clusterEps': 0.5,
    'clusterMinSamples': 0.05,
    'scaleRdParticles': 0.0,
    'minDistance': 0.01,
    'minAngle': 10.0,
    'repropagateCountNeeded': 1,
    'HZ' : 20.0,
}

def generate_launch_description():
    pkg_urdf_name = "puzzlebot_description"
    pkg_kinematics_name = "puzzlebot_kinematics"
    pkg_navigation_name = "puzzlebot_navigation"
    # Paths
    pkg__kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_nav_share = FindPackageShare(pkg_navigation_name).find(pkg_navigation_name)

    pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    default_rviz_config_path = os.path.join(pkg_nav_share, "rviz", "mcl.rviz")

    # Add the path for the Gazebo model (adjust based on where the saved model files are)
    gazebo_model_path = os.path.join(pkg_urdf_share, "models", "mcl_world")

    return LaunchDescription([
        # Launch arguments
        *[
            DeclareLaunchArgument(
                name=key,
                default_value=str(value) if isinstance(value, (int, float)) else ("true" if value else "false"),
                description=f"Parameter {key} for monte_carlo_localisation node"
            )
            for key, value in mcl_args.items()
        ],
        DeclareLaunchArgument(
            name="model", default_value=default_model_path,
            description="Absolute path to robot urdf.xacro file"
        ),
        DeclareLaunchArgument(
            name="rviz", default_value="false",
            description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            name="prefix", default_value="",
            description="Prefix for robot link/joint names"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_controllers", default_value="false",
            description="Whether to include Gazebo controllers"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_odom", default_value="false",
            description="Whether to include Gazebo odometry"
        ),
        DeclareLaunchArgument(
            name="use_mcl_clustering", default_value="false",
            description="Whether to use clustering in MCL algorithm"
        ),


        # Optional RViz launch
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path]
        ),
        
        Node(
            package="puzzlebot_navigation",
            executable="custom_map_server.py",
            name="custom_map_server",
            output="screen",
        ),
        
        Node(
            package="puzzlebot_navigation",
            executable="monte_carlo_localisation.py",
            name="monte_carlo_localisation",
            output="screen",
            parameters=[
                {key: LaunchConfiguration(key) for key in mcl_args.keys()}
            ],
        ),

        Node(
            package="puzzlebot_navigation",
            executable="local_map.py",
            name="local_map",
            output="screen",
            parameters=[
            ],
        ),
        
    ])
