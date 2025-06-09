import os
from math import pi
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # pkg_urdf_name = "puzzlebot_description"
    # pkg_kinematics_name = "puzzlebot_kinematics"
    # pkg_navigation_name = "puzzlebot_navigation"
    # # Paths
    # pkg__kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    # pkg_nav_share = FindPackageShare(pkg_navigation_name).find(pkg_navigation_name)

    # pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    # default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    # default_rviz_config_path = os.path.join(pkg_nav_share, "rviz", "mcl.rviz")

    # # Add the path for the Gazebo model (adjust based on where the saved model files are)
    # gazebo_model_path = os.path.join(pkg_urdf_share, "models", "mcl_world")

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            name="controller_type", default_value="pure_pursuit",
            description="Type of controller wanted: pure_pursuit, PID, MCP"
        ),
        DeclareLaunchArgument(
            name="linear_speed", default_value="0.1",
            description="Desired linear speed for controller"
        ),
        DeclareLaunchArgument(
            name="angular_speed", default_value="0.3",
            description="Desired linear speed for controller"
        ),
        DeclareLaunchArgument(
            name="lookahead_distance", default_value="0.2",
            description="Lookahead distance used in PurePursuit controller"
        ),
        DeclareLaunchArgument(
            name="orientation_tolerance", default_value="0.15",
            description="Lookahead distance used in PurePursuit controller"
        ),
        DeclareLaunchArgument(
            name="kP", default_value="0.2",
            description="Proportional gain used in PID controller"
        ),
        DeclareLaunchArgument(
            name="kI", default_value="0.2",
            description="Integral gain used in PID controller"
        ),
        DeclareLaunchArgument(
            name="kD", default_value="0.2",
            description="Derivative gain used in PID controller"
        ),
        DeclareLaunchArgument(
            name="usingBugAlgorithm", default_value="false",
            description="Derivative gain used in PID controller"
        ),
        DeclareLaunchArgument(
            name="theta_resolution", default_value=str(pi/8),
            description="Used to bin theta into the hash map"
        ),
        DeclareLaunchArgument(
            name="translational_weight", default_value="0.1",
            description="Scale used for translational distance in A*"
        ),
        DeclareLaunchArgument(
            name="rotational_weight", default_value="0.9",
            description="Scale used for theta distance in A*"
        ),
        DeclareLaunchArgument(
            name="interpolation_steps", default_value="50",
            description="Amount of interpolation between SE2States"
        ),
        DeclareLaunchArgument(
            name="using_real_sampling", default_value="false",
            description="Using real sampling on SE2States or the grid map for A* algorithm"
        ),
        DeclareLaunchArgument(
            name="robot_width", default_value="0.2",
            description="Width used for basefootprint"
        ),
        DeclareLaunchArgument(
            name="robot_height", default_value="0.2",
            description="Height used for basefootprint"
        ),
        
    
        # State publisher
        Node(
            package="puzzlebot_planning",
            executable="astar_planner_server",
            name="astar_planner_server",
            parameters=[{
                "theta_resolution" : LaunchConfiguration("theta_resolution"),
                "translational_weight" : LaunchConfiguration("translational_weight"),
                "rotational_weight" : LaunchConfiguration("rotational_weight"),
                "interpolation_steps" : LaunchConfiguration("interpolation_steps"),
                "using_real_sampling" : LaunchConfiguration("using_real_sampling"),
                "robot_width" : LaunchConfiguration("robot_width"),
                "robot_height" : LaunchConfiguration("robot_height"),
            }],
            output="screen"
        ),

        Node(
            package="puzzlebot_controller",
            executable="controller_node",
            name="controller_node",
            parameters=[{
                "controller_type" : LaunchConfiguration("controller_type"),
                "linear_speed" : LaunchConfiguration("linear_speed"),
                "angular_speed" : LaunchConfiguration("angular_speed"),
                "lookahead_distance" : LaunchConfiguration("lookahead_distance"),
                "orientation_tolerance" : LaunchConfiguration("orientation_tolerance"),
                "kP" : LaunchConfiguration("kP"),
                "kI" : LaunchConfiguration("kI"),
                "kD" : LaunchConfiguration("kD"),
                "usingBugAlgorithm" : LaunchConfiguration("usingBugAlgorithm")
            }],
            output="screen"
        )
    ])
