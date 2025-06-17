import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # Find package and file paths
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="puzzlebot_description"
    ).find("puzzlebot_description")
    default_model_path = os.path.join(
        pkg_share, "urdf", "robot.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "visualizer.rviz")

    # Declare launch arguments
    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to the robot URDF file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to the RVIZ config file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="use_gui",
            default_value="false",  # Default to false
            description="Flag to enable/disable the joint_state_publisher_gui",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="prefix",
            default_value="",
            description="Prefix for robot link/joint names",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="use_gazebo_controllers",
            default_value="false",
            description="Whether to include Gazebo controllers",
        )
    )

    # Command to process the xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # Nodes
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    # Conditionally launch joint_state_publisher_gui
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_gui")),
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # Add all nodes to the launch description
    nodes = [
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node,  # Only launches if 'use_gui' is true
    ]

    return launch.LaunchDescription(args + nodes)