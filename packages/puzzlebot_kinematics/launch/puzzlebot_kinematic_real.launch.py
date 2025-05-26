import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
import launch_ros.actions
import os


def generate_launch_description():
    # Find package and file paths
    pkg_urdf_share = launch_ros.substitutions.FindPackageShare(
        package="puzzlebot_description"
    ).find("puzzlebot_description")
    pkg_kinematics_share = launch_ros.substitutions.FindPackageShare(
        package="puzzlebot_kinematics"
    ).find("puzzlebot_kinematics")

    default_model_path = os.path.join(
        pkg_urdf_share, "urdf", "robot.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_kinematics_share, "rviz", "visualizer.rviz")

    # Declare launch arguments
    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rviz", default_value="false",
            description="Launch RViz?"
        )
    )
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

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    
    differential_dk_node = launch_ros.actions.Node(
        package="puzzlebot_kinematics",
        executable="differential_direct_kinematics_real.py",
        name="differential_direct_kinematics_real",
        output="screen",
    )

    wheel_tf_broadcaster_node = launch_ros.actions.Node(
        package="puzzlebot_kinematics",
        executable="wheel_transform_broadcaster_real.py",
        name="wheel_transform_broadcaster_real",
        output="screen",
    )


    # Add all nodes to the launch description
    nodes = [
        robot_state_publisher_node,
        rviz_node,
        differential_dk_node,
        wheel_tf_broadcaster_node,
    ]

    return launch.LaunchDescription(args + nodes)