import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("tiago_whole_body_controller")
    params_file = os.path.join(pkg_share, "config", "wbc_params.yaml")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            get_package_share_directory("tiago_autonomy_bringup"),
            "urdf",
            "tiago_autonomy.urdf",
        ]),
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    wbc_node = Node(
        package="tiago_whole_body_controller",
        executable="tiago_whole_body_controller_node",
        name="tiago_dual_wbc_controller",
        output="screen",
        parameters=[
            params_file,
            {"robot_description": robot_description},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        wbc_node,
    ])