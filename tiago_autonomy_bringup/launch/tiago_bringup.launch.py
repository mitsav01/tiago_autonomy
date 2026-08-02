import os
from sys import executable

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    tiago_description_file = os.path.join(
        get_package_share_directory("tiago_autonomy_bringup"),
        "urdf",
        "tiago_autonomy.urdf",
    )

    robot_description = Command(
        [FindExecutable(name="xacro"), " ", tiago_description_file]
    )

    rviz_config = os.path.join(
        get_package_share_directory("tiago_autonomy_bringup"),
        "rviz2",
        "rviz_config.rviz",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)}
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="both",
    )

    return LaunchDescription(
        [
            rsp_node,
            rviz_node,
            joint_state_publisher_node,
        ]
    )
