import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_share = get_package_share_directory("tiago_moveit_config")
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder("tiago_dual", package_name="tiago_moveit_config")
        .to_moveit_configs()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz_node])