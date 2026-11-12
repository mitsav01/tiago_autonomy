import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("tiago_dual", package_name="tiago_moveit_config")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "trajectory_execution.control_multi_dof_joint_variables": True,
                "default_workspace_bounds": 10.0,
            },
        ],
    )

    return LaunchDescription([move_group_node])