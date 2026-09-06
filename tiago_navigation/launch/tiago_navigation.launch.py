import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    tiago_nav_pkg = get_package_share_directory("tiago_navigation")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    ekf_params_file = os.path.join(
        tiago_nav_pkg,
        "config",
        "ekf.yaml",
    )

    map_yaml_file = os.path.join(
        tiago_nav_pkg,
        "maps",
        "apartment_map.yaml",
    )

    apriltag_dock_params_file = os.path.join(
        tiago_nav_pkg,
        "config",
        "apriltag_dock.yaml",
    )

    nav2_launch_file_dir = os.path.join(nav2_pkg, "launch", "bringup_launch.py")

    nav2_params_file = os.path.join(
        tiago_nav_pkg,
        "config",
        "nav2_tiago_params.yaml",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": True}],
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file_dir),
        launch_arguments={
            "slam": "False",
            "map": map_yaml_file,
            "use_sim_time": "True",
            "params_file": nav2_params_file,
            "autostart": "True",
            "use_respawn": "True",
            "use_composition": "False",
        }.items(),
    )

    apriltag_dock_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_dock_node",
        output="screen",
        parameters=[apriltag_dock_params_file, {"use_sim_time": True}],
        remappings=[
            ("image_rect", "/head_front_camera/image"),
            ("camera_info", "/head_front_camera/camera_info"),
        ],
    )

    dock_pose_publisher_node = Node(
        package="tiago_navigation",
        executable="dock_pose_publisher.py",
        name="dock_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            ekf_node,
            nav2_node,
            apriltag_dock_node,
            dock_pose_publisher_node,
        ]
    )
