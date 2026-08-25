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

    nav2_launch_file_dir = os.path.join(
        nav2_pkg,
        "launch", "bringup_launch.py"
    )

    nav2_params_file = os.path.join(
        tiago_nav_pkg,
        "config",
        "nav2_tiago_params.yaml",
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file,
                    {'use_sim_time': True}
                    ]
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file_dir),
        launch_arguments={
            'slam': 'True',
            'map': map_yaml_file,
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_respawn': 'True',
            'use_composition': 'False',
        }.items(),
        # remappings=[
        #     ('/cmd_vel','/mobile_base_controller/cmd_vel')
        # ]
    )

    return LaunchDescription([
        ekf_node,
        nav2_node,
    ])