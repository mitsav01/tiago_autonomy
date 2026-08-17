import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    default_ekf_config_file = os.path.join(
        get_package_share_directory("tiago_navigation"),
        "config",
        "ekf.yaml",
    )

    ekf_config_file = LaunchConfiguration('ekf_config_file')

    declare_ekf_config_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_file,
        description='Full path to ekf config file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (TurtleBot) clock if true'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, 
                    {'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_ekf_config_cmd,
        declare_use_sim_time_cmd,
        ekf_node
    ])