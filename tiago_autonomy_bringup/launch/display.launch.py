import os
from sys import executable

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
def generate_launch_description():


    tiago_xacro_file = os.path.join(get_package_share_directory('tiago_autonomy_bringup'),'urdf','tiago_dual_custom.urdf')

    robot_description = Command(
        [FindExecutable(name='xacro'),' ', tiago_xacro_file]
    )

    rviz_config = os.path.join(get_package_share_directory('tiago_autonomy_bringup'),'rviz2','rviz_config.rviz')

    controller_config = os.path.join(get_package_share_directory('tiago_autonomy_bringup'),'config','tiago_controllers.yaml')

    rsp_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output= 'both',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    rviz_node = Node(
        package='rviz2',
        executable = 'rviz2',
        output= 'both',
        arguments=['-d', rviz_config],
    )

    control_node = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [
            {'robot_description': robot_description},
            controller_config
        ],
    )

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster'
            ],
            output='both'
        )

    return LaunchDescription([
        rsp_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        control_node
    ])