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
            controller_config,
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

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_left_controller'
        ],
        output='both'
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_right_controller'
        ],
        output='both'
    )

    torso_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'torso_controller'
        ],
        output='both'
    )

    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_gripper_controller'
        ],
        output='both'
    )

    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'right_gripper_controller'
        ],
        output='both'
    )

    return LaunchDescription([
        rsp_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
        torso_controller_spawner,
        left_gripper_controller_spawner,
        right_gripper_controller_spawner,
    ])