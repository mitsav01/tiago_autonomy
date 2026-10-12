import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tiago_rtt_controllers')
    ops_file = os.path.join(pkg_share, 'config', 'tiago_control.ops')

    load_point_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_right_point_controller',
            'arm_left_point_controller',
            '-c', '/controller_manager',
            '--inactive',
        ],
        output='screen',
    )

    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '-c', '/controller_manager',
            '--deactivate', 'arm_right_controller', 'arm_left_controller',
            '--activate', 'arm_right_point_controller', 'arm_left_point_controller',
            '--strict',
        ],
        output='screen',
    )

    deployer_proc = ExecuteProcess(
        cmd=['taskset', '-c', '2,3', 'deployer-gnulinux', '-s', ops_file],
        output='screen',
        emulate_tty=True,
    )

    switch_on_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=load_point_controllers,
            on_exit=[switch_controllers],
        )
    )

    deployer_on_switched = RegisterEventHandler(
        OnProcessExit(
            target_action=switch_controllers,
            on_exit=[deployer_proc],
        )
    )

    return LaunchDescription([
        load_point_controllers,
        switch_on_loaded,
        deployer_on_switched,
    ])