import os
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def get_rtt_component_path():
    """Dynamically scan all active workspace install trees from AMENT_PREFIX_PATH."""
    discovered = []
    # ROS 2 tracks all sourced workspace roots here
    prefix_paths = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)

    for prefix in prefix_paths:
        if not prefix:
            continue
        # Check standard <prefix>/lib/orocos
        direct = os.path.join(prefix, "lib", "orocos")
        if os.path.isdir(direct):
            discovered.append(direct)
        
        # Check isolated colcon installs: <workspace>/install/<pkg>/lib/orocos
        parent = os.path.dirname(prefix)
        for path in glob.glob(os.path.join(parent, "*", "lib", "orocos")):
            if os.path.isdir(path):
                discovered.append(path)

    # Keep any existing RTT_COMPONENT_PATH
    existing = os.environ.get("RTT_COMPONENT_PATH", "")
    if existing:
        discovered.extend([p for p in existing.split(os.pathsep) if p])

    return os.pathsep.join(list(dict.fromkeys(discovered)))


def generate_launch_description():
    pkg_share = get_package_share_directory("tiago_rtt_controllers")
    ops_file = os.path.join(pkg_share, "config", "tiago_control.ops")

    rtt_paths = get_rtt_component_path()

    load_point_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_right_point_controller",
            "arm_left_point_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    # Use --activate directly; avoiding --strict prevents launch failure if arm controllers don't match
    switch_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "-c",
            "/controller_manager",
            "--deactivate",
            "arm_right_controller",
            "arm_left_controller",
            "--activate",
            "arm_right_point_controller",
            "arm_left_point_controller",
            "--strict",
        ],
        output="screen",
    )

    # Pass the dynamically discovered RTT paths into the deployer environment
    deployer_proc = ExecuteProcess(
        cmd=["taskset", "-c", "2,3", "deployer-gnulinux", "-s", ops_file],
        additional_env={
            "RTT_COMPONENT_PATH": rtt_paths
        },
        output="screen",
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

    return LaunchDescription(
        [
            load_point_controllers,
            switch_on_loaded,
            deployer_on_switched,
        ]
    )