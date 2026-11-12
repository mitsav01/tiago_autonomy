import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():

    tiago_bringup_pkg = get_package_share_directory("tiago_autonomy_bringup")

    required_packages = [
        "tiago_autonomy_bringup",
        "iai_apartment",
        "pmb2_description",
        "tiago_description",
        "tiago_dual_description",
        "pal_robotiq_description",
        "pal_urdf_utils",
        "iai_refills_lab",
    ]

    gz_paths = []
    for pkg in required_packages:
        try:
            path = get_package_share_directory(pkg)
            gz_paths.append(path)
            gz_paths.append(os.path.dirname(path))
        except Exception:
            pass

    old_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if old_path:
        gz_paths.append(old_path)

    resource_path_str = ":".join(dict.fromkeys(gz_paths))

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=resource_path_str
    )
    gz_file_path = SetEnvironmentVariable(
        name="GZ_FILE_PATH", value=resource_path_str
    )

    world_file = os.path.join(tiago_bringup_pkg, "urdf", "dm_room.sdf")
    robot_file = os.path.join(tiago_bringup_pkg, "urdf", "tiago_autonomy.urdf")
    rviz_config = os.path.join(tiago_bringup_pkg, "rviz2", "rviz_config.rviz")

    robot_description = Command([FindExecutable(name="xacro"), " ", robot_file])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Spawn centered on Shelf 2 (x = 1.75) in the open aisle (y = 0.05, ~1.23 m from shelf front)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "retail_world",
            "-name",
            "tiago_dual",
            "-string",
            robot_description,
            "-x",
            "1.75",
            "-y",
            "0.05",
            "-z",
            "0.05",
            "-Y",
            "-1.5708",
        ],
        output="screen",
    )

    delay_spawn_robot = TimerAction(
        period=5.0,
        actions=[spawn_robot],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/base_imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/head_front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/head_front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/head_front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/head_front_camera/depth_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    spawn_motion_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobile_base_controller",
            "torso_controller",
            "head_controller",
            "arm_left_controller",
            "arm_right_controller",
            "left_robotiq_gripper_controller",
            "right_robotiq_gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Automatically move torso & arms into an in-front-of-chest ready posture right after controllers load
    ready_pose_torso = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/torso_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: ['torso_lift_joint'], points: [{positions: [0.28], time_from_start: {sec: 2, nanosec: 0}}]}",
        ],
        output="screen",
    )

    ready_pose_left = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/arm_left_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: ['arm_left_1_joint','arm_left_2_joint','arm_left_3_joint','arm_left_4_joint','arm_left_5_joint','arm_left_6_joint','arm_left_7_joint'], points: [{positions: [0.35, -0.55, -0.35, 1.65, -1.57, 0.15, 0.0], time_from_start: {sec: 2, nanosec: 500000000}}]}",
        ],
        output="screen",
    )

    ready_pose_right = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/arm_right_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            "{joint_names: ['arm_right_1_joint','arm_right_2_joint','arm_right_3_joint','arm_right_4_joint','arm_right_5_joint','arm_right_6_joint','arm_right_7_joint'], points: [{positions: [-0.35, -0.55, 0.35, 1.65, 1.57, 0.15, 0.0], time_from_start: {sec: 2, nanosec: 500000000}}]}",
        ],
        output="screen",
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_motion_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[spawn_motion_controllers],
        )
    )

    delay_ready_pose = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_motion_controllers,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[ready_pose_torso, ready_pose_left, ready_pose_right],
                )
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gz_resource_path,
            gz_file_path,
            gz_sim_launch,
            robot_state_publisher,
            delay_spawn_robot,
            gz_bridge,
            delay_joint_state_broadcaster,
            delay_motion_controllers,
            delay_ready_pose,
            # rviz_node,
        ]
    )