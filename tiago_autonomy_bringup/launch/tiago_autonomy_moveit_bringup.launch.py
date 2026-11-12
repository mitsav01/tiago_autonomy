import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
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
            models_dir = os.path.join(path, "models")
            if os.path.exists(models_dir):
                gz_paths.append(models_dir)
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
            "1.9",
            "-y",
            "-0.4",
            "-z",
            "0.05",
            "-Y",
            "-1.57"
        ],
        output="screen",
    )

    delay_spawn_robot = TimerAction(
        period=5.0,
        actions=[spawn_robot],
    )

    # Bridge sensor streams, clock, odometry, and mobile base velocity
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/base_imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/head_front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/head_front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/head_front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/head_front_camera/depth_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Static transform publisher to resolve MoveIt's virtual_joint (odom -> base_footprint)
    static_tf_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_odom_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
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

    # Motion controller spawners aligned with hardware and MoveIt mapping
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

    # Bridge node to convert MoveIt virtual joint trajectories to /cmd_vel
    virtual_joint_to_cmd_vel_node = Node(
        package="tiago_autonomy_bringup",
        executable="base_trajectory_action_server.py",
        name="virtual_joint_to_cmd_vel",
        parameters=[{"use_sim_time": True}],
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

    # Launch bridge node only after motion controllers are active
    delay_virtual_joint_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_motion_controllers,
            on_exit=[virtual_joint_to_cmd_vel_node],
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
            static_tf_odom,
            delay_joint_state_broadcaster,
            delay_motion_controllers,
            delay_virtual_joint_node,
            # rviz_node,
        ]
    )