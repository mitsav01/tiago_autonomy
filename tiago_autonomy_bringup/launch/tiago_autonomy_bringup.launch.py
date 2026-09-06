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

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=":".join(gz_paths)
    )

    world_file = os.path.join(tiago_bringup_pkg, "urdf", "apartment_gazebo.sdf")
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
            "-name",
            "tiago_dual",
            "-string",
            robot_description,
            "-x",
            "5.0",
            "-y",
            "2.0",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    # gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    #         "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    #         "/base_imu@sensor_msgs/msg/Imu[gz.msgs.Imu",
    #         "/head_front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/head_front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    #         "/head_front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/head_front_camera/depth_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    #     ],
    #     parameters=[{"use_sim_time": True}],
    #     output="screen",
    # )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",

            # Capitalization is important
            "/base_imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

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

    # 2. Base & Body Trajectory Controllers
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

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=4.0,
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
            gz_sim_launch,
            robot_state_publisher,
            spawn_robot,
            gz_bridge,
            delay_joint_state_broadcaster,
            delay_motion_controllers,
            rviz_node,
        ]
    )