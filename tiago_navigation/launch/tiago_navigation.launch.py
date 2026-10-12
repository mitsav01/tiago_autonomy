import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    tiago_nav_pkg = get_package_share_directory("tiago_navigation")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    # Paths
    ekf_params_file = os.path.join(tiago_nav_pkg, "config", "ekf.yaml")
    map_yaml_file = os.path.join(tiago_nav_pkg, "maps", "apartment_map.yaml")
    nav2_params_file = os.path.join(tiago_nav_pkg, "config", "nav2_tiago_params.yaml")
    apriltag_dock_params_file = os.path.join(
        tiago_nav_pkg, "config", "apriltag_dock.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    set_env_var = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )

    # 1. Robot Localization (EKF)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2. Nav2 Bringup (AMCL, Planner, Controller, BT Navigator)
    nav2_launch_file = os.path.join(nav2_pkg, "launch", "bringup_launch.py")
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "slam": "False",
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": "True",
            "use_respawn": "True",
            "use_composition": "False",
        }.items(),
    )

    # 3. AprilTag Detection Node
    apriltag_dock_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_dock_node",
        output="screen",
        parameters=[
            apriltag_dock_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("image_rect", "/head_front_camera/image"),
            ("image_raw", "/head_front_camera/image"),
            ("image", "/head_front_camera/image"),
            ("camera_info", "/head_front_camera/camera_info"),
        ],
    )

    # 4. Visual Dock Pose Publisher
    dock_pose_publisher_node = Node(
        package="tiago_navigation",
        executable="dock_pose_publisher.py",
        name="dock_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "fixed_frame": "map",
                "tag_frame": "tag36h11:0",
                "dock_center_x": 8.17,
                "tag_to_dock_y_offset": -1.38,
            }
        ],
        remappings=[
            ("detected_dock_pose", "/detected_dock_pose"),
        ],
    )

    # 5. OpenNav Docking Server
    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        parameters=[
            ParameterFile(nav2_params_file, allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("detected_dock_pose", "/detected_dock_pose"),
            ("cmd_vel", "/dock_cmd_vel"),
        ],
    )

    # 6. Static TF for Dock reference frame in map
    dock_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="dock_static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "8.17",
            "--y", "4.85",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "1.5708",
            "--frame-id", "map",
            "--child-frame-id", "dock_tiago",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 7. Velocity Relay: Twist (/dock_cmd_vel) -> TwistStamped (/cmd_vel_smoothed)
    twist_relay_node = Node(
        package="tiago_navigation",
        executable="twist_to_twiststamped.py",
        name="twist_to_twiststamped_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/dock_cmd_vel",
                "output_topic": "/cmd_vel_smoothed",
                "frame_id": "base_footprint",
            }
        ],
    )

    return LaunchDescription(
        [
            set_env_var,
            declare_use_sim_time,
            ekf_node,
            nav2_bringup,
            apriltag_dock_node,
            dock_pose_publisher_node,
            docking_server_node,
            dock_static_tf,
            twist_relay_node,
        ]
    )