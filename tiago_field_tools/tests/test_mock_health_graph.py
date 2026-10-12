from collections import namedtuple
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import psutil
import yaml
import pytest

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

# Ensure package scripts and root are importable
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from health_monitor import evaluate_readiness


@pytest.fixture(scope="module")
def rclpy_session():
    """Initializes and tears down rclpy once for the test module."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def create_mock_config(
    tmp_dir: Path, required_nodes=None, required_topics=None, min_disk=1.0
):
    """Generates a minimal acceptance_tests.yaml tailored for mock evaluation."""
    cfg = {
        "monitoring": {
            "topic_stale_timeout_seconds": 2.0,
            "min_disk_free_gb": min_disk,
            "max_ram_percent": 95.0,
            "required_nodes": required_nodes
            or [
                "/robot_state_publisher",
                "/controller_manager",
            ],
            "required_topics": required_topics
            or [
                {"name": "/scan", "min_hz": 5.0},
                {"name": "/mobile_base_controller/odom", "min_hz": 10.0},
            ],
        },
        "navigation": {
            "action_server": "/navigate_to_pose",
            "timeout_seconds": 10.0,
        },
    }
    cfg_path = tmp_dir / "test_acceptance.yaml"
    with open(cfg_path, "w") as f:
        yaml.safe_dump(cfg, f)
    return str(cfg_path)


class MockRobotGraph(Node):
    """Publishes dummy sensor topics and runs dummy action servers."""

    def __init__(self):
        super().__init__("robot_state_publisher")

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(
            Odometry, "/mobile_base_controller/odom", 10
        )

        # 10 Hz timer for scan, 20 Hz timer for odom
        self.create_timer(0.1, self._publish_scan)
        self.create_timer(0.05, self._publish_odom)

        # Mock NavigateToPose Action Server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute_action_cb,
        )

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_laser_link"
        self.scan_pub.publish(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        self.odom_pub.publish(msg)

    def _execute_action_cb(self, goal_handle: ServerGoalHandle):
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result


def test_empty_graph_reports_not_ready(rclpy_session):
    """Verifies that an idle ROS 2 graph produces NOT_READY with explicit missing nodes."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        cfg_file = create_mock_config(Path(tmp_dir), min_disk=0.5)
        result = evaluate_readiness(config_path=cfg_file, sim_time=False)

        assert result["status"] == "NOT_READY"
        assert any(
            "Missing node: /robot_state_publisher" in err
            for err in result["failed_checks"]
        )
        assert any(
            "Topic not published: /scan" in err for err in result["failed_checks"]
        )


def test_mock_graph_healthy(rclpy_session, monkeypatch):
    """Spins a mock graph and verifies evaluate_readiness reports READY with mocked disk headroom."""
    # Mock disk_usage so low-disk laptop conditions never fail unit tests
    DiskUsage = namedtuple("DiskUsage", ["total", "used", "free", "percent"])
    monkeypatch.setattr(
        psutil,
        "disk_usage",
        lambda path: DiskUsage(100 * (1024**3), 20 * (1024**3), 80 * (1024**3), 20.0),
    )

    mock_robot = MockRobotGraph()
    mock_controller_mgr = Node("controller_manager")

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_robot)
    executor.add_node(mock_controller_mgr)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # DDS discovery stabilization window
        time.sleep(1.0)

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg_file = create_mock_config(
                Path(tmp_dir),
                required_nodes=["/robot_state_publisher", "/controller_manager"],
                required_topics=[
                    {"name": "/scan", "min_hz": 5.0},
                    {"name": "/mobile_base_controller/odom", "min_hz": 10.0},
                ],
                min_disk=1.0,
            )
            result = evaluate_readiness(config_path=cfg_file, sim_time=False)

            assert result["status"] == "READY", f"Expected READY, got {result}"
            assert len(result["failed_checks"]) == 0
            assert result["metrics"]["topics"]["/scan"]["status"] == "HEALTHY"
            assert (
                result["metrics"]["topics"]["/mobile_base_controller/odom"]["status"]
                == "HEALTHY"
            )
            assert result["metrics"]["nav_action_ready"] is True

    finally:
        executor.shutdown()
        mock_robot.destroy_node()
        mock_controller_mgr.destroy_node()
        spin_thread.join(timeout=1.0)


def test_mock_graph_degraded_hz(rclpy_session, monkeypatch):
    """Verifies that under-publishing sensor topics triggers DEGRADED status."""
    DiskUsage = namedtuple("DiskUsage", ["total", "used", "free", "percent"])
    monkeypatch.setattr(
        psutil,
        "disk_usage",
        lambda path: DiskUsage(100 * (1024**3), 20 * (1024**3), 80 * (1024**3), 20.0),
    )

    mock_robot = Node("robot_state_publisher")
    mock_controller = Node("controller_manager")
    scan_pub = mock_robot.create_publisher(LaserScan, "/scan", 10)

    # Publish at ~2 Hz when 10 Hz is expected
    mock_robot.create_timer(0.5, lambda: scan_pub.publish(LaserScan()))

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_robot)
    executor.add_node(mock_controller)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        time.sleep(0.5)

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg_file = create_mock_config(
                Path(tmp_dir),
                required_nodes=["/robot_state_publisher", "/controller_manager"],
                required_topics=[{"name": "/scan", "min_hz": 10.0}],
                min_disk=1.0,
            )
            result = evaluate_readiness(config_path=cfg_file, sim_time=False)

            assert result["status"] in ["DEGRADED", "NOT_READY"]
            assert any(
                "Topic degraded: /scan" in warn for warn in result["degraded_checks"]
            )

    finally:
        executor.shutdown()
        mock_robot.destroy_node()
        mock_controller.destroy_node()
        spin_thread.join(timeout=1.0)
