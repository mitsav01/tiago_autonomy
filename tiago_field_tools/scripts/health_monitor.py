#!/usr/bin/env python3
"""
TIAGo Field Tools - Health Monitor
Monitors host resources, critical ROS 2 nodes, sensor topic frequencies,
message freshness (sim & wall clock), and Nav2 action server readiness.
"""

import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import time
import psutil
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from nav2_msgs.action import NavigateToPose


def get_default_config_path() -> str:
    """Dynamically resolves acceptance_tests.yaml across environments."""
    env_cfg = os.environ.get("TIAGO_FIELD_CONFIG")
    if env_cfg and os.path.isfile(env_cfg):
        return os.path.abspath(env_cfg)

    try:
        pkg_share = get_package_share_directory("tiago_field_tools")
        installed_path = Path(pkg_share) / "config" / "acceptance_tests.yaml"
        if installed_path.is_file():
            return str(installed_path)
    except PackageNotFoundError:
        pass

    # Source tree fallback:
    src_relative_path = (
        Path(__file__).resolve().parent.parent / "config" / "acceptance_tests.yaml"
    )
    if src_relative_path.is_file():
        return str(src_relative_path)

    raise FileNotFoundError(
        "Could not locate 'acceptance_tests.yaml'. Ensure tiago_field_tools is built "
        "or define the TIAGO_FIELD_CONFIG environment variable."
    )


class HealthMonitor(Node):
    def __init__(self, config_path: str, sim_time_override: bool = None):
        super().__init__("health_monitor")

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config path does not exist: {config_path}")

        with open(config_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        # Configure simulation time safely without re-declaring built-in parameter in Jazzy
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        if sim_time_override is not None:
            self.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, sim_time_override)]
            )
        else:
            # Auto-detect Gazebo clock if not explicitly configured via ROS args
            active_topics = [t[0] for t in self.get_topic_names_and_types()]
            if "/clock" in active_topics:
                self.set_parameters(
                    [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
                )

        self.stale_timeout = float(
            self.cfg.get("monitoring", {}).get("topic_stale_timeout_seconds", 2.0)
        )
        self.failed_checks = []
        self.degraded_checks = []
        self.metrics_snapshot = {}

    def check_system_resources(self):
        cpu = psutil.cpu_percent(interval=0.2)
        mem = psutil.virtual_memory().percent
        disk = psutil.disk_usage("/").free / (1024**3)

        self.metrics_snapshot["cpu_percent"] = cpu
        self.metrics_snapshot["ram_percent"] = mem
        self.metrics_snapshot["disk_free_gb"] = round(disk, 2)

        monitoring_cfg = self.cfg.get("monitoring", {})
        min_disk_gb = float(monitoring_cfg.get("min_disk_free_gb", 1.0))
        max_ram_pct = float(monitoring_cfg.get("max_ram_percent", 95.0))
        warn_ram_pct = float(monitoring_cfg.get("warn_ram_percent", 85.0))
        max_cpu_pct = float(monitoring_cfg.get("max_cpu_percent", 95.0))

        if mem > max_ram_pct:
            self.failed_checks.append(
                f"RAM critical: {mem:.1f}% used (>{max_ram_pct}% limit)"
            )
        elif mem > warn_ram_pct:
            self.degraded_checks.append(f"RAM elevated: {mem:.1f}% used")

        if cpu > max_cpu_pct:
            self.degraded_checks.append(f"CPU load critical: {cpu:.1f}%")

        if disk < min_disk_gb:
            self.failed_checks.append(
                f"Disk space dangerously low: {disk:.1f} GB free (<{min_disk_gb} GB)"
            )

    def check_nodes(self):
        required = self.cfg.get("monitoring", {}).get("required_nodes", [])
        if not required:
            return

        # Multi-sample window to guard against DDS discovery dropouts
        discovered_nodes = set()
        for _ in range(3):
            for n in self.get_node_names():
                discovered_nodes.add(n.lstrip("/"))
            rclpy.spin_once(self, timeout_sec=0.1)

        self.metrics_snapshot["active_nodes_count"] = len(discovered_nodes)

        for req in required:
            clean_req = req.lstrip("/")
            # Accept either /controller_manager or Gazebo plugin /gz_ros_control
            if clean_req == "controller_manager" and (
                "gz_ros_control" in discovered_nodes
                or "controller_manager" in discovered_nodes
            ):
                continue
            if clean_req not in discovered_nodes:
                self.failed_checks.append(f"Missing node: /{clean_req}")

    def _resolve_msg_type(self, type_str: str):
        try:
            mod_name, cls_name = type_str.replace("/", ".").rsplit(".", 1)
            module = importlib.import_module(mod_name)
            return getattr(module, cls_name)
        except Exception:
            return None

    def check_topic_health(self):
        sample_time = 1.2
        required_topics = self.cfg.get("monitoring", {}).get("required_topics", [])
        topic_types = dict(self.get_topic_names_and_types())

        self.metrics_snapshot["topics"] = {}

        for t_info in required_topics:
            topic = t_info["name"]
            min_hz = float(t_info.get("min_hz", 1.0))

            if topic not in topic_types or not topic_types[topic]:
                self.failed_checks.append(f"Topic not published: {topic}")
                self.metrics_snapshot["topics"][topic] = {
                    "hz": 0.0,
                    "status": "NOT_PUBLISHED",
                }
                continue

            msg_cls = self._resolve_msg_type(topic_types[topic][0])
            if msg_cls is None:
                self.failed_checks.append(f"Cannot resolve message type for {topic}")
                continue

            received_timestamps = []
            header_stamps = []

            def cb(msg):
                now_wall = time.time()
                received_timestamps.append(now_wall)
                if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                    stamp_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
                    header_stamps.append(stamp_sec)

            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            sub = self.create_subscription(msg_cls, topic, cb, qos)
            t_start = time.time()
            while time.time() - t_start < sample_time:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.destroy_subscription(sub)

            count = len(received_timestamps)
            actual_hz = count / sample_time
            self.metrics_snapshot["topics"][topic] = {"hz": round(actual_hz, 1)}

            if count == 0:
                self.failed_checks.append(f"Topic dead (0 Hz): {topic}")
                self.metrics_snapshot["topics"][topic]["status"] = "DEAD"
            elif actual_hz < min_hz:
                self.degraded_checks.append(
                    f"Topic degraded: {topic} at {actual_hz:.1f} Hz (<{min_hz} Hz expected)"
                )
                self.metrics_snapshot["topics"][topic]["status"] = "DEGRADED"
            else:
                self.metrics_snapshot["topics"][topic]["status"] = "HEALTHY"
                if header_stamps:
                    now_ros_sec = self.get_clock().now().nanoseconds * 1e-9
                    latest_stamp = max(header_stamps)
                    age = abs(now_ros_sec - latest_stamp)
                    if age > self.stale_timeout:
                        self.failed_checks.append(
                            f"Topic {topic} stamp stale: age {age:.2f}s > {self.stale_timeout}s limit"
                        )

    def check_action_servers(self):
        nav_action = self.cfg.get("navigation", {}).get(
            "action_server", "/navigate_to_pose"
        )
        client = ActionClient(self, NavigateToPose, nav_action)
        is_ready = client.wait_for_server(timeout_sec=2.0)
        self.metrics_snapshot["nav_action_ready"] = is_ready

        if not is_ready:
            self.failed_checks.append(f"Action server offline: {nav_action}")
        client.destroy()

    def run_full_audit(self) -> dict:
        self.failed_checks.clear()
        self.degraded_checks.clear()
        self.metrics_snapshot.clear()

        self.check_system_resources()
        self.check_nodes()
        self.check_topic_health()
        self.check_action_servers()

        if self.failed_checks:
            status = "NOT_READY"
        elif self.degraded_checks:
            status = "DEGRADED"
        else:
            status = "READY"

        return {
            "status": status,
            "failed_checks": self.failed_checks.copy(),
            "degraded_checks": self.degraded_checks.copy(),
            "metrics": self.metrics_snapshot.copy(),
            "timestamp": time.time(),
        }


def evaluate_readiness(config_path: str = None, sim_time: bool = None) -> dict:
    if config_path is None:
        config_path = get_default_config_path()

    already_initialized = rclpy.ok()
    if not already_initialized:
        rclpy.init()

    monitor = HealthMonitor(config_path, sim_time_override=sim_time)
    result = monitor.run_full_audit()
    monitor.destroy_node()

    if not already_initialized:
        rclpy.shutdown()

    return result


def print_dashboard(res: dict):
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    RESET = "\033[0m"

    status = res["status"]
    color = GREEN if status == "READY" else (YELLOW if status == "DEGRADED" else RED)

    print(f"\n{BOLD}===================================================={RESET}")
    print(f"{BOLD}   TIAGo SYSTEM HEALTH STATUS: {color}{status}{RESET}")
    print(f"{BOLD}===================================================={RESET}")

    metrics = res.get("metrics", {})
    if "cpu_percent" in metrics:
        print(
            f"Host: CPU {metrics['cpu_percent']}% | RAM {metrics['ram_percent']}% | Disk Free {metrics['disk_free_gb']} GB"
        )
    if "active_nodes_count" in metrics:
        print(f"Graph: {metrics['active_nodes_count']} Active Nodes Discovered")

    if "topics" in metrics:
        print(f"\n{BOLD}Monitored Topics:{RESET}")
        for t, data in metrics["topics"].items():
            t_status = data.get("status", "OK")
            t_col = (
                GREEN
                if t_status == "HEALTHY"
                else (YELLOW if t_status == "DEGRADED" else RED)
            )
            print(f"  {t_col}[{t_status:<9}]{RESET} {t:<35} : {data.get('hz', 0.0)} Hz")

    if res["failed_checks"]:
        print(f"\n{BOLD}{RED}Critical Failures:{RESET}")
        for err in res["failed_checks"]:
            print(f"  {RED}[FAIL]{RESET} {err}")

    if res["degraded_checks"]:
        print(f"\n{BOLD}{YELLOW}Operational Warnings:{RESET}")
        for warn in res["degraded_checks"]:
            print(f"  {YELLOW}[WARN]{RESET} {warn}")

    if not res["failed_checks"] and not res["degraded_checks"]:
        print(
            f"\n{GREEN}[OK] All systems, frequencies, and action servers are nominal.{RESET}"
        )
    print("----------------------------------------------------\n")


def main():
    parser = argparse.ArgumentParser(
        description="TIAGo Field Pre-flight Health Monitor"
    )
    parser.add_argument(
        "--config", type=str, default=None, help="Custom acceptance_tests.yaml path"
    )
    parser.add_argument("--sim", action="store_true", help="Force use_sim_time:=true")
    parser.add_argument(
        "--json", action="store_true", help="Output machine-readable JSON only"
    )
    parser.add_argument(
        "--loop", type=float, default=None, help="Run continuously every N seconds"
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Run continuously every 2.0 seconds with screen clear",
    )
    args, _ = parser.parse_known_args()

    loop_interval = 2.0 if args.watch else args.loop

    try:
        while True:
            if args.watch:
                print("\033[H\033[J", end="")

            res = evaluate_readiness(
                config_path=args.config, sim_time=True if args.sim else None
            )

            if args.json:
                print(json.dumps(res, indent=2))
            else:
                print_dashboard(res)

            if loop_interval is None:
                sys.exit(0 if res["status"] == "READY" else 1)

            time.sleep(loop_interval)

    except KeyboardInterrupt:
        print("\nHealth monitor stopped by user.")
        sys.exit(0)


if __name__ == "__main__":
    main()
