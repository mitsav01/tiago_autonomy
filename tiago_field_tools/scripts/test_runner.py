#!/usr/bin/env python3
"""
TIAGo Field Tools - Navigation Test Runner
Executes automated navigation acceptance trials with pre-flight health validation,
goal timeout handling, and structured JSONL result logging.
"""

import json
import math
import os
from pathlib import Path
import sys
import time
import yaml

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

# Import health audit from sibling module
from health_monitor import evaluate_readiness


def get_default_config_path() -> str:
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

    src_relative_path = Path(__file__).resolve().parent.parent / "config" / "acceptance_tests.yaml"
    if src_relative_path.is_file():
        return str(src_relative_path)

    raise FileNotFoundError("Could not locate 'acceptance_tests.yaml'.")


class TestRunnerNode(Node):
    def __init__(self, config_path: str):
        super().__init__('tiago_test_runner')

        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.config_path = config_path

        # 1. Parameter management for simulation time
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        active_topics = [t[0] for t in self.get_topic_names_and_types()]
        self.is_sim_time = self.get_parameter('use_sim_time').value or ('/clock' in active_topics)
        if self.is_sim_time:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # 2. Action Client setup
        self.nav_action_name = self.cfg.get('navigation', {}).get('action_server', '/navigate_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        # 3. Output directory setup
        self.output_dir = Path(self.cfg.get('test_suite', {}).get('output_dir', '/tmp/tiago_validation_results'))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trials_file = self.output_dir / "raw_trials.jsonl"

    def execute_nav_trial(self, trial_id: int, goal_def: dict) -> dict:
        record = {
            "trial_id": trial_id,
            "type": "navigation",
            "goal": goal_def,
            "start_time": time.time(),
            "end_time": None,
            "duration": None,
            "success": False,
            "failure_reason": None,
            "pre_check": None,
        }

        # Step 1: Pre-flight health verification
        pre_check = evaluate_readiness(self.config_path, sim_time=self.is_sim_time)
        record["pre_check"] = pre_check

        if pre_check.get("status") == "NOT_READY":
            record["failure_reason"] = f"Pre-check failed: {', '.join(pre_check.get('failed_checks', []))}"
            record["end_time"] = time.time()
            record["duration"] = 0.0
            return record

        # Step 2: Ensure Action Server is reachable
        if not self.nav_client.wait_for_server(timeout_sec=4.0):
            record["failure_reason"] = f"Action server '{self.nav_action_name}' offline or unresponsive"
            record["end_time"] = time.time()
            record["duration"] = 0.0
            return record

        # Step 3: Construct NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_def['x'])
        goal_msg.pose.pose.position.y = float(goal_def['y'])
        yaw = float(goal_def.get('yaw', 0.0))
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        t_start = time.time()
        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            record["failure_reason"] = "Goal rejected by Nav2 Planner/Costmap"
            record["end_time"] = time.time()
            record["duration"] = round(record["end_time"] - t_start, 2)
            return record

        # Step 4: Monitor goal execution with timeout guard
        timeout = float(self.cfg.get('navigation', {}).get('timeout_seconds', 120.0))
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t_start > timeout:
                goal_handle.cancel_goal_async()
                record["failure_reason"] = f"Trial exceeded timeout limit of {timeout}s"
                record["end_time"] = time.time()
                record["duration"] = timeout
                return record

        record["end_time"] = time.time()
        record["duration"] = round(record["end_time"] - t_start, 2)
        action_result = result_future.result()

        if action_result.status == GoalStatus.STATUS_SUCCEEDED:
            record["success"] = True
        elif action_result.status == GoalStatus.STATUS_ABORTED:
            record["failure_reason"] = "Nav2 aborted goal (path planning failure or costmap lethal collision)"
        elif action_result.status == GoalStatus.STATUS_CANCELED:
            record["failure_reason"] = "Nav2 goal was canceled"
        else:
            record["failure_reason"] = f"Nav2 returned status code {action_result.status}"

        return record

    def run_suite(self):
        goals = self.cfg.get('navigation', {}).get('goals', [{'x': 1.0, 'y': 0.0, 'yaw': 0.0}])
        repetitions = int(self.cfg.get('test_suite', {}).get('repetitions', 5))

        print("\n==========================================")
        print(f" Starting Acceptance Test Suite ({repetitions} Trials)")
        print(f" Target File: {self.trials_file}")
        print("==========================================\n")

        with open(self.trials_file, 'w') as f_out:
            for i in range(repetitions):
                goal = goals[i % len(goals)]
                print(f"Trial {i + 1}/{repetitions} -> Goal: x={goal['x']}, y={goal['y']}, yaw={goal.get('yaw', 0.0)}")

                result = self.execute_nav_trial(i + 1, goal)
                status_str = "\033[92mSUCCESS\033[0m" if result['success'] else f"\033[91mFAILED\033[0m ({result['failure_reason']})"
                dur_str = f"{result['duration']}s" if result['duration'] is not None else "N/A"
                print(f"  Result: {status_str} | Duration: {dur_str}\n")

                f_out.write(json.dumps(result) + "\n")
                f_out.flush()
                time.sleep(1.0)


def main():
    rclpy.init()
    config_path = get_default_config_path()
    runner = TestRunnerNode(config_path)

    try:
        runner.run_suite()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()