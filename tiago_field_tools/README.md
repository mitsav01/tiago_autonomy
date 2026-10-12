# `tiago_field_tools`

An industrial validation, automated diagnostics, and Site Acceptance Test (SAT) engineering toolkit for the PAL Robotics TIAGo mobile manipulator under ROS 2 Jazzy.

---

---

## 1. Executive Summary

`tiago_field_tools` bridges the gap between raw robotic stack execution and verified field readiness. Built for **Field Application Engineers** and **System Integration Teams**, it converts manual verification into a reproducible test protocol:

* **Pre-Flight Health Audit (`health_monitor.py`):** Multi-factor readiness evaluations auditing host memory/CPU headroom, node graph discovery, sensor stream frequencies (`/scan`, `/odom`, `/tf`, `/head_front_camera/image`), and hardware/simulation clock desynchronization.
* **Automated Mission Execution (`test_runner.py`):** Parameterized Nav2 trial execution with pre-flight safety gates, goal timeout enforcement, and immediate JSONL telemetry logging.
* **Statistical Aggregation (`metrics_collector.py`):** Turnkey extraction of Key Performance Indicators (KPIs): success rates, median/P95 latencies, min/max execution times, and failure mode classifications.
* **Site Acceptance Test Reports (`report_generator.py`):** Automatic synthesis of human-readable Markdown engineering reports complete with acceptance threshold comparisons, baseline deltas, and actionable root-cause hypotheses.

---

## 2. Architecture & Data Flow

```text
+-----------------------------------------------------------------------------------------+
|                               config/acceptance_tests.yaml                              |
|           (Thresholds, Monitored Topics & Rates, Required Nodes, Waypoint Targets)      |
+-----------------------------------------------------------------------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------------+
|                                     test_runner.py                                      |
|                                                                                         |
|   +---------------------------------------+                                             |
|   |           health_monitor.py           |                                             |
|   |  - Host RAM / CPU Headroom Check      |                                             |
|   |  - ROS 2 Node Graph Discovery         |                                             |
|   |  - Sensor Topic Hz & Stamp Age Audit  |                                             |
|   |  - Nav2 / Docking Action Server Check |                                             |
|   +---------------------------------------+                                             |
|                       |                                                                 |
|         +-------------+-------------+                                                   |
|         |                           |                                                   |
|  [ READY / DEGRADED ]          [ NOT_READY ]                                            |
|         |                           |                                                   |
|         v                           |                                                   |
|  Dispatch Nav2 Goal                 |                                                   |
|  (/navigate_to_pose)                |                                                   |
|         |                           |                                                   |
|         v                           |                                                   |
|  Timeout & Recovery Guard           v                                                   |
|         |                   Abort Trial Early                                           |
|         |                   (Skip Dispatch)                                             |
|         |                           |                                                   |
|         +-------------+-------------+                                                   |
|                       |                                                                 |
|                       v                                                                 |
|               raw_trials.jsonl                                                          |
+-----------------------------------------------------------------------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------------+
|                                  metrics_collector.py                                   |
|             - Success Rates, Median / P95 Latencies, Min / Max Execution Limits         |
|             - Categorize Failure Reasons (Costmap Lethal, Timeouts, OOM Drops)          |
|             - Output: aggregated_metrics.json & metrics.csv                             |
+-----------------------------------------------------------------------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------------+
|                                  report_generator.py                                    |
|             - Verify Against Config Pass / Fail Acceptance Criteria                     |
|             - Compare Against Benchmark / Previous Baselines                            |
|             - Output: acceptance_report.md                                              |
+-----------------------------------------------------------------------------------------+

```

---

## 3. Package Structure

```text
tiago_field_tools/
├── CMakeLists.txt                    # Pure Python install directives via ament_cmake
├── package.xml                       # Package dependencies (Nav2, action_msgs, psutil)
├── config/
│   └── acceptance_tests.yaml         # Acceptance thresholds, monitored topics, and goals
├── scripts/
│   ├── health_monitor.py             # Pre-flight host & ROS 2 graph audit tool
│   ├── test_runner.py                # Parameterized Nav2 batch execution engine
│   ├── metrics_collector.py          # Metric aggregator & statistical processor
│   └── report_generator.py           # Site Acceptance Test Markdown report generator
├── tests/
│   └── test_schema_and_metrics.py    # Offline unit tests for config parsing & KPI math
└── README.md

```

---

## 4. Acceptance Criteria Configuration (`acceptance_tests.yaml`)

All criteria are controlled declaratively in `config/acceptance_tests.yaml`:

```yaml
test_suite:
  name: "tiago_application_validation"
  repetitions: 10
  output_dir: "/tmp/tiago_validation_results"

thresholds:
  min_nav_success_rate: 0.85
  min_dock_success_rate: 0.80
  max_nav_duration_s: 90.0
  max_dock_duration_s: 45.0
  max_dock_position_error_m: 0.05
  max_dock_yaw_error_deg: 3.0

monitoring:
  topic_stale_timeout_seconds: 2.0
  record_cpu_memory: true
  required_nodes:
    - "/robot_state_publisher"
    - "/controller_manager"
    - "/bt_navigator"
    - "/docking_server"
  required_topics:
    - name: "/scan"
      expected_hz: 10.0
      min_hz: 7.0
    - name: "/mobile_base_controller/odom"
      expected_hz: 50.0
      min_hz: 30.0
    - name: "/tf"
      expected_hz: 50.0
      min_hz: 30.0
    - name: "/head_front_camera/image"
      expected_hz: 15.0
      min_hz: 5.0

navigation:
  action_server: "/navigate_to_pose"
  timeout_seconds: 120.0
  goals:
    - {x: 1.5, y: 0.0, yaw: 0.0}
    - {x: 0.5, y: -0.5, yaw: 1.57}
    - {x: 0.0, y: 0.0, yaw: 0.0}

docking:
  action_server: "/dock_robot"
  tag_id: 0
  timeout_seconds: 60.0
  expected_tag_frame: "tag_0"

```

---

## 5. Getting Started

### Prerequisites

* **Operating System:** Ubuntu 24.04 LTS (Noble) / Ubuntu 22.04 LTS (Jammy)
* **ROS 2:** Jazzy Jalisco (or Humble Hawksbill)
* **Python Runtime Dependencies:**
```bash
sudo apt-get update && sudo apt-get install -y \
  python3-psutil \
  python3-yaml \
  python3-numpy \
  python3-pytest

```



### Build & Workspace Installation

```bash
cd ~/portfolio
# Ensure execution permissions on scripts
chmod +x src/tiago_autonomy/tiago_field_tools/scripts/*.py

# Build using symlink install for quick script iterations
colcon build --symlink-install --packages-select tiago_field_tools
source install/setup.bash

```

---

## 6. Execution Guide

### Pre-Flight System Audit (`health_monitor.py`)

Run an instant snapshot audit:

```bash
ros2 run tiago_field_tools health_monitor.py --sim

```

Or run in persistent dashboard mode (auto-refreshing every 2 seconds):

```bash
ros2 run tiago_field_tools health_monitor.py --sim --watch

```

Output display:

```text
====================================================
   TIAGo SYSTEM HEALTH STATUS: READY
====================================================
Host: CPU 32.4% | RAM 68.2% | Disk Free 5.86 GB
Graph: 32 Active Nodes Discovered

Monitored Topics:
  [HEALTHY  ] /scan                               : 10.0 Hz
  [HEALTHY  ] /mobile_base_controller/odom        : 50.0 Hz
  [HEALTHY  ] /tf                                 : 50.0 Hz
  [HEALTHY  ] /head_front_camera/image            : 15.0 Hz

[OK] All systems, frequencies, and action servers are nominal.
----------------------------------------------------

```

### Automated Acceptance Trials (`test_runner.py`)

Launch the trial suite against your active simulation or physical robot:

```bash
ros2 run tiago_field_tools test_runner.py --ros-args -p use_sim_time:=true
```

Live progress output:

```text
==========================================
 Starting Acceptance Test Suite (10 Trials)
 Target File: /tmp/tiago_validation_results/raw_trials.jsonl
==========================================

Trial 1/10 -> Goal: x=1.5, y=0.0, yaw=0.0
  Result: SUCCESS | Duration: 18.42s

Trial 2/10 -> Goal: x=0.5, y=-0.5, yaw=1.57
  Result: SUCCESS | Duration: 12.18s

```

### Metrics Aggregation & Reporting

```bash
# 1. Aggregate JSONL into structured statistics and tabular CSV
ros2 run tiago_field_tools metrics_collector.py

# 2. Compile Site Acceptance Test (SAT) Markdown report
ros2 run tiago_field_tools report_generator.py

# 3. View the generated report
cat /tmp/tiago_validation_results/acceptance_report.md

```

---

## 7. Sample Field SAT Acceptance Report

```markdown
# Site Acceptance Test (SAT) Protocol: tiago_application_validation
**Date:** 2026-10-12 23:21:00  
**Overall Result:** 🟢 PASS

---

## 1. Key Performance Summary
| Metric | Measured | Target Threshold | Baseline | Status |
| :--- | :--- | :--- | :--- | :--- |
| Navigation Success Rate | **90.0%** | ≥ 85% | 70.0% | OK |
| Median Navigation Duration | **38.6 s** | ≤ 90.0 s | 54.2 s | OK |
| 95th Percentile Duration | **64.3 s** | N/A | 82.1 s | OK |

## 2. Failure Analysis Breakdown
| Failure Classification | Occurrences | Root Cause Hypothesis |
| :--- | :--- | :--- |
| `Nav2 aborted goal` | 1 | Costmap lethal inflation zone collision at goal |

## 3. Engineering Recommendations
- The deployed build satisfies all functional acceptance criteria.
- Target goal `(0.5, -0.5)` is positioned near a costmap inflation boundary; offset by +0.15m in X for enhanced clearance.

```

---

## 8. Continuous Integration Pipeline

The package includes multi-stage automation compatible with standard GitHub Actions runners (`ubuntu-noble` / ROS 2 Jazzy):

1. **Stage 1 (Unit & Schema Tests):** Verifies `acceptance_tests.yaml` schema integrity and executes statistical calculations against synthetic datasets via `pytest` (< 5 seconds).
2. **Stage 2 (Mock Graph Audit):** Verifies that `health_monitor.py` flags `NOT_READY` and returns exit code `1` when run on an idle/empty ROS 2 graph without hanging.
3. **Stage 3 (Simulation Integration Suite):** Executes headless Gazebo integration runs under `xvfb-run` when manually dispatched via `workflow_dispatch`.
4. **Stage 4 (Artifact Archival):** Packages `acceptance_report.md`, `aggregated_metrics.json`, and `metrics.csv` as downloadable GitHub Action run artifacts.

---

## 9. Diagnostic Runbook & Common Field Issues

| Observed Failure | Root Cause | Engineering Remediation |
| --- | --- | --- |
| **`RAM critical: >94%` or missing nodes** | Host Linux Out-Of-Memory (OOM) killer terminated `gzserver` or ROS node. | Add a 6 GB swap space (`sudo fallocate -l 6G /swapfile && sudo swapon /swapfile`) and run Gazebo headless (`gui:=false`). |
| **`stamp stale: age 1.79e9s > 2.0s`** | Clock desynchronization: Node evaluating wall-clock time while stack runs on simulation time. | Add `--sim` flag or pass `--ros-args -p use_sim_time:=true`. |
| **`Nav2 returned status code 6` in < 0.1s** | `STATUS_ABORTED`: Goal falls inside obstacle space, costmap inflation layer, or outside global map bounds. | Verify goal coordinates in RViz; check `/global_costmap/costmap` occupancy grid. |
| **`Topic degraded: /scan at 4.2 Hz`** | High CPU saturation causing Gazebo Real-Time Factor (RTF) to drop below 0.5. | Reduce simulator update rate, disable RViz/GUI, or lower physics solver iteration count in world file. |

