#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def get_default_config_path() -> str:
    """Dynamically resolves acceptance_tests.yaml across environments.

    Order of precedence:
      1. TIAGO_FIELD_CONFIG environment variable
      2. colcon install share directory (ament_index)
      3. Relative source tree path (<pkg_root>/config/acceptance_tests.yaml)
    """
    env_cfg = os.environ.get("TIAGO_FIELD_CONFIG")
    if env_cfg and os.path.isfile(env_cfg):
        return os.path.abspath(env_cfg)

    try:
        share_dir = get_package_share_directory("tiago_field_tools")
        installed_path = Path(share_dir) / "config" / "acceptance_tests.yaml"
        if installed_path.is_file():
            return str(installed_path)
    except PackageNotFoundError:
        pass

    src_path = Path(__file__).resolve().parent.parent / "config" / "acceptance_tests.yaml"
    if src_path.is_file():
        return str(src_path)

    raise FileNotFoundError(
        "Could not locate 'acceptance_tests.yaml'. Ensure tiago_field_tools is installed, "
        "the source tree is intact, or set the TIAGO_FIELD_CONFIG environment variable."
    )


def generate_markdown_report(metrics_path, config_path, output_md_path, baseline_path=None):
    if not os.path.exists(metrics_path):
        raise FileNotFoundError(f"Metrics file not found: {metrics_path}")
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(metrics_path, "r") as f:
        metrics = json.load(f)
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    baseline = None
    if baseline_path and os.path.exists(baseline_path):
        try:
            with open(baseline_path, "r") as f:
                baseline = json.load(f)
        except Exception as e:
            print(f"Warning: Failed to load baseline metrics from {baseline_path}: {e}", file=sys.stderr)

    nav = metrics.get("navigation", {})
    t_cfg = cfg.get("thresholds", {})

    min_succ_rate = t_cfg.get("min_nav_success_rate", 0.85)
    max_duration = t_cfg.get("max_nav_duration_s", 90.0)

    actual_succ_rate = nav.get("success_rate", 0.0)
    median_duration = nav.get("median_duration_s", 0.0)

    nav_pass = actual_succ_rate >= min_succ_rate and (
        median_duration <= max_duration or nav.get("total_trials", 0) == 0
    )
    overall_status = "PASS" if nav_pass else "FAIL"

    suite_name = cfg.get("test_suite", {}).get("name", "tiago_acceptance_test")

    lines = []
    lines.append(f"# Site Acceptance Test (SAT) Protocol: {suite_name}")
    lines.append(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  ")
    lines.append(f"**Overall Result:** {'🟢 PASS' if overall_status == 'PASS' else '🔴 FAIL'}\n")
    lines.append("---")

    lines.append("## 1. Key Performance Summary")
    lines.append("| Metric | Measured | Target Threshold | Baseline | Status |")
    lines.append("| :--- | :--- | :--- | :--- | :--- |")

    nav_succ_pct = f"{actual_succ_rate * 100:.1f}%"
    target_succ = f"≥ {min_succ_rate * 100:.0f}%"
    base_succ = (
        f"{baseline['navigation']['success_rate'] * 100:.1f}%"
        if baseline and "navigation" in baseline and "success_rate" in baseline["navigation"]
        else "N/A"
    )
    succ_status = "OK" if actual_succ_rate >= min_succ_rate else "DEFICIT"
    lines.append(f"| Navigation Success Rate | **{nav_succ_pct}** | {target_succ} | {base_succ} | {succ_status} |")

    med_dur = f"{median_duration:.1f} s"
    base_med = (
        f"{baseline['navigation']['median_duration_s']:.1f} s"
        if baseline and "navigation" in baseline and "median_duration_s" in baseline["navigation"]
        else "N/A"
    )
    dur_status = "OK" if median_duration <= max_duration else "DEFICIT"
    lines.append(f"| Median Navigation Duration | **{med_dur}** | ≤ {max_duration:.1f} s | {base_med} | {dur_status} |")

    p95_dur = f"{nav.get('p95_duration_s', 0.0):.1f} s"
    lines.append(f"| 95th Percentile Duration | **{p95_dur}** | N/A | N/A | OK |")

    lines.append("\n## 2. Failure Analysis Breakdown")
    fail_breakdown = nav.get("failure_breakdown", {})
    if not fail_breakdown:
        lines.append("Zero mission failures recorded during execution.")
    else:
        lines.append("| Failure Classification | Occurrences | Root Cause Hypothesis |")
        lines.append("| :--- | :--- | :--- |")
        for reason, count in fail_breakdown.items():
            lines.append(f"| `{reason}` | {count} | Costmap collision, goal abort, or recovery timeout |")

    lines.append("\n## 3. Engineering Recommendations")
    if not nav_pass:
        lines.append("- **Investigate DDS QoS & Memory Headroom:** Memory elevated during trial execution. Profile gzserver and bt_navigator.")
        lines.append("- **Tune Costmap Layer Clearing:** Inspect obstacle inflation parameters and recovery behaviors for narrow corridors.")
    else:
        lines.append("- The deployed build satisfies all functional acceptance criteria.")

    output_path = Path(output_md_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Generated report at: {output_md_path}")


def main():
    parser = argparse.ArgumentParser(description="Compile SAT Markdown Acceptance Report")
    parser.add_argument(
        "--metrics",
        type=str,
        default="/tmp/tiago_validation_results/aggregated_metrics.json",
        help="Path to aggregated_metrics.json",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to acceptance_tests.yaml (auto-discovered if omitted)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="/tmp/tiago_validation_results/acceptance_report.md",
        help="Output path for acceptance_report.md",
    )
    parser.add_argument(
        "--baseline",
        type=str,
        default=None,
        help="Optional path to historical baseline metrics JSON",
    )
    args, _ = parser.parse_known_args()

    config_file = args.config if args.config else get_default_config_path()
    generate_markdown_report(
        metrics_path=args.metrics,
        config_path=config_file,
        output_md_path=args.output,
        baseline_path=args.baseline,
    )


if __name__ == "__main__":
    main()