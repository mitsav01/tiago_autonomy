import json
import os
from pathlib import Path
import sys
import tempfile
import yaml
import pytest

# Ensure scripts directory is resolvable regardless of where pytest is invoked
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from metrics_collector import aggregate_metrics


def test_acceptance_config_schema():
    cfg_file = PACKAGE_ROOT / "config" / "acceptance_tests.yaml"
    assert cfg_file.is_file(), f"acceptance_tests.yaml not found at {cfg_file}"

    with open(cfg_file, "r") as f:
        cfg = yaml.safe_load(f)

    for section in ["test_suite", "thresholds", "monitoring", "navigation"]:
        assert section in cfg, f"Missing section '{section}' in acceptance_tests.yaml"

    assert 0.0 < cfg["thresholds"]["min_nav_success_rate"] <= 1.0


def test_metrics_collector_math():
    with tempfile.TemporaryDirectory() as tmp:
        trials_path = Path(tmp) / "raw_trials.jsonl"
        out_json = Path(tmp) / "aggregated_metrics.json"
        out_csv = Path(tmp) / "metrics.csv"

        sample_trials = [
            {"trial_id": 1, "type": "navigation", "success": True, "duration": 20.0, "failure_reason": None},
            {"trial_id": 2, "type": "navigation", "success": True, "duration": 30.0, "failure_reason": None},
            {"trial_id": 3, "type": "navigation", "success": False, "duration": 60.0, "failure_reason": "Timeout"},
        ]
        with open(trials_path, "w") as f:
            for t in sample_trials:
                f.write(json.dumps(t) + "\n")

        aggregate_metrics(str(trials_path), str(out_json), str(out_csv))

        with open(out_json, "r") as f:
            data = json.load(f)

        assert data["navigation"]["total_trials"] == 3
        assert data["navigation"]["successful_trials"] == 2
        assert round(data["navigation"]["success_rate"], 2) == 0.67
        assert data["navigation"]["median_duration_s"] == 25.0