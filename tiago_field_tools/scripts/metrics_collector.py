#!/usr/bin/env python3
import json
import csv
import numpy as np

def aggregate_metrics(raw_jsonl_path, output_json_path, output_csv_path):
    trials = []
    with open(raw_jsonl_path, 'r') as f:
        for line in f:
            if line.strip():
                trials.append(json.loads(line))

    if not trials:
        print("No trials found to aggregate.")
        return

    total = len(trials)
    nav_trials = [t for t in trials if t['type'] == 'navigation']
    dock_trials = [t for t in trials if t['type'] == 'docking']

    def compute_stats(subset):
        if not subset:
            return {"count": 0, "success_rate": 0.0, "durations": []}
        successes = [t for t in subset if t['success']]
        durations = [t['duration'] for t in successes if t['duration'] is not None]
        failures = [t for t in subset if not t['success']]

        reasons = {}
        for f in failures:
            r = f.get('failure_reason', 'Unknown')
            reasons[r] = reasons.get(r, 0) + 1

        return {
            "total_trials": len(subset),
            "successful_trials": len(successes),
            "failed_trials": len(failures),
            "success_rate": len(successes) / len(subset),
            "median_duration_s": float(np.median(durations)) if durations else None,
            "p95_duration_s": float(np.percentile(durations, 95)) if durations else None,
            "min_duration_s": float(np.min(durations)) if durations else None,
            "max_duration_s": float(np.max(durations)) if durations else None,
            "failure_breakdown": reasons
        }

    summary = {
        "overall": {
            "total_trials": total,
            "success_rate": sum(1 for t in trials if t['success']) / total
        },
        "navigation": compute_stats(nav_trials),
        "docking": compute_stats(dock_trials)
    }

    # Save JSON summary
    with open(output_json_path, 'w') as f:
        json.dump(summary, f, indent=2)

    # Export flat CSV for graphing
    with open(output_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["trial_id", "type", "success", "duration_s", "failure_reason"])
        for t in trials:
            writer.writerow([
                t['trial_id'],
                t['type'],
                t['success'],
                f"{t['duration']:.2f}" if t['duration'] is not None else "",
                t.get('failure_reason', "")
            ])

    print(f"Metrics successfully written to:\n - {output_json_path}\n - {output_csv_path}")

if __name__ == '__main__':
    aggregate_metrics(
        "/tmp/tiago_validation_results/raw_trials.jsonl",
        "/tmp/tiago_validation_results/aggregated_metrics.json",
        "/tmp/tiago_validation_results/metrics.csv"
    )