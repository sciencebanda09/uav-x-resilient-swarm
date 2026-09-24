"""Evaluate UAV-X against deterministic but unannounced disturbance mixes.

The controller never receives a special hidden-test mode.  Each case is just a
normal ``ScenarioConfig`` assembled from a reproducible random seed, which
makes this useful as a Stage 1 robustness check and as a Stage 2 smoke test.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_x.core.models import ScenarioConfig
from uav_x.simulation.runner import run


def make_case(index: int, rng: random.Random, duration: float) -> ScenarioConfig:
    failure = 500.0 + rng.uniform(0.0, 1300.0) if rng.random() < 0.75 else None
    outage_start = 500.0 + rng.uniform(0.0, 1300.0) if rng.random() < 0.75 else None
    outage_duration = rng.uniform(30.0, 150.0) if outage_start is not None else 0.0
    emergency = 600.0 + rng.uniform(0.0, 900.0) if rng.random() < 0.65 else None
    return ScenarioConfig(
        seed=10_000 + index,
        duration_s=duration,
        # Keep the fleet and mission size aligned with the Stage 1 challenge
        # envelope; vary the disturbances, not the number of assigned PoIs.
        fleet_size=6,
        radio_range_m=rng.uniform(90.0, 115.0),
        packet_loss=rng.uniform(0.0, 0.12),
        failure_time_s=failure,
        outage_start_s=outage_start,
        outage_duration_s=outage_duration,
        emergency_time_s=emergency,
        initial_battery_pct=rng.uniform(80.0, 100.0),
        wind_scale=rng.uniform(0.8, 1.5),
        gps_noise_m=rng.uniform(0.0, 1.0),
    )


def evaluate(cases: int = 12, seed: int = 2026, duration: float = 2700.0) -> dict:
    rng = random.Random(seed)
    rows = []
    for index in range(cases):
        cfg = make_case(index, rng, duration)
        summary = run(cfg, policy="heuristic", scenario_name=f"hidden-{index + 1}")[-1]
        passed = (
            summary["mission_completion_rate"] >= 0.75
            and summary["actual_collision_count"] == 0
            and summary["battery_violations"] == 0
            and summary["geofence_violations"] == 0
            and summary["unlanded_count"] == 0
        )
        rows.append({"case": index + 1, "passed": passed,
                     "config": asdict(cfg), "summary": summary})
    metric_names = ("mission_completion_rate", "connectivity_availability",
                    "packet_delivery_ratio", "actual_collision_count",
                    "battery_violations", "geofence_violations")
    aggregate = {}
    for name in metric_names:
        values = [row["summary"][name] for row in rows]
        aggregate[name] = {"mean": sum(values) / len(values),
                           "min": min(values), "max": max(values)}
    return {"suite": "hidden_disturbance_eval/v1", "seed": seed,
            "cases": cases, "duration_s": duration,
            "pass_count": sum(row["passed"] for row in rows),
            "pass_rate": sum(row["passed"] for row in rows) / max(1, cases),
            "aggregate": aggregate, "runs": rows}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=12)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--duration", type=float, default=2700.0)
    parser.add_argument("--out", default="reports/hidden_disturbances.json")
    args = parser.parse_args()
    result = evaluate(args.cases, args.seed, args.duration)
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: result[key] for key in
                      ("suite", "cases", "duration_s", "pass_count", "pass_rate", "aggregate")},
                     indent=2))


if __name__ == "__main__":
    main()
