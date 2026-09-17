"""Benchmark and policy comparison helpers for the challenge submission."""
from __future__ import annotations
import argparse, json
from pathlib import Path
from statistics import mean, pstdev
from .core.models import ScenarioConfig
from .simulation.runner import run

def compare(seed: int = 7, duration: float = 120.0) -> dict:
    cfg = ScenarioConfig(seed=seed, duration_s=duration)
    result = {}
    for policy in ("heuristic", "ccpl"):
        try:
            result[policy] = run(cfg, policy=policy)[-1]
        except Exception as exc:
            result[policy] = {"error": str(exc)}
    return result


def _stats(rows: list[dict], field: str) -> dict:
    values = [float(row[field]) for row in rows if row.get(field) is not None]
    return {"mean": mean(values) if values else None,
            "min": min(values) if values else None,
            "max": max(values) if values else None,
            "stdev": pstdev(values) if len(values) > 1 else 0.0}


def run_suite(seeds: list[int] | None = None, duration: float = 120.0) -> dict:
    """Run the six challenge-facing scenarios over repeatable seeds."""
    seeds = [7, 17, 27] if seeds is None else seeds
    scenario_values = {
        "baseline": {},
        "outage_recovery": {"outage_start_s": 35, "outage_duration_s": 15},
        "uav_failure": {"failure_time_s": 48},
        "emergency_priority": {"emergency_time_s": 65},
        "return_home_recharge": {"initial_battery_pct": 35, "return_home_time_s": 10},
        "simultaneous_failure_outage": {"failure_time_s": 48, "outage_start_s": 45,
                                         "outage_duration_s": 15, "packet_loss": 0.08,
                                         "radio_range_m": 220},
    }
    scenarios = {}
    for name, overrides in scenario_values.items():
        rows = []
        for seed in seeds:
            cfg = ScenarioConfig(seed=seed, duration_s=duration, **overrides)
            summary = run(cfg, policy="heuristic", scenario_name=name)[-1]
            rows.append({"seed": seed, **summary})
        scenarios[name] = {
            "runs": rows,
            "statistics": {field: _stats(rows, field) for field in (
                "mission_completion_rate", "connectivity_availability",
                "packet_delivery_ratio", "mean_latency_ms",
                "actual_collision_count", "battery_violations",
                "geofence_violations", "recovery_time_s_mean")},
        }
    return {"seeds": seeds, "duration_s": duration, "scenarios": scenarios}

def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--seeds", type=int, nargs="+")
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--out", default="reports/policy_comparison.json")
    p.add_argument("--all", action="store_true", help="run the complete Stage 1 scenario suite")
    a = p.parse_args()
    result = run_suite(a.seeds, a.duration) if a.all else compare(a.seed, a.duration)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f: json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))

if __name__ == "__main__": main()
