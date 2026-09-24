"""Run controlled UAV-X ablations for the Stage 1 evidence package."""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_x.core.models import ScenarioConfig
from uav_x.simulation.runner import run


SCENARIOS = {
    "baseline": ScenarioConfig(seed=7, duration_s=2700,
                                emergency_time_s=None),
    "return_home_recharge": ScenarioConfig(seed=29, duration_s=2700,
                                            initial_battery_pct=35,
                                            return_home_time_s=300,
                                            emergency_time_s=None),
    "simultaneous_failure_outage": ScenarioConfig(
        seed=31, duration_s=2700, failure_time_s=800, outage_start_s=850,
        outage_duration_s=120, packet_loss=0.08,
        emergency_time_s=None),
}


def variants(name: str, cfg: ScenarioConfig) -> ScenarioConfig:
    if name == "full_system":
        return cfg
    if name == "no_retransmissions":
        return replace(cfg, retransmit_limit=0)
    if name == "hop_only_routing":
        return replace(cfg, reliability_aware_routing=False)
    if name == "slow_recharge":
        return replace(cfg, charge_rate_pct_s=0.7, recharge_resume_pct=95.0,
                       recharge_trigger_margin_pct=0.0)
    raise ValueError(f"unknown ablation: {name}")


def evaluate() -> dict:
    rows = []
    for scenario, base_cfg in SCENARIOS.items():
        full_summary = None
        for variant in ("full_system", "no_retransmissions",
                        "hop_only_routing", "slow_recharge"):
            cfg = variants(variant, base_cfg)
            summary = run(cfg, policy="heuristic", scenario_name=f"ablation-{scenario}-{variant}")[-1]
            if variant == "full_system":
                full_summary = summary
            rows.append({"scenario": scenario, "variant": variant,
                         "summary": summary})
        for row in rows:
            if row["scenario"] == scenario and row["variant"] != "full_system":
                for metric in ("mission_completion_rate", "packet_delivery_ratio",
                               "connectivity_availability", "mission_completion_time_s"):
                    value = row["summary"][metric]
                    reference = full_summary[metric]
                    row.setdefault("delta_vs_full", {})[metric] = (
                        None if value is None or reference is None else value - reference
                    )
    return {"suite": "ablation_eval/v1", "duration_s": 2700.0,
            "scenarios": list(SCENARIOS), "variants": ["full_system",
            "no_retransmissions", "hop_only_routing", "slow_recharge"],
            "runs": rows}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="reports/ablations.json")
    args = parser.parse_args()
    result = evaluate()
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    compact = [{"scenario": row["scenario"], "variant": row["variant"],
                "completion": row["summary"]["mission_completion_rate"],
                "delivery": row["summary"]["packet_delivery_ratio"],
                "availability": row["summary"]["connectivity_availability"]}
               for row in result["runs"]]
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
