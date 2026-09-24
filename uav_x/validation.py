"""Monte Carlo, sensitivity, and policy comparison reports."""
from __future__ import annotations
import argparse, json
from pathlib import Path
from .core.models import ScenarioConfig
from .simulation.runner import run

def monte_carlo(seeds: list[int], duration: float = 2700.0) -> dict:
    summaries = [run(ScenarioConfig(seed=s, duration_s=duration), policy="heuristic")[-1] for s in seeds]
    return {"seeds": seeds, "mean_completion": sum(x["mission_completion_rate"] for x in summaries)/len(summaries), "mean_delivery": sum(x["packet_delivery_ratio"] for x in summaries)/len(summaries), "mean_latency_ms": sum(x["mean_latency_ms"] for x in summaries)/len(summaries), "runs": summaries}

def sensitivity(field: str, values: list[float], seed: int = 7, duration: float = 30.0) -> list[dict]:
    output = []
    for value in values:
        cfg = ScenarioConfig(seed=seed, duration_s=duration); setattr(cfg, field, value)
        summary = run(cfg, policy="heuristic")[-1]; output.append({"value": value, "packet_delivery_ratio": summary["packet_delivery_ratio"], "completion": summary["mission_completion_rate"], "latency_ms": summary["mean_latency_ms"]})
    return output

def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--out", default="reports/validation.json"); p.add_argument("--duration", type=float, default=2700.0); a = p.parse_args()
    result = {"duration_s": a.duration,
              "monte_carlo": monte_carlo([7, 17, 27], a.duration),
              "packet_loss_sensitivity": sensitivity("packet_loss", [0.0, .05, .1, .2], duration=a.duration)}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True); Path(a.out).write_text(json.dumps(result, indent=2), encoding="utf-8"); print(json.dumps(result, indent=2))

if __name__ == "__main__": main()
