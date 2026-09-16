"""Compare heuristic and CCPL policy runs on identical seeds/configurations."""
from __future__ import annotations
import argparse, json
from pathlib import Path
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

def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--seed", type=int, default=7); p.add_argument("--duration", type=float, default=120.0); p.add_argument("--out", default="reports/policy_comparison.json")
    a = p.parse_args(); result = compare(a.seed, a.duration); Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f: json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))

if __name__ == "__main__": main()
