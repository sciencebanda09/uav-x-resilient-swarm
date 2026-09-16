"""Train and checkpoint a UAV-X CCPL policy on delayed mission consequences."""
from __future__ import annotations
import argparse, json
from pathlib import Path
from .core.ccpl_adapter import CCPLPolicy
from .core.ccpl_training import SwarmCCPLEnv

def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--episodes", type=int, default=25); p.add_argument("--seed", type=int, default=7); p.add_argument("--out", default="checkpoints/uav_x_ccpl.pkl"); a = p.parse_args()
    policy = CCPLPolicy(seed=a.seed, required=True); env = SwarmCCPLEnv(seed=a.seed)
    results = policy.train(env, episodes=a.episodes); Path(a.out).parent.mkdir(parents=True, exist_ok=True); policy.save_checkpoint(a.out)
    print(json.dumps({"backend": policy.backend, "checkpoint": a.out, "episodes": len(results), "last": results[-1] if results else {}}, indent=2))

if __name__ == "__main__": main()
