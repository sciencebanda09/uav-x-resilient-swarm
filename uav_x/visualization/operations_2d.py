"""Dependency-light matplotlib operations view."""
from __future__ import annotations
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from ..interfaces.log_schema import read_jsonl

def render(path: str, output: str | None = None) -> None:
    if not Path(path).is_file():
        raise FileNotFoundError(
            f"UAV-X log not found: {path}. Run the simulator first, for example "
            "python -m uav_x.simulation.runner --out baseline.jsonl"
        )
    ticks = [r for r in read_jsonl(path) if r["record_type"] == "tick"]
    if not ticks: raise ValueError("log has no tick records")
    last = ticks[-1]; fig, ax = plt.subplots(figsize=(10, 8)); ax.set_title("UAV-X resilient swarm operations")
    obstacle_path = Path(__file__).resolve().parents[2] / "scene" / "obstacles.json"
    if obstacle_path.exists():
        import json
        for obstacle in json.loads(obstacle_path.read_text(encoding="utf-8")).get("obstacles", []):
            cx, cy, _ = obstacle["center_m"]; sx, sy, _ = obstacle["size_m"]
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor="#a66b45", edgecolor="#333333", alpha=.25, zorder=0))
    role_seen = set()
    for u in last["uavs"]:
        role = u["role"]
        label = role if role not in role_seen else "_nolegend_"
        role_seen.add(role)
        ax.scatter(u["position_m"][0], u["position_m"][1], s=90, label=label)
    for p in last["pois"]:
        color = "green" if p["status"] == "SURVEYED" else ("red" if p["priority"] == 1 else "orange")
        ax.scatter(p["position_m"][0], p["position_m"][1], marker="*", c=color, s=130)
    for link in last["links"]:
        if not link["available"]: continue
        src = next((u for u in last["uavs"] if u["id"] == link["source_id"]), None); dst = next((u for u in last["uavs"] if u["id"] == link["target_id"]), None)
        if src and dst: ax.plot([src["position_m"][0], dst["position_m"][0]], [src["position_m"][1], dst["position_m"][1]], "b-", alpha=.25)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.grid(alpha=.2); ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1,1)); fig.tight_layout()
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=140)
    else: plt.show()

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="artifacts/operations_2d.png"); a = p.parse_args(); render(a.log, a.out)
