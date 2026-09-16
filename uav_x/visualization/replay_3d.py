"""Optional Python cinematic 3D replay from the frozen JSONL log."""
from __future__ import annotations
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ..interfaces.log_schema import read_jsonl

def replay(path: str, output: str | None = None, interval_ms: int = 100) -> None:
    ticks = [r for r in read_jsonl(path) if r["record_type"] == "tick"]
    if not ticks: raise ValueError("log has no tick records")
    fig = plt.figure(figsize=(12, 8), facecolor="#08111c"); ax = fig.add_subplot(111, projection="3d")
    def draw(index: int):
        tick = ticks[index]; ax.clear(); ax.set_facecolor("#08111c"); ax.set_xlim(0, 500); ax.set_ylim(0, 500); ax.set_zlim(0, 140); ax.view_init(elev=38, azim=35 + index*.4)
        ax.set_xlabel("East"); ax.set_ylabel("North"); ax.set_zlabel("Altitude"); ax.set_title(f"UAV-X CINEMATIC REPLAY  |  t={tick['time_s']:.1f}s", color="white", pad=12)
        # Low-poly terrain reference surface for visual context.
        xs = [0, 125, 250, 375, 500]; ys = [0, 125, 250, 375, 500]
        X, Y = __import__("numpy").meshgrid(xs, ys); Z = 4 + 7*__import__("numpy").sin(X/110) * __import__("numpy").cos(Y/130)
        ax.plot_surface(X, Y, Z, cmap="terrain", alpha=.22, linewidth=0)
        for link in tick.get("links", []):
            if not link["available"]: continue
            a = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None); b = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if a and b: ax.plot([a["position_m"][0], b["position_m"][0]], [a["position_m"][1], b["position_m"][1]], [a["position_m"][2], b["position_m"][2]], color="#45aaf2", alpha=.42, linewidth=1.2)
        for p in tick["pois"]:
            x, y, z = p["position_m"]; ax.scatter(x, y, 5, c="#39d98a" if p["status"] == "SURVEYED" else "#ff4d6d" if p["priority"] == 1 else "#ffb547", marker="*", s=80)
        for u in tick["uavs"]:
            x, y, z = u["position_m"]; color = "#65748b" if u.get("failed") else "#39d98a" if u["role"] == "SURVEY" else "#45aaf2" if u["role"] == "RELAY" else "#ffb547"; ax.scatter(x, y, z, c=color, s=55, edgecolors="white", linewidth=.4); ax.text(x, y, z+5, u["id"], color="white", fontsize=7)
        ax.text2D(.02, .94, f"Connected: {sum(u['gcs_reachable'] for u in tick['uavs'] if not u.get('failed'))}/{len(tick['uavs'])}   |   Surveyed: {sum(p['status']=='SURVEYED' for p in tick['pois'])}/{len(tick['pois'])}", transform=ax.transAxes, color="white")
    anim = FuncAnimation(fig, draw, frames=len(ticks), interval=interval_ms, repeat=False)
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True); suffix = Path(output).suffix.lower(); anim.save(output, writer="pillow" if suffix == ".gif" else None, dpi=120)
    else: plt.show()

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="artifacts/replay_3d.gif"); p.add_argument("--interval", type=int, default=100); a = p.parse_args(); replay(a.log, a.out, a.interval)
