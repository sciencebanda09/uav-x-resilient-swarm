"""Optional Python cinematic 3D replay from the frozen JSONL log."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from ..interfaces.log_schema import read_jsonl

def replay(path: str, output: str | None = None, interval_ms: int = 100) -> None:
    ticks = [r for r in read_jsonl(path) if r["record_type"] == "tick"]
    if not ticks: raise ValueError("log has no tick records")
    fig = plt.figure(figsize=(12, 8), facecolor="#08111c"); ax = fig.add_subplot(111, projection="3d")
    heightmap_path = Path(__file__).resolve().parents[2] / "viewer" / "public" / "scene" / "terrain_heightmap.json"
    if heightmap_path.exists():
        terrain = json.loads(heightmap_path.read_text(encoding="utf-8")); source = np.asarray(terrain["elevation_m"], dtype=float); stride = max(1, source.shape[0] // 28); source = source[::stride, ::stride]; axis = np.linspace(0, 500, source.shape[0]); X, Y = np.meshgrid(axis, axis); Z = source
    else:
        axis = np.linspace(0, 500, 20); X, Y = np.meshgrid(axis, axis); Z = 4 + 7*np.sin(X/110) * np.cos(Y/130)
    terrain_max = float(np.max(Z))
    def draw(index: int):
        tick = ticks[index]; ax.clear(); ax.set_facecolor("#08111c"); ax.set_xlim(0, 500); ax.set_ylim(0, 500); ax.set_zlim(0, max(140, terrain_max + 65)); ax.view_init(elev=38, azim=35 + index*.4)
        ax.set_xlabel("East"); ax.set_ylabel("North"); ax.set_zlabel("Altitude"); ax.set_title(f"UAV-X CINEMATIC REPLAY  |  t={tick['time_s']:.1f}s", color="white", pad=12)
        ax.plot_surface(X, Y, Z, cmap="gist_earth", alpha=.48, linewidth=0, antialiased=True,
                        shade=True, edgecolor=(.08, .12, .10, .12))
        gcs_index = int(np.clip(round(40 / 500 * (len(axis) - 1)), 0, len(axis) - 1))
        ax.scatter(40, 40, float(Z[gcs_index, gcs_index]) + 20, c="#00c8ff", marker="s", s=90, edgecolors="black", linewidth=.8, depthshade=False)
        for link in tick.get("links", []):
            if not link["available"]: continue
            a = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None); b = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if a and b: ax.plot([a["position_m"][0], b["position_m"][0]], [a["position_m"][1], b["position_m"][1]], [a["position_m"][2], b["position_m"][2]], color="#45aaf2", alpha=.42, linewidth=1.2)
        for p in tick["pois"]:
            x, y, z = p["position_m"]; ax.scatter(x, y, z + 2, c="#00ff9d" if p["status"] == "SURVEYED" else "#ff1744" if p["priority"] == 1 else "#ffd000", marker="*", s=115, edgecolors="black", linewidth=.8, depthshade=False)
        for uav_index, u in enumerate(tick["uavs"]):
            x, y, z = u["position_m"]
            color = "#9aa0a6" if u.get("failed") else {"SURVEY":"#00ff66", "RELAY":"#00c8ff", "RECOVER":"#ff9d00", "RETURN":"#ff3158", "CHARGE":"#d86cff"}.get(u["role"], "#ffffff")
            ax.plot([x, x], [y, y], [float(np.min(Z)), z], color=color, alpha=.28, linewidth=.8)
            ax.scatter(x, y, z, c=color, s=115, marker="^", edgecolors="black", linewidth=1.0, depthshade=False)
            ax.text(x, y, z+5+(uav_index%3)*2, u["id"], color="white", fontsize=7, fontweight="bold",
                    bbox=dict(facecolor="black", alpha=.7, edgecolor=color, pad=1.5))
            if u.get("task_id"):
                target = next((p for p in tick["pois"] if p["id"] == u["task_id"]), None)
                if target:
                    ax.plot([x, target["position_m"][0]], [y, target["position_m"][1]], [z, target["position_m"][2] + 3], color="#00ff66", linestyle="--", linewidth=1.5, alpha=.85)
            if u.get("survey_capture_active") and u.get("survey_footprint_radius_m", 0) > 0:
                theta = np.linspace(0, 2*np.pi, 40); radius = u["survey_footprint_radius_m"]
                ax.plot(x + radius*np.cos(theta), y + radius*np.sin(theta), np.full_like(theta, z), color="#00ff66", alpha=.55, linewidth=1.2)
        coverage = tick.get("survey_coverage_fraction", 0.0) * 100.0
        ax.text2D(.02, .94, f"Connected: {sum(u['gcs_reachable'] for u in tick['uavs'] if not u.get('failed'))}/{len(tick['uavs'])}   |   Surveyed: {sum(p['status']=='SURVEYED' for p in tick['pois'])}/{len(tick['pois'])}   |   Mapped: {coverage:.1f}%", transform=ax.transAxes, color="white")
        ax.text2D(.02, .02, "GREEN dashed = survey assignment   |   green ring = camera footprint   |   cyan square = GCS", transform=ax.transAxes, color="white", fontsize=8, bbox=dict(facecolor="black", alpha=.75, edgecolor="#00c8ff", pad=4))
    anim = FuncAnimation(fig, draw, frames=len(ticks), interval=interval_ms, repeat=False)
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True); suffix = Path(output).suffix.lower(); anim.save(output, writer="pillow" if suffix == ".gif" else None, dpi=120)
    else: plt.show()

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="artifacts/replay_3d.gif"); p.add_argument("--interval", type=int, default=100); a = p.parse_args(); replay(a.log, a.out, a.interval)
