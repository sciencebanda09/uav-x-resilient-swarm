"""Synchronized operations replay with mission, battery, links, and events."""
from __future__ import annotations
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from ..interfaces.log_schema import read_jsonl

ROLE_COLORS = {"SURVEY": "#39d98a", "RELAY": "#45aaf2", "RECOVER": "#ffb547",
               "RETURN": "#ff6b6b", "CHARGE": "#c084fc", "STANDBY": "#aab4c3"}

def replay(path: str, output: str | None = None, interval_ms: int = 100) -> None:
    records = read_jsonl(path)
    ticks = [r for r in records if r["record_type"] == "tick"]
    events = [r for r in records if r["record_type"] == "event"]
    if not ticks: raise ValueError("log has no tick records")
    fig = plt.figure(figsize=(15, 8), facecolor="#101722")
    grid = fig.add_gridspec(2, 3, width_ratios=[3.6, 1.35, 1.35], height_ratios=[4, 1], hspace=.25, wspace=.22)
    ax = fig.add_subplot(grid[0, 0]); status = fig.add_subplot(grid[0, 1]); chart = fig.add_subplot(grid[0, 2]); timeline = fig.add_subplot(grid[1, :])
    axes = [ax, status, chart, timeline]
    for item in axes: item.set_facecolor("#101722"); item.tick_params(colors="#d8e2f0"); item.title.set_color("#ffffff")
    obstacle_path = Path(__file__).resolve().parents[2] / "scene" / "obstacles.json"
    obstacles = []
    if obstacle_path.exists():
        import json
        obstacles = json.loads(obstacle_path.read_text(encoding="utf-8")).get("obstacles", [])

    def draw(index: int):
        tick = ticks[index]; ax.clear(); status.clear(); chart.clear(); timeline.clear()
        ax.set_facecolor("#101722"); ax.set_xlim(0, 500); ax.set_ylim(0, 500); ax.set_aspect("equal"); ax.grid(color="#273449", alpha=.65)
        ax.set_title(f"UAV-X OPERATIONS REPLAY   t = {tick['time_s']:.1f}s", color="#ffffff", loc="left", fontweight="bold")
        ax.set_xlabel("East (m)", color="#aab4c3"); ax.set_ylabel("North (m)", color="#aab4c3")
        ax.add_patch(plt.Rectangle((2, 2), 496, 496, fill=False, edgecolor="#ffce56", linewidth=1.5, linestyle="--"))
        for obstacle in obstacles:
            cx, cy, _ = obstacle["center_m"]; sx, sy, _ = obstacle["size_m"]
            color = "#d97745" if obstacle.get("kind") == "building" else "#4f8fc9" if obstacle.get("kind") == "tower" else "#8b7355"
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=color, edgecolor="#f4f4f4", alpha=.28, linewidth=1.0, zorder=1))
        ax.scatter(40, 40, marker="s", s=150, c="#45aaf2", edgecolors="white", zorder=6)
        ax.annotate("GCS\ncommand + data sink", (40, 40), xytext=(8, 8), textcoords="offset points", color="#45aaf2", fontsize=7, bbox=dict(facecolor="#101722", alpha=.8, edgecolor="#45aaf2", pad=2))
        for link in tick.get("links", []):
            if not link["available"]: continue
            a = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None); b = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if a and b:
                q = 1.0 - link["packet_loss"]; ax.plot([a["position_m"][0], b["position_m"][0]], [a["position_m"][1], b["position_m"][1]], color=(.2, .8, .95, max(.12, q*.7)), linewidth=1 + q*2)
        for poi_index, p in enumerate(tick["pois"]):
            color = "#39d98a" if p["status"] == "SURVEYED" else "#ff4d6d" if p["priority"] == 1 else "#ffb547"
            ax.scatter(p["position_m"][0], p["position_m"][1], c=color, marker="*", s=160, edgecolors="white", linewidth=.4)
        for uav_index, u in enumerate(tick["uavs"]):
            x, y, _ = u["position_m"]; color = "#65748b" if u.get("failed") else ROLE_COLORS.get(u["role"], "white")
            ax.scatter(x, y, s=110, c=color, edgecolors="white", linewidth=.6, zorder=4)
            if u.get("survey_capture_active") and u.get("survey_footprint_radius_m", 0) > 0:
                ax.add_patch(Circle((x, y), u["survey_footprint_radius_m"], facecolor="#39d98a", edgecolor="#39d98a", alpha=.12, linewidth=1.2, zorder=1))
            if u.get("task_id"):
                target = next((p for p in tick["pois"] if p["id"] == u["task_id"]), None)
                if target:
                    ax.plot([x, target["position_m"][0]], [y, target["position_m"][1]], color="#39d98a", linestyle="--", linewidth=1.5, alpha=.75, zorder=2)
        ax.text(.02, .02, "GREEN dashed = assigned survey task   |   blue = relay/data route   |   translucent circle = camera footprint", transform=ax.transAxes, color="#d8e2f0", fontsize=8, bbox=dict(facecolor="#101722", alpha=.85, edgecolor="#45aaf2", pad=4))
        status.set_title("SWARM STATUS", loc="left", fontweight="bold"); status.axis("off")
        active = [u for u in tick["uavs"] if not u.get("failed")]; connected = sum(u["gcs_reachable"] for u in active); surveyed = sum(p["status"] == "SURVEYED" for p in tick["pois"])
        wind = tick.get("weather", {}).get("wind_mps", [0, 0, 0]); coverage = tick.get("survey_coverage_fraction", 0.0) * 100.0; status.text(.02, .96, f"CONNECTED   {connected}/{len(active)}\nSURVEYED    {surveyed}/{len(tick['pois'])}\nMAPPED      {coverage:.1f}%\nWIND        {wind[0]:.1f}, {wind[1]:.1f} m/s", color="#ffffff", va="top", fontsize=10, linespacing=1.35)
        y = .68
        for u in tick["uavs"]:
            status.text(.02, y, f"{u['id']}  {u['battery_pct']:5.1f}%  {u['role']}", color=ROLE_COLORS.get(u["role"], "#aab4c3"), fontsize=8); y -= .055
        chart.set_title("BATTERY / LINK QUALITY", loc="left", fontweight="bold"); chart.set_xlim(0, 1); chart.set_ylim(0, max(1, len(tick["uavs"])))
        chart.set_yticks(range(len(tick["uavs"]))); chart.set_yticklabels([u["id"] for u in tick["uavs"]], color="#d8e2f0", fontsize=8); chart.set_xticks([0, .5, 1]); chart.grid(axis="x", color="#273449")
        for row, u in enumerate(tick["uavs"]):
            chart.barh(row, u["battery_pct"] / 100, color=ROLE_COLORS.get(u["role"], "#aab4c3"), alpha=.85); chart.scatter(u["link_quality"], row, color="#ffffff", s=20, zorder=3)
        timeline.set_facecolor("#101722"); timeline.set_xlim(0, max(ticks[-1]["time_s"], 1)); timeline.set_ylim(0, 1); timeline.set_yticks([]); timeline.set_xlabel("MISSION TIMELINE (seconds)", color="#aab4c3"); timeline.grid(axis="x", color="#273449")
        labeled_events = set()
        for event in events:
            if event["time_s"] <= tick["time_s"]:
                event_type = event["event_type"]
                color = "#ff4d6d" if event_type in {"UAV_FAILURE", "LINK_OUTAGE"} else "#ffb547"
                timeline.axvline(event["time_s"], color=color, alpha=.35)
                if event_type not in labeled_events:
                    y = .25 + .16 * (len(labeled_events) % 4)
                    timeline.text(event["time_s"], y, event_type.replace("_", " "), rotation=35, color=color, fontsize=7, ha="right")
                    labeled_events.add(event_type)
        fig.suptitle("RESILIENT BVLOS SWARM | DISASTER RESPONSE", color="#45aaf2", fontsize=13, fontweight="bold")
    anim = FuncAnimation(fig, draw, frames=len(ticks), interval=interval_ms, repeat=False)
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True); suffix = Path(output).suffix.lower(); anim.save(output, writer="pillow" if suffix == ".gif" else None, dpi=120)
    else: plt.show()

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="artifacts/replay_2d.gif"); p.add_argument("--interval", type=int, default=100); a = p.parse_args(); replay(a.log, a.out, a.interval)
