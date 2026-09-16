"""Synchronized operations replay with mission, battery, links, and events."""
from __future__ import annotations
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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

    def draw(index: int):
        tick = ticks[index]; ax.clear(); status.clear(); chart.clear(); timeline.clear()
        ax.set_facecolor("#101722"); ax.set_xlim(0, 500); ax.set_ylim(0, 500); ax.set_aspect("equal"); ax.grid(color="#273449", alpha=.65)
        ax.set_title(f"UAV-X OPERATIONS REPLAY   t = {tick['time_s']:.1f}s", color="#ffffff", loc="left", fontweight="bold")
        ax.set_xlabel("East (m)", color="#aab4c3"); ax.set_ylabel("North (m)", color="#aab4c3")
        ax.add_patch(plt.Rectangle((2, 2), 496, 496, fill=False, edgecolor="#ffce56", linewidth=1.5, linestyle="--"))
        for link in tick.get("links", []):
            if not link["available"]: continue
            a = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None); b = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if a and b:
                q = 1.0 - link["packet_loss"]; ax.plot([a["position_m"][0], b["position_m"][0]], [a["position_m"][1], b["position_m"][1]], color=(.2, .8, .95, max(.12, q*.7)), linewidth=1 + q*2)
        for p in tick["pois"]:
            color = "#39d98a" if p["status"] == "SURVEYED" else "#ff4d6d" if p["priority"] == 1 else "#ffb547"
            ax.scatter(p["position_m"][0], p["position_m"][1], c=color, marker="*", s=160, edgecolors="white", linewidth=.4)
            ax.text(p["position_m"][0]+5, p["position_m"][1]+4, f"{p['id']} {p['survey_progress']*100:.0f}%", color="#d8e2f0", fontsize=7)
        for u in tick["uavs"]:
            x, y, _ = u["position_m"]; color = "#65748b" if u.get("failed") else ROLE_COLORS.get(u["role"], "white")
            ax.scatter(x, y, s=110, c=color, edgecolors="white", linewidth=.6, zorder=4); ax.text(x+5, y+5, f"{u['id']}\n{u['role']}", color="#ffffff", fontsize=7)
        status.set_title("SWARM STATUS", loc="left", fontweight="bold"); status.axis("off")
        active = [u for u in tick["uavs"] if not u.get("failed")]; connected = sum(u["gcs_reachable"] for u in active); surveyed = sum(p["status"] == "SURVEYED" for p in tick["pois"])
        wind = tick.get("weather", {}).get("wind_mps", [0, 0, 0]); status.text(.02, .96, f"CONNECTED   {connected}/{len(active)}\nSURVEYED    {surveyed}/{len(tick['pois'])}\nWIND        {wind[0]:.1f}, {wind[1]:.1f} m/s\n", color="#ffffff", va="top", fontsize=11, linespacing=1.5)
        y = .78
        for u in tick["uavs"]:
            status.text(.02, y, f"{u['id']}  {u['battery_pct']:5.1f}%  {u['role']}", color=ROLE_COLORS.get(u["role"], "#aab4c3"), fontsize=8); y -= .055
        chart.set_title("BATTERY / LINK QUALITY", loc="left", fontweight="bold"); chart.set_xlim(0, 1); chart.set_ylim(0, max(1, len(tick["uavs"])))
        chart.set_yticks(range(len(tick["uavs"]))); chart.set_yticklabels([u["id"] for u in tick["uavs"]], color="#d8e2f0", fontsize=8); chart.set_xticks([0, .5, 1]); chart.grid(axis="x", color="#273449")
        for row, u in enumerate(tick["uavs"]):
            chart.barh(row, u["battery_pct"] / 100, color=ROLE_COLORS.get(u["role"], "#aab4c3"), alpha=.85); chart.scatter(u["link_quality"], row, color="#ffffff", s=20, zorder=3)
        timeline.set_facecolor("#101722"); timeline.set_xlim(0, max(ticks[-1]["time_s"], 1)); timeline.set_ylim(0, 1); timeline.set_yticks([]); timeline.set_xlabel("MISSION TIMELINE (seconds)", color="#aab4c3"); timeline.grid(axis="x", color="#273449")
        for event in events:
            if event["time_s"] <= tick["time_s"]:
                color = "#ff4d6d" if event["event_type"] in {"UAV_FAILURE", "LINK_OUTAGE"} else "#ffb547"; timeline.axvline(event["time_s"], color=color, alpha=.65); timeline.text(event["time_s"], .55, event["event_type"].replace("_", " "), rotation=45, color=color, fontsize=7, ha="right")
        fig.suptitle("RESILIENT BVLOS SWARM | DISASTER RESPONSE", color="#45aaf2", fontsize=13, fontweight="bold")
    anim = FuncAnimation(fig, draw, frames=len(ticks), interval=interval_ms, repeat=False)
    if output:
        Path(output).parent.mkdir(parents=True, exist_ok=True); suffix = Path(output).suffix.lower(); anim.save(output, writer="pillow" if suffix == ".gif" else None, dpi=120)
    else: plt.show()

if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out", default="artifacts/replay_2d.gif"); p.add_argument("--interval", type=int, default=100); a = p.parse_args(); replay(a.log, a.out, a.interval)
