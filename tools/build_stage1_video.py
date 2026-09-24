"""Build a judge-facing failure/recovery animation from a canonical UAV-X log."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_x.interfaces.log_schema import read_jsonl


ROLE_COLORS = {"SURVEY": "#39d98a", "RELAY": "#45aaf2", "RECOVER": "#ffb547",
               "RETURN": "#ff6b6b", "CHARGE": "#c084fc", "STANDBY": "#aab4c3"}
EVENT_COLORS = {"UAV_FAILURE": "#ff3158", "LINK_OUTAGE": "#ff9d00",
                "RETURN_HOME": "#c084fc", "EMERGENCY_POI": "#ff4d6d"}


def build(log_path: str, output: str, stride: int = 2, fps: int = 12) -> None:
    records = read_jsonl(log_path)
    ticks = [row for row in records if row["record_type"] == "tick"]
    events = [row for row in records if row["record_type"] == "event"]
    if not ticks:
        raise ValueError("log contains no tick records")
    frames = list(range(0, len(ticks), max(1, stride)))
    final_time = ticks[-1]["time_s"]
    event_times = {event["event_type"]: event["time_s"] for event in events}
    manifest = next((row for row in records if row["record_type"] == "manifest"), {})
    arena = float(manifest.get("arena_m", 1000.0))
    gcs_xy = (manifest.get("gcs_position_m") or [-75.0, 500.0])[:2]
    margin = float((manifest.get("config") or {}).get("geofence_margin_m", 2.0))
    min_x = float((manifest.get("config") or {}).get("geofence_min_x_m", -100.0))

    fig = plt.figure(figsize=(12.8, 7.2), facecolor="#08111c")
    grid = fig.add_gridspec(2, 3, width_ratios=[2.4, 1.15, 1.15],
                            height_ratios=[3.0, 1.0], hspace=.28, wspace=.22)
    ax_map = fig.add_subplot(grid[:, 0])
    ax_status = fig.add_subplot(grid[0, 1])
    ax_metrics = fig.add_subplot(grid[0, 2])
    ax_timeline = fig.add_subplot(grid[1, 1:])

    def style(axis):
        axis.set_facecolor("#08111c")
        axis.tick_params(colors="#b9c8d8", labelsize=8)
        for spine in axis.spines.values():
            spine.set_color("#253449")

    for axis in (ax_map, ax_status, ax_metrics, ax_timeline):
        style(axis)

    def phase(time_s: float) -> str:
        if "UAV_FAILURE" in event_times and time_s >= event_times["UAV_FAILURE"]:
            if "LINK_OUTAGE" in event_times and time_s < event_times["LINK_OUTAGE"] + 15:
                return "COMBINED DISTURBANCE"
            return "SWARM RECOVERY"
        if "LINK_OUTAGE" in event_times and time_s >= event_times["LINK_OUTAGE"]:
            return "COMMUNICATION OUTAGE"
        return "NORMAL SURVEY"

    def draw(frame_index: int) -> None:
        tick = ticks[frame_index]
        time_s = tick["time_s"]
        for axis in (ax_map, ax_status, ax_metrics, ax_timeline):
            axis.clear()
            style(axis)

        ax_map.set_xlim(min_x, arena); ax_map.set_ylim(0, arena); ax_map.set_aspect("equal")
        ax_map.grid(color="#1b2b40", alpha=.75)
        ax_map.set_xlabel("East (m)", color="#b9c8d8")
        ax_map.set_ylabel("North (m)", color="#b9c8d8")
        ax_map.add_patch(Rectangle((min_x, margin), arena - min_x - margin, arena - 2 * margin, fill=False,
                                   edgecolor="#ffcf66", linestyle="--", linewidth=1.2))
        obstacle_path = Path(__file__).resolve().parents[1] / "scene" / "obstacles.json"
        if obstacle_path.exists():
            obstacles = json.loads(obstacle_path.read_text(encoding="utf-8")).get("obstacles", [])
            for obstacle in obstacles:
                cx, cy, _ = obstacle["center_m"]; sx, sy, _ = obstacle["size_m"]
                ax_map.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                           facecolor="#895e48", edgecolor="#d6b19e",
                                           alpha=.24, linewidth=.7))
        ax_map.scatter(*gcs_xy, marker="s", s=150, c="#45aaf2", edgecolors="white", zorder=6)
        ax_map.text(gcs_xy[0] + 10, gcs_xy[1] + 6, "GCS", color="#45aaf2", fontsize=8, weight="bold")
        for link in tick.get("links", []):
            if not link["available"]:
                continue
            source = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None)
            target = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if source and target:
                quality = 1.0 - link["packet_loss"]
                ax_map.plot([source["position_m"][0], target["position_m"][0]],
                            [source["position_m"][1], target["position_m"][1]],
                            color="#45aaf2", alpha=.12 + .45 * quality,
                            linewidth=.7 + 1.4 * quality)
        for poi in tick["pois"]:
            color = "#39d98a" if poi["status"] == "SURVEYED" else "#ff4d6d" if poi["priority"] == 1 else "#ffb547"
            ax_map.scatter(poi["position_m"][0], poi["position_m"][1], c=color,
                           marker="*", s=115, edgecolors="white", linewidth=.4, zorder=3)
        start = max(0, frame_index - 14)
        for uav_id in [u["id"] for u in tick["uavs"]]:
            path = [row["uavs"] for row in ticks[start:frame_index + 1]]
            points = [[u["position_m"][0], u["position_m"][1]] for group in path
                      for u in group if u["id"] == uav_id]
            if len(points) > 1:
                ax_map.plot([point[0] for point in points], [point[1] for point in points],
                            color="#73869d", alpha=.45, linewidth=1.0)
        for uav in tick["uavs"]:
            x, y, _ = uav["position_m"]
            color = "#687789" if uav.get("failed") else ROLE_COLORS.get(uav["role"], "white")
            ax_map.scatter(x, y, s=125, c=color, edgecolors="white", linewidth=.7, zorder=5)
            ax_map.text(x + 5, y + 5, uav["id"].replace("UAV-", "U"), color=color, fontsize=7, weight="bold")
            if uav.get("survey_capture_active") and uav.get("survey_footprint_radius_m", 0) > 0:
                ax_map.add_patch(Circle((x, y), uav["survey_footprint_radius_m"],
                                        facecolor="#39d98a", edgecolor="#39d98a", alpha=.10))
        ax_map.set_title(f"{phase(time_s)}   |   t = {time_s:.0f}s", loc="left",
                         color="#ffffff", weight="bold", fontsize=11)

        active = [u for u in tick["uavs"] if not u.get("failed")]
        surveyed = sum(p["status"] == "SURVEYED" for p in tick["pois"])
        connected = sum(u["gcs_reachable"] for u in active)
        ax_status.axis("off")
        ax_status.set_title("MISSION STATUS", loc="left", color="white", weight="bold")
        ax_status.text(.03, .92, f"SURVEYED\n{surveyed}/{len(tick['pois'])}", color="#39d98a", fontsize=17, weight="bold", va="top")
        ax_status.text(.03, .66, f"CONNECTED\n{connected}/{len(active)}", color="#45aaf2", fontsize=17, weight="bold", va="top")
        ax_status.text(.03, .40, f"COVERAGE\n{tick.get('survey_coverage_fraction', 0) * 100:.1f}%", color="#ffcf66", fontsize=17, weight="bold", va="top")
        ax_status.text(.03, .12, "CANONICAL LOG REPLAY\nNo state invented by renderer", color="#aab4c3", fontsize=8, va="top")

        ax_metrics.set_title("BATTERY / ROLE", loc="left", color="white", weight="bold")
        ax_metrics.set_xlim(0, 100); ax_metrics.set_ylim(-.5, len(tick["uavs"]) - .5)
        ax_metrics.set_yticks(range(len(tick["uavs"])))
        ax_metrics.set_yticklabels([u["id"].replace("UAV-", "U") for u in tick["uavs"]])
        ax_metrics.grid(axis="x", color="#1b2b40")
        for row, uav in enumerate(tick["uavs"]):
            color = "#687789" if uav.get("failed") else ROLE_COLORS.get(uav["role"], "white")
            ax_metrics.barh(row, uav["battery_pct"], color=color, alpha=.88)
            ax_metrics.text(min(98, uav["battery_pct"] + 2), row, uav["role"], color="#d8e2f0", fontsize=7, va="center")
        ax_metrics.set_xlabel("Battery (%)", color="#b9c8d8")

        times = [row["time_s"] for row in ticks[:frame_index + 1]]
        completion = [sum(p["status"] == "SURVEYED" for p in row["pois"]) / max(1, len(row["pois"])) for row in ticks[:frame_index + 1]]
        availability = [sum(u["gcs_reachable"] for u in row["uavs"] if not u.get("failed")) / max(1, sum(not u.get("failed") for u in row["uavs"])) for row in ticks[:frame_index + 1]]
        ax_timeline.plot(times, completion, color="#39d98a", linewidth=2.0, label="Mission completion")
        ax_timeline.plot(times, availability, color="#45aaf2", linewidth=1.7, label="GCS availability")
        for event in events:
            if event["time_s"] <= time_s:
                color = EVENT_COLORS.get(event["event_type"], "#ffcf66")
                ax_timeline.axvline(event["time_s"], color=color, alpha=.55, linewidth=1.0)
        ax_timeline.set_xlim(0, max(final_time, 1)); ax_timeline.set_ylim(0, 1.05)
        ax_timeline.set_xlabel("Mission time (s)", color="#b9c8d8")
        ax_timeline.set_ylabel("Fraction", color="#b9c8d8")
        ax_timeline.legend(loc="lower right", fontsize=7, facecolor="#101c2b", labelcolor="white")
        ax_timeline.grid(color="#1b2b40")

        current_events = [event["event_type"].replace("_", " ") for event in events
                          if event["event_type"] in EVENT_COLORS
                          and abs(event["time_s"] - time_s) < 0.51]
        if current_events:
            fig.text(.5, .942, "  •  ".join(current_events), ha="center", color="#ffcf66", weight="bold", fontsize=10)
        fig.suptitle("UAV-X  |  RESILIENT BVLOS SWARM  |  FAILURE → RECOVERY DEMONSTRATION",
                     color="#45aaf2", fontsize=14, weight="bold", y=.995)

    animation = FuncAnimation(fig, draw, frames=frames, interval=1000 / fps, repeat=False)
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.suffix.lower() != ".gif":
        raise SystemExit("This portable builder emits GIF video; use an external encoder for MP4 if desired.")
    animation.save(output_path, writer=PillowWriter(fps=fps), dpi=100)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--out", default="artifacts/stage1_failure_recovery.gif")
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--fps", type=int, default=12)
    args = parser.parse_args()
    build(args.log, args.out, args.stride, args.fps)
    print(args.out)


if __name__ == "__main__":
    main()
