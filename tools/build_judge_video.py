"""Build a polished 16:9 judge-facing UAV-X demonstration video.

The source of truth remains the canonical JSONL replay.  This renderer adds
presentation structure without inventing positions, events, or metrics.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_x.interfaces.log_schema import read_jsonl


ROLE = {"SURVEY": "#20e3a2", "RELAY": "#35a7ff", "BACKBONE": "#00d5ff", "RECOVER": "#ffbd5a",
        "RETURN": "#ff637d", "CHARGE": "#bd8cff", "STANDBY": "#9aa9bb",
        "LAND": "#738194"}
EVENT = {"UAV_FAILURE": ("UAV FAILURE", "#ff416c"),
         "LINK_OUTAGE": ("LINK OUTAGE", "#ffae42"),
         "EMERGENCY_POI": ("PRIORITY-1 EMERGENCY", "#ff4d6d"),
         "RETURN_HOME": ("RETURN-TO-HOME", "#bd8cff")}


def rounded(ax, xy, width, height, color="#0d1b2a", edge="#203a55", alpha=.96):
    patch = FancyBboxPatch(xy, width, height, boxstyle="round,pad=0.012,rounding_size=0.025",
                           transform=ax.transAxes, facecolor=color, edgecolor=edge,
                           linewidth=1.0, alpha=alpha)
    ax.add_patch(patch)


def build(log_path: str, output: str, stride: int = 20, fps: int = 24, hold: int = 12) -> None:
    rows = read_jsonl(log_path)
    ticks = [r for r in rows if r["record_type"] == "tick"]
    events = [r for r in rows if r["record_type"] == "event"]
    summary = next((r.get("payload", {}) for r in rows if r["record_type"] == "summary"), {})
    manifest = next((r.get("payload", {}) for r in rows if r["record_type"] == "manifest"), {})
    if not ticks:
        raise ValueError("log contains no tick records")

    # Keep the video short and intentional: one frame represents a meaningful
    # mission interval, while event frames are never skipped.
    selected = set(range(0, len(ticks), max(1, stride)))
    for event in events:
        if event["event_type"] not in EVENT:
            continue
        idx = min(len(ticks) - 1, max(0, int(round(event["time_s"]))))
        selected.update(range(max(0, idx - 2), min(len(ticks), idx + 3)))
    frames = sorted(selected)
    arena = float(manifest.get("arena_m", 1000.0))
    gcs = (manifest.get("gcs_position_m") or [-75.0, 500.0])[:2]
    min_x = float((manifest.get("config") or {}).get("geofence_min_x_m", -100.0))
    final_time = float(ticks[-1]["time_s"])
    event_by_time = {}
    for e in events:
        event_by_time.setdefault(round(float(e["time_s"])), []).append(e)

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#06101b")
    grid = fig.add_gridspec(12, 16, left=.035, right=.98, top=.90, bottom=.08,
                            wspace=.75, hspace=.75)
    ax = fig.add_subplot(grid[:9, :10])
    side = fig.add_subplot(grid[:8, 10:])
    timeline = fig.add_subplot(grid[9:, :])
    for axis in (ax, side, timeline):
        axis.set_facecolor("#06101b")
        axis.tick_params(colors="#9fb4c8", labelsize=8)
        for spine in axis.spines.values(): spine.set_color("#1d344a")

    def phase(t):
        active = [e["event_type"] for e in events if e["time_s"] <= t]
        if "UAV_FAILURE" in active and "LINK_OUTAGE" in active: return "RECOVERY / RECONFIGURATION"
        if "UAV_FAILURE" in active: return "FAILURE RECOVERY"
        if "LINK_OUTAGE" in active: return "COMMUNICATION OUTAGE"
        if "EMERGENCY_POI" in active: return "PRIORITY RESPONSE"
        return "AUTONOMOUS SURVEY"

    render_indices = [frame for frame in frames for _ in range(max(1, hold))]

    def draw(k):
        tick = ticks[render_indices[k]]; t = float(tick["time_s"])
        for axis in (ax, side, timeline): axis.clear()
        for axis in (ax, side, timeline):
            axis.set_facecolor("#06101b")
            axis.tick_params(colors="#9fb4c8", labelsize=8)
            for spine in axis.spines.values(): spine.set_color("#1d344a")

        # Main map.
        ax.set_xlim(min_x - 25, arena + 25); ax.set_ylim(-20, arena + 20); ax.set_aspect("equal")
        ax.grid(color="#17304a", alpha=.65, linewidth=.6)
        ax.set_xlabel("EASTING (m)", color="#9fb4c8", fontsize=8)
        ax.set_ylabel("NORTHING (m)", color="#9fb4c8", fontsize=8)
        ax.add_patch(Rectangle((0, 0), arena, arena, fill=False, edgecolor="#3e9fc5",
                               linestyle="--", linewidth=1.2, alpha=.9))
        ax.scatter(*gcs, marker="s", s=180, c="#35a7ff", edgecolors="white", linewidth=.8, zorder=8)
        ax.text(gcs[0] + 18, gcs[1] + 17, "GCS / HOME", color="#67c4ff", fontsize=8, weight="bold")

        # Faint operational sectors make the geometry legible at a glance.
        for x in (250, 500, 750): ax.axvline(x, color="#17304a", linewidth=.5, alpha=.4)
        for y in (250, 500, 750): ax.axhline(y, color="#17304a", linewidth=.5, alpha=.4)

        for link in tick.get("links", []):
            if not link.get("available"): continue
            source = next((u for u in tick["uavs"] if u["id"] == link["source_id"]), None)
            target = next((u for u in tick["uavs"] if u["id"] == link["target_id"]), None)
            if source and target:
                q = max(0.05, 1.0 - float(link.get("packet_loss", 1.0)))
                ax.plot([source["position_m"][0], target["position_m"][0]],
                        [source["position_m"][1], target["position_m"][1]],
                        color="#35a7ff", alpha=.12 + .55*q, linewidth=.6 + 1.8*q, zorder=2)

        for p in tick["pois"]:
            x, y = p["position_m"][:2]
            done = p["status"] == "SURVEYED"
            color = "#20e3a2" if done else ("#ff4d6d" if p["priority"] == 1 else "#ffbd5a")
            ax.scatter(x, y, marker="*", s=155 if p["priority"] == 1 else 105,
                       c=color, edgecolors="white", linewidth=.45, zorder=6)
            if done: ax.scatter(x, y, s=250, facecolors="none", edgecolors=color, alpha=.35, linewidth=1.0)

        for u in tick["uavs"]:
            x, y, _ = u["position_m"]
            color = "#657489" if u.get("failed") else ROLE.get(u["role"], "white")
            ax.scatter(x, y, s=105, c=color, edgecolors="white", linewidth=.7, zorder=7)
            # The takeoff/landing ring is intentionally dense; the role panel
            # already labels those vehicles, so suppress colliding map labels
            # while they are within one separation radius of home.
            if ((x - gcs[0]) ** 2 + (y - gcs[1]) ** 2) ** .5 > 80:
                ax.text(x + 8, y + 8, u["id"].replace("UAV-", "U"), color=color, fontsize=7, weight="bold")
            if u.get("survey_capture_active"):
                radius = max(8, float(u.get("survey_footprint_radius_m", 0)))
                ax.add_patch(Circle((x, y), radius, color="#20e3a2", alpha=.08, zorder=1))
        ax.set_title(f"{phase(t)}   •   T+{t/60:.1f} MIN", loc="left", color="white", weight="bold", fontsize=12)

        # Side KPI panel.
        side.set_xlim(0, 1); side.set_ylim(0, 1); side.axis("off")
        side.text(.02, .98, "MISSION CONTROL", color="#67c4ff", fontsize=12, weight="bold", va="top")
        side.text(.02, .935, "CANONICAL REPLAY / LIVE STATE", color="#718ba0", fontsize=7, va="top")
        surveyed = sum(p["status"] == "SURVEYED" for p in tick["pois"])
        active = [u for u in tick["uavs"] if not u.get("failed")]
        connected = sum(bool(u.get("gcs_reachable")) for u in active)
        cards = [("POI SURVEY", f"{surveyed}/{len(tick['pois'])}", "#20e3a2"),
                 ("GCS CONNECTIVITY", f"{connected}/{len(active)}", "#35a7ff"),
                 ("GROUND COVERAGE", f"{100*float(tick.get('survey_coverage_fraction',0)):.1f}%", "#ffbd5a")]
        for i, (label, value, color) in enumerate(cards):
            y = .80 - i*.17; rounded(side, (.02, y-.11), .96, .13)
            side.text(.06, y+.005, label, color="#8ea4b8", fontsize=7, va="center")
            side.text(.06, y-.065, value, color=color, fontsize=18, weight="bold", va="center")
        side.text(.02, .29, "SWARM ROLES", color="#67c4ff", fontsize=9, weight="bold")
        for i, u in enumerate(tick["uavs"]):
            y = .245 - i*.035
            side.add_patch(Rectangle((.03, y-.008), .025, .016, transform=side.transAxes,
                                     color="#657489" if u.get("failed") else ROLE.get(u["role"], "white")))
            side.text(.07, y, f"{u['id'].replace('UAV-','U')}  {u['role']:<7}  {u['battery_pct']:5.1f}%",
                      color="#c1d0dc", fontsize=7, va="center", family="monospace")

        # Time-series evidence.
        upto = render_indices[k] + 1; ts = [float(r["time_s"]) for r in ticks[:upto]]
        comp = [sum(p["status"] == "SURVEYED" for p in r["pois"])/max(1,len(r["pois"])) for r in ticks[:upto]]
        avail = [sum(bool(u.get("gcs_reachable")) for u in r["uavs"] if not u.get("failed"))/max(1,sum(not u.get("failed") for u in r["uavs"])) for r in ticks[:upto]]
        timeline.plot(ts, comp, color="#20e3a2", linewidth=2.3, label="Mission completion")
        timeline.plot(ts, avail, color="#35a7ff", linewidth=1.6, label="Reachable to GCS")
        for e in events:
            if e["event_type"] in EVENT and e["time_s"] <= t:
                color = EVENT.get(e["event_type"], ("", "#ffbd5a"))[1]
                timeline.axvline(e["time_s"], color=color, alpha=.55, linewidth=.8)
        timeline.axvline(t, color="white", alpha=.8, linewidth=1.0)
        timeline.set_xlim(0, max(final_time, 1)); timeline.set_ylim(0, 1.05)
        timeline.set_ylabel("FRACTION", color="#9fb4c8", fontsize=7)
        timeline.set_xlabel("MISSION TIME (s)", color="#9fb4c8", fontsize=7)
        timeline.grid(color="#17304a", alpha=.65); timeline.legend(loc="lower right", fontsize=7,
                     facecolor="#0d1b2a", edgecolor="#203a55", labelcolor="white")

        current = [e for e in event_by_time.get(round(t), []) if e["event_type"] in EVENT]
        if current:
            names = [EVENT.get(e["event_type"], (e["event_type"], "#ffbd5a"))[0] for e in current]
            fig.text(.53, .925, "  •  ".join(names), ha="center", color="#ffbd5a", fontsize=9, weight="bold")

    fig.suptitle("UAV-X  |  RESILIENT BVLOS SWARM", color="#67c4ff", fontsize=17, weight="bold", y=.978)
    fig.text(.035, .946, "DISASTER RESPONSE / AUTONOMOUS SURVEY / FAILURE RECOVERY", color="#718ba0", fontsize=8)
    animation = FuncAnimation(fig, draw, frames=len(render_indices), interval=1000/fps, repeat=False)
    output_path = Path(output); output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, bitrate=4500, metadata={"title": "UAV-X Stage 1 Demonstration"})
    animation.save(output_path, writer=writer, dpi=100)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--out", default="artifacts/uav_x_judge_demo.mp4")
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--hold", type=int, default=12,
                        help="repeat each sampled mission state for this many video frames")
    args = parser.parse_args()
    build(args.log, args.out, args.stride, args.fps, args.hold)
    print(args.out)


if __name__ == "__main__":
    main()
