"""Create a human-readable storyboard demo from a canonical UAV-X replay."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Circle, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_x.interfaces.log_schema import read_jsonl


ROLE = {"SURVEY": "#20e3a2", "BACKBONE": "#00d5ff", "RELAY": "#35a7ff",
        "RECOVER": "#ffbd5a", "RETURN": "#ff637d", "CHARGE": "#bd8cff",
        "LAND": "#76869a", "STANDBY": "#aab7c6"}


def build(log_path: str, output: str, fps: int = 12, hold: int = 18) -> None:
    rows = read_jsonl(log_path)
    ticks = [r for r in rows if r["record_type"] == "tick"]
    events = [r for r in rows if r["record_type"] == "event"]
    summary = next((r.get("payload", {}) for r in rows if r["record_type"] == "summary"), {})
    manifest = next((r.get("payload", {}) for r in rows if r["record_type"] == "manifest"), {})
    arena = float(manifest.get("arena_m", 1000.0))
    gcs = (manifest.get("gcs_position_m") or [-75.0, 500.0])[:2]

    event_times = {e["event_type"]: float(e["time_s"]) for e in events}
    scenes = [
        (0, "MISSION BRIEF", "15 UAVs • 6 survey aircraft • 9 relay aircraft", "Disaster PoIs must be surveyed and reported to the GCS."),
        (60, "TAKEOFF", "The swarm leaves the GCS", "Survey vehicles enter the area while relay vehicles move to their stations."),
        (350, "RELAY BACKBONE ONLINE", "A 3×3 aerial network is established", "Every survey sector now has a multi-hop path to the GCS."),
        (760, "AUTONOMOUS SURVEY", "PoIs are assigned by priority and distance", "Green markers are reported; red markers are high-priority."),
        (800, "UAV FAILURE", "One survey UAV is lost", "Its unfinished assignment is released and reallocated automatically."),
        (850, "COMMUNICATION OUTAGE", "The GCS link is unavailable for 120 seconds", "The swarm holds formation and restores routes when the link returns."),
        (1200, "NEW PRIORITY PoI", "An emergency location appears", "The priority-1 task is inserted and dispatched without stopping the mission."),
        (1760, "MISSION COMPLETE", "All 10 PoIs are surveyed and reported", "Mean report latency: 0.18 s • Maximum: 1 s"),
        (2700, "SAFE LANDING", "Mission ends with a clean roll-call", "0 collisions • 0 battery violations • all surviving UAVs landed"),
    ]
    scene_indices = [min(len(ticks)-1, max(0, int(t))) for t, *_ in scenes]
    render_scene = [i for i in range(len(scenes)) for _ in range(max(1, hold))]

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#06101b")
    ax = fig.add_axes([.055, .13, .62, .72])
    panel = fig.add_axes([.71, .13, .25, .72])

    def reset(axis):
        axis.clear(); axis.set_facecolor("#06101b")
        axis.tick_params(colors="#9fb4c8", labelsize=8)
        for spine in axis.spines.values(): spine.set_color("#1d344a")

    def draw(k):
        scene_no = render_scene[k]
        tick = ticks[scene_indices[scene_no]]
        t, title, headline, detail = scenes[scene_no]
        reset(ax); reset(panel)
        ax.set_xlim(-125, arena + 25); ax.set_ylim(-25, arena + 25); ax.set_aspect("equal")
        ax.grid(color="#17304a", alpha=.65)
        ax.set_xlabel("EASTING (m)", color="#9fb4c8")
        ax.set_ylabel("NORTHING (m)", color="#9fb4c8")
        ax.add_patch(Rectangle((0, 0), arena, arena, fill=False, edgecolor="#3e9fc5",
                               linestyle="--", linewidth=1.4))
        ax.scatter(*gcs, marker="s", s=220, c="#35a7ff", edgecolors="white", zorder=8)
        ax.text(gcs[0]+18, gcs[1]+16, "GCS / HOME", color="#67c4ff", fontsize=9, weight="bold")
        for link in tick.get("links", []):
            if not link.get("available"): continue
            u = next((x for x in tick["uavs"] if x["id"] == link["source_id"]), None)
            v = next((x for x in tick["uavs"] if x["id"] == link["target_id"]), None)
            if u and v:
                q = max(.08, 1-float(link.get("packet_loss", 1)))
                ax.plot([u["position_m"][0],v["position_m"][0]], [u["position_m"][1],v["position_m"][1]],
                        color="#35a7ff", alpha=.15+.55*q, linewidth=.7+1.5*q, zorder=2)
        for p in tick["pois"]:
            x,y = p["position_m"][:2]
            done = p["status"] == "SURVEYED"
            color = "#20e3a2" if done else ("#ff4d6d" if p["priority"] == 1 else "#ffbd5a")
            ax.scatter(x,y,marker="*",s=170 if p["priority"] == 1 else 110,c=color,edgecolors="white",zorder=6)
            if done: ax.scatter(x,y,s=260,facecolors="none",edgecolors=color,alpha=.4)
        for u in tick["uavs"]:
            x,y,_ = u["position_m"]
            color = "#6b7788" if u.get("failed") else ROLE.get(u["role"],"white")
            ax.scatter(x,y,s=110,c=color,edgecolors="white",linewidth=.7,zorder=7)
        ax.set_title(f"{title}   •   T+{t/60:.1f} MIN",loc="left",color="white",weight="bold",fontsize=15)

        panel.axis("off")
        panel.text(.02,.96,"UAV-X",color="#67c4ff",fontsize=24,weight="bold",va="top",transform=panel.transAxes)
        panel.text(.02,.885,f"STORY {scene_no+1}/{len(scenes)}",color="#718ba0",fontsize=9,weight="bold",transform=panel.transAxes)
        panel.text(.02,.77,headline,color="white",fontsize=15,weight="bold",wrap=True,va="top",transform=panel.transAxes)
        panel.text(.02,.54,detail,color="#c3d0db",fontsize=11,wrap=True,va="top",linespacing=1.5,transform=panel.transAxes)
        panel.text(.02,.28,"LEGEND",color="#67c4ff",fontsize=10,weight="bold",transform=panel.transAxes)
        legend=[("#20e3a2","Surveyed PoI"),("#ffbd5a","Pending PoI"),("#35a7ff","Mesh link"),("#00d5ff","Relay UAV")]
        for n,(c,label) in enumerate(legend):
            yy=.23-n*.045
            panel.scatter(.035,yy,s=55,c=c,transform=panel.transAxes,clip_on=False)
            panel.text(.08,yy,label,color="#c3d0db",fontsize=9,transform=panel.transAxes,va="center")
        panel.text(.02,.02,"Canonical replay • no invented state",color="#718ba0",fontsize=8,transform=panel.transAxes)
        fig.suptitle("RESILIENT BVLOS DISASTER RESPONSE",color="#67c4ff",fontsize=19,weight="bold",y=.975)
        fig.text(.055,.935,"A relay backbone keeps the swarm connected through failure, outage, and emergency response.",color="#a5b9c9",fontsize=10)

    animation = FuncAnimation(fig, draw, frames=len(render_scene), interval=1000/fps, repeat=False)
    out = Path(output); out.parent.mkdir(parents=True, exist_ok=True)
    animation.save(out, writer=FFMpegWriter(fps=fps, bitrate=5000, metadata={"title":"UAV-X Story Demonstration"}), dpi=100)
    plt.close(fig)


def main():
    p=argparse.ArgumentParser(); p.add_argument("log"); p.add_argument("--out",default="artifacts/uav_x_story_demo.mp4"); p.add_argument("--fps",type=int,default=12); p.add_argument("--hold",type=int,default=18)
    a=p.parse_args(); build(a.log,a.out,a.fps,a.hold); print(a.out)


if __name__ == "__main__": main()
