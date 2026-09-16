"""Deterministic Python-only UAV-X simulation runner."""
from __future__ import annotations
import argparse, json, uuid
from pathlib import Path
import numpy as np
from ..core.models import GCS, PoI, ScenarioConfig, UAV, battery_step, enforce_separation
from ..core.connectivity import ConnectivityGraph
from ..core.task_allocation import HeuristicController
from ..interfaces.log_schema import LogEnvelope, write_jsonl
from ..metrics.mission_metrics import summarize

def default_world(cfg: ScenarioConfig):
    gcs = GCS(np.array([40.0, 40.0, 20.0]))
    uavs = [UAV(f"UAV-{i+1:02d}", np.array([45.0 + i*28, 50.0 + (i%2)*35, 65.0])) for i in range(6)]
    pois = [PoI(f"POI-{i+1:02d}", np.array([130 + (i%3)*100, 130 + (i//3)*110, 0.0]), (i % 5) + 1) for i in range(8)]
    return gcs, uavs, pois

def run(cfg: ScenarioConfig, out_path: str | None = None,
        policy: str = "auto", checkpoint: str | None = None) -> list[dict]:
    rng = np.random.default_rng(cfg.seed); run_id = f"seed-{cfg.seed}"
    gcs, uavs, pois = default_world(cfg); graph = ConnectivityGraph(gcs, cfg.radio_range_m, cfg.packet_loss, rng)
    controller = HeuristicController(cfg, policy=policy, checkpoint=checkpoint); records = [LogEnvelope(run_id, "manifest", 0, 0.0, payload={"scenario": "baseline", "seed": cfg.seed, "dt_s": cfg.dt, "arena_m": cfg.arena_m, "gcs_position_m": gcs.position.tolist(), "policy": policy, "ccpl": controller.ccpl.backend_info()}).to_dict()]
    try:
      for tick in range(cfg.ticks):
        t = tick * cfg.dt
        outage = cfg.outage_start_s is not None and cfg.outage_start_s <= t < cfg.outage_start_s + cfg.outage_duration_s
        if cfg.failure_time_s is not None and t >= cfg.failure_time_s and not any(u.failed for u in uavs):
            uavs[-1].failed = True
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "UAV_FAILURE", "actor_id": uavs[-1].uid, "related_id": None, "details": {}, "cause": "scenario disturbance"}).to_dict())
        if cfg.emergency_time_s is not None and abs(t - cfg.emergency_time_s) < cfg.dt / 2:
            p = PoI("POI-EMERGENCY", np.array([410.0, 390.0, 0.0]), 1); pois.append(p)
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "EMERGENCY_POI", "actor_id": None, "related_id": p.pid, "details": {"priority": 1}, "cause": "new disaster report"}).to_dict())
        graph.update(uavs, outage=outage)
        for event in controller.assign(uavs, pois, gcs, graph, tick):
            records.append(LogEnvelope(run_id, "event", tick, t, payload=event).to_dict())
        controller.move(uavs, pois, gcs, graph, cfg.dt)
        if enforce_separation(uavs, cfg.min_separation_m, cfg.arena_m):
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "SAFETY_OVERRIDE", "actor_id": None, "related_id": None, "details": {"minimum_separation_m": cfg.min_separation_m}, "cause": "collision"}).to_dict())
        for u in uavs:
            if u.failed: continue
            battery_step(u, cfg.dt)
            if u.role == "SURVEY" and u.task_id:
                p = next(x for x in pois if x.pid == u.task_id)
                if u.distance_to(p.position + np.array([0,0,50])) < 4:
                    p.survey_progress = min(1.0, p.survey_progress + 0.15)
                    if p.survey_progress >= 1.0: p.status = "SURVEYED"
            if u.role == "RETURN" and u.distance_to(gcs.position) < 5:
                u.role, u.mode, u.battery_pct = "CHARGE", "CHARGING", min(100.0, u.battery_pct + 0.7)
            elif u.role == "CHARGE":
                u.battery_pct = min(100.0, u.battery_pct + 0.7)
                if u.battery_pct >= 95: u.role, u.mode = "STANDBY", "CONNECTED"
        graph.update(uavs, outage=outage)
        tick_uavs = [{"id": u.uid, "position_m": [round(float(x), 4) for x in u.position], "velocity_mps": [round(float(x),4) for x in u.velocity], "battery_pct": round(u.battery_pct, 3), "role": u.role, "task_id": u.task_id, "mode": u.mode, "route_to_gcs": graph.route(u.uid), "hop_count": max(0, len(graph.route(u.uid))-1), "gcs_reachable": graph.reachable(u.uid), "link_quality": round(graph.quality(u.uid), 4), "failed": u.failed} for u in uavs]
        tick_pois = [{"id": p.pid, "position_m": p.position.tolist(), "priority": p.priority, "status": p.status, "assigned_uav_id": p.assigned_uav_id, "survey_progress": round(p.survey_progress, 4)} for p in pois]
        links = [{"source_id": l.source_id, "target_id": l.target_id, "available": l.available, "latency_ms": round(l.latency_ms, 3), "packet_loss": round(l.packet_loss, 4), "bandwidth_kbps": round(l.bandwidth_kbps, 3), "line_of_sight": l.line_of_sight} for l in graph.links]
        records.append(LogEnvelope(run_id, "tick", tick, t, payload={"uavs": tick_uavs, "pois": tick_pois, "links": links}).to_dict())
    finally:
        controller.ccpl.close()
    records.append(LogEnvelope(run_id, "summary", cfg.ticks, cfg.duration_s, payload=summarize(records)).to_dict())
    if out_path: write_jsonl(out_path, records)
    return records

def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--out", default="uav_x_run.jsonl"); parser.add_argument("--seed", type=int, default=None); parser.add_argument("--duration", type=float, default=None); parser.add_argument("--scenario", help="YAML scenario path"); parser.add_argument("--policy", choices=("auto", "heuristic", "ccpl"), default="auto"); parser.add_argument("--checkpoint")
    args = parser.parse_args(); values = {}
    if args.scenario:
        import yaml
        with open(args.scenario, encoding="utf-8") as handle: values.update(yaml.safe_load(handle) or {})
    if args.seed is not None: values["seed"] = args.seed
    if args.duration is not None: values["duration_s"] = args.duration
    records = run(ScenarioConfig(**values), args.out, policy=args.policy, checkpoint=args.checkpoint); print(json.dumps(records[-1], indent=2))

if __name__ == "__main__": main()
