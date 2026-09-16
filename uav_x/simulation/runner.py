"""Deterministic Python-only UAV-X simulation runner."""
from __future__ import annotations
import argparse, json, uuid, math
from pathlib import Path
import numpy as np
from ..core.models import GCS, PoI, ScenarioConfig, UAV, battery_step
from ..core.connectivity import ConnectivityGraph
from ..core.environment import TerrainModel, WeatherModel
from ..core.config import SeedManager, config_hash
from ..core.task_allocation import HeuristicController
from ..core.obstacles import ObstacleField
from ..interfaces.log_schema import LogEnvelope, write_jsonl
from ..metrics.mission_metrics import summarize

def default_world(cfg: ScenarioConfig):
    # GCS mast sits on the terrain at ~23 m elevation with a 20 m antenna.
    gcs = GCS(np.array([40.0, 40.0, 45.0]))
    # Start above the real Wayanad terrain envelope (0–75 m) with a safe
    # 35 m terrain clearance, so the baseline replay begins airborne.
    uavs = [UAV(f"UAV-{i+1:02d}", np.array([45.0 + i*28, 50.0 + (i%2)*35, 110.0]), battery_pct=cfg.initial_battery_pct) for i in range(6)]
    pois = [PoI(f"POI-{i+1:02d}", np.array([130 + (i%3)*100, 130 + (i//3)*110, 0.0]), (i % 5) + 1) for i in range(8)]
    return gcs, uavs, pois

def run(cfg: ScenarioConfig, out_path: str | None = None,
        policy: str = "auto", checkpoint: str | None = None) -> list[dict]:
    rng = SeedManager(cfg.seed).get("network"); run_id = f"seed-{cfg.seed}"
    gcs, uavs, pois = default_world(cfg); heightmap_path = Path(__file__).resolve().parents[2] / "viewer" / "public" / "scene" / "terrain_heightmap.json"; terrain = TerrainModel(cfg.arena_m, cfg.seed, heightmap_path=str(heightmap_path) if heightmap_path.exists() else None); weather_model = WeatherModel(cfg.seed); graph = ConnectivityGraph(gcs, cfg.radio_range_m, cfg.packet_loss, rng, terrain=terrain)
    obstacle_path = Path(__file__).resolve().parents[2] / "scene" / "obstacles.json"
    obstacles = ObstacleField.from_json(obstacle_path) if obstacle_path.exists() else None
    controller = HeuristicController(cfg, policy=policy, checkpoint=checkpoint, obstacles=obstacles, terrain=terrain); records = [LogEnvelope(run_id, "manifest", 0, 0.0, payload={"scenario": "baseline", "seed": cfg.seed, "config_hash": config_hash(cfg), "dt_s": cfg.dt, "arena_m": cfg.arena_m, "gcs_position_m": gcs.position.tolist(), "policy": policy, "scene_id": "wayanad_kerala_real" if heightmap_path.exists() else "procedural_fallback", "terrain_source": "viewer/public/scene/terrain_heightmap.json" if heightmap_path.exists() else "seeded_procedural", "obstacle_count": len(obstacles.obstacles) if obstacles else 0, "ccpl": controller.ccpl.backend_info()}).to_dict()]
    try:
      for tick in range(cfg.ticks):
        t = tick * cfg.dt
        weather_state = weather_model.step(t)
        for u in uavs:
            u.wind_mps = weather_state.wind_mps.copy()
        outage = cfg.outage_start_s is not None and cfg.outage_start_s <= t < cfg.outage_start_s + cfg.outage_duration_s
        if cfg.outage_start_s is not None and abs(t - cfg.outage_start_s) < cfg.dt / 2:
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "LINK_OUTAGE", "actor_id": "GCS", "related_id": None, "details": {"duration_s": cfg.outage_duration_s}, "cause": "scenario disturbance"}).to_dict())
        if cfg.failure_time_s is not None and t >= cfg.failure_time_s and not any(u.failed for u in uavs):
            uavs[-1].failed = True
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "UAV_FAILURE", "actor_id": uavs[-1].uid, "related_id": None, "details": {}, "cause": "scenario disturbance"}).to_dict())
        if cfg.return_home_time_s is not None and abs(t - cfg.return_home_time_s) < cfg.dt / 2:
            uavs[0].battery_pct = min(uavs[0].battery_pct, cfg.reserve_pct - 1.0)
            uavs[0].role, uavs[0].task_id, uavs[0].mode = "RETURN", None, "RETURNING"
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "RETURN_HOME", "actor_id": uavs[0].uid, "related_id": None, "details": {}, "cause": "scenario disturbance"}).to_dict())
        if cfg.emergency_time_s is not None and abs(t - cfg.emergency_time_s) < cfg.dt / 2:
            p = PoI("POI-EMERGENCY", np.array([410.0, 390.0, 0.0]), 1); pois.append(p)
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "EMERGENCY_POI", "actor_id": None, "related_id": p.pid, "details": {"priority": 1}, "cause": "new disaster report"}).to_dict())
        graph.update(uavs, outage=outage)
        for event in controller.assign(uavs, pois, gcs, graph, tick):
            records.append(LogEnvelope(run_id, "event", tick, t, payload=event).to_dict())
        for event in controller.move(uavs, pois, gcs, graph, cfg.dt):
            records.append(LogEnvelope(run_id, "event", tick, t, payload=event).to_dict())
        for u in uavs:
            if u.failed: continue
            battery_step(u, cfg.dt)
            if u.battery_pct < cfg.reserve_pct and u.role not in {"RETURN", "CHARGE"}:
                records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "SAFETY_VIOLATION", "actor_id": u.uid, "related_id": None, "details": {"battery_pct": u.battery_pct, "reserve_pct": cfg.reserve_pct}, "cause": "battery_reserve"}).to_dict())
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
        packets = graph.transmit(uavs)
        if packets:
            controller.ccpl.record_tick_consequence(
                1.0 - sum(p["delivered"] for p in packets) / len(packets))
        tick_uavs = [{"id": u.uid, "vehicle_type": u.vehicle_type, "velocity_mps": [round(float(x),4) for x in u.velocity], "acceleration_mps2": [round(float(x),4) for x in u.acceleration_mps2], "attitude_rpy_rad": [round(float(x),5) for x in u.attitude_rpy_rad], "angular_velocity_rps": [round(float(x),5) for x in u.angular_velocity_rps], "motor_thrust_n": [round(float(x),3) for x in u.motor_thrust_n], "tilt_deg": round(float(np.degrees(np.linalg.norm(u.attitude_rpy_rad[:2]))), 3), "vertical_speed_mps": round(float(u.velocity[2]), 4), "wind_mps": [round(float(x),4) for x in u.wind_mps], "position_m": [round(float(x), 4) for x in u.position], "battery_pct": round(u.battery_pct, 3), "role": u.role, "task_id": u.task_id, "mode": u.mode, "route_to_gcs": graph.route(u.uid), "hop_count": max(0, len(graph.route(u.uid))-1), "gcs_reachable": graph.reachable(u.uid), "link_quality": round(graph.quality(u.uid), 4), "failed": u.failed} for u in uavs]
        tick_pois = [{"id": p.pid, "position_m": p.position.tolist(), "priority": p.priority, "status": p.status, "assigned_uav_id": p.assigned_uav_id, "survey_progress": round(p.survey_progress, 4)} for p in pois]
        links = [{"source_id": l.source_id, "target_id": l.target_id, "available": l.available, "latency_ms": round(l.latency_ms, 3), "packet_loss": round(l.packet_loss, 4), "bandwidth_kbps": round(l.bandwidth_kbps, 3), "line_of_sight": l.line_of_sight} for l in graph.links]
        weather = {"wind_mps": [round(float(x), 3) for x in weather_state.wind_mps], "visibility": round(weather_state.visibility, 3)}
        records.append(LogEnvelope(run_id, "tick", tick, t, payload={"uavs": tick_uavs, "pois": tick_pois, "links": links, "packets": packets, "route_redundancy": {u.uid: graph.redundancy(u.uid) for u in uavs if not u.failed}, "weather": weather}).to_dict())
    finally:
        controller.ccpl.close()
    records.append(LogEnvelope(run_id, "summary", cfg.ticks, cfg.duration_s, payload=summarize(records)).to_dict())
    if out_path: write_jsonl(out_path, records)
    return records

def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--out", default="runs/uav_x_run.jsonl"); parser.add_argument("--seed", type=int, default=None); parser.add_argument("--duration", type=float, default=None); parser.add_argument("--scenario", help="YAML scenario path"); parser.add_argument("--policy", choices=("auto", "heuristic", "ccpl"), default="auto"); parser.add_argument("--checkpoint")
    args = parser.parse_args(); values = {}
    if args.scenario:
        import yaml
        with open(args.scenario, encoding="utf-8") as handle: values.update(yaml.safe_load(handle) or {})
    if args.seed is not None: values["seed"] = args.seed
    if args.duration is not None: values["duration_s"] = args.duration
    records = run(ScenarioConfig(**values), args.out, policy=args.policy, checkpoint=args.checkpoint); print(json.dumps(records[-1], indent=2))

if __name__ == "__main__": main()
