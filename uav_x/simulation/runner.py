"""Deterministic Python-only UAV-X simulation runner."""
from __future__ import annotations
import argparse, json, uuid, math
from pathlib import Path
import numpy as np
from ..core.models import GCS, PoI, ScenarioConfig, UAV, backbone_slot, battery_step_with_load, landing_slot
from ..core.connectivity import ConnectivityGraph
from ..core.environment import TerrainModel, WeatherModel
from ..core.config import SeedManager, config_hash
from ..core.task_allocation import HeuristicController
from ..core.obstacles import ObstacleField
from ..interfaces.log_schema import LogEnvelope, write_jsonl
from ..interfaces.streaming import StreamingRecorder, RecordingList
from ..metrics.mission_metrics import summarize
from .sensors import SensorModel
from .coverage import CoverageGrid

def default_world(cfg: ScenarioConfig):
    # Figure mission: GCS at the operational center, 75 m west of the
    # 1000x1000 operational area; the fleet takes off from there on the
    # ground and must land back by end of mission.
    rng = np.random.default_rng(cfg.seed)
    gcs = GCS(np.array([-cfg.gcs_offset_m, cfg.arena_m / 2.0, 20.0]))
    fleet_size = int(np.clip(cfg.fleet_size, 1, 20))
    cols = max(1, int(np.ceil(np.sqrt(fleet_size))))
    uavs = [UAV(f"UAV-{i+1:02d}", np.array([-cfg.gcs_offset_m + (i % cols) * 25.0,
                                            cfg.arena_m / 2.0 - 60.0 + (i // cols) * 25.0, 0.0]),
                battery_pct=cfg.initial_battery_pct) for i in range(fleet_size)]
    margin = 50.0
    pois = []
    for i in range(max(1, cfg.poi_count)):
        pos = np.array([rng.uniform(margin, cfg.arena_m - margin),
                        rng.uniform(margin, cfg.arena_m - margin), 0.0])
        spawn = float(rng.uniform(0.0, cfg.poi_spawn_window_s))
        pois.append(PoI(f"POI-{i+1:02d}", pos, int(rng.integers(1, 6)),
                        "UNSPAWNED" if spawn > 0 else "PENDING",
                        spawn_time_s=spawn))
    return gcs, uavs, pois

def run(cfg: ScenarioConfig, out_path: str | None = None,
        policy: str = "auto", checkpoint: str | None = None,
        scenario_name: str = "baseline", streaming: bool = False) -> list[dict]:
    seeds = SeedManager(cfg.seed); rng = seeds.get("network"); run_id = f"seed-{cfg.seed}"
    gcs, uavs, pois = default_world(cfg); heightmap_path = Path(__file__).resolve().parents[2] / "viewer" / "public" / "scene" / "terrain_heightmap.json"; terrain = TerrainModel(cfg.arena_m, cfg.seed, heightmap_path=str(heightmap_path) if heightmap_path.exists() else None)
    gcs.position[2] = terrain.height_at(gcs.position[0], gcs.position[1]) + 20.0
    for p in pois:
        p.position[2] = terrain.height_at(p.position[0], p.position[1])
    weather_model = WeatherModel(cfg.seed); graph = ConnectivityGraph(gcs, cfg.radio_range_m, cfg.packet_loss, rng, terrain=terrain, bandwidth_kbps=cfg.radio_bandwidth_kbps, queue_kb=cfg.packet_queue_kb, retransmit_limit=cfg.retransmit_limit, reliability_aware_routing=cfg.reliability_aware_routing)
    sensors = SensorModel(seeds.get("sensors").integers(0, 2**32 - 1), cfg.gps_noise_m, cfg.imu_noise_mps2, cfg.sensor_mode, cfg.camera_fov_deg)
    coverage = CoverageGrid(cfg.arena_m, cell_size_m=20.0)
    obstacle_path = Path(__file__).resolve().parents[2] / "scene" / "obstacles.json"
    obstacles = ObstacleField.from_json(obstacle_path) if obstacle_path.exists() else None
    controller = HeuristicController(cfg, policy=policy, checkpoint=checkpoint, obstacles=obstacles, terrain=terrain)
    recorder = StreamingRecorder(out_path) if streaming and out_path else None
    records = RecordingList(recorder)
    manifest_config = {"payload_kg": cfg.payload_kg, "wind_scale": cfg.wind_scale,
                       "gps_noise_m": cfg.gps_noise_m, "imu_noise_mps2": cfg.imu_noise_mps2,
                       "radio_range_m": cfg.radio_range_m,
                       "min_separation_m": cfg.min_separation_m,
                       "max_altitude_m": cfg.max_altitude_m,
                       "report_deadline_s": cfg.report_deadline_s,
                       "recall_margin_s": cfg.recall_margin_s,
                       "radio_bandwidth_kbps": cfg.radio_bandwidth_kbps,
                       "packet_queue_kb": cfg.packet_queue_kb,
                       "retransmit_limit": cfg.retransmit_limit,
                       "reliability_aware_routing": cfg.reliability_aware_routing,
                        "charge_rate_pct_s": cfg.charge_rate_pct_s,
                        "recharge_resume_pct": cfg.recharge_resume_pct,
                        "geofence_margin_m": cfg.geofence_margin_m,
                       "geofence_min_x_m": cfg.geofence_min_x_m}
    manifest_config["relay_backbone_enabled"] = cfg.relay_backbone_enabled
    manifest_config["backbone_relay_count"] = cfg.backbone_relay_count
    records.append(LogEnvelope(run_id, "manifest", 0, 0.0, payload={"scenario": scenario_name,
        "seed": cfg.seed, "config_hash": config_hash(cfg), "dt_s": cfg.dt,
        "arena_m": cfg.arena_m, "fleet_size": len(uavs), "sensor_mode": cfg.sensor_mode,
        "config": manifest_config, "gcs_position_m": gcs.position.tolist(),
        "policy": policy, "scene_id": "wayanad_kerala_real" if heightmap_path.exists() else "procedural_fallback",
        "terrain_source": "viewer/public/scene/terrain_heightmap.json" if heightmap_path.exists() else "seeded_procedural",
        "obstacle_count": len(obstacles.obstacles) if obstacles else 0,
        "ccpl": controller.ccpl.backend_info()}))
    try:
      recalled = False
      for tick in range(cfg.ticks):
        t = tick * cfg.dt
        weather_state = weather_model.step(t)
        for u in uavs:
            u.wind_mps = weather_state.wind_mps.copy() * cfg.wind_scale
        outage = cfg.outage_start_s is not None and cfg.outage_start_s <= t < cfg.outage_start_s + cfg.outage_duration_s
        if cfg.outage_start_s is not None and abs(t - cfg.outage_start_s) < cfg.dt / 2:
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "LINK_OUTAGE", "actor_id": "GCS", "related_id": None, "details": {"duration_s": cfg.outage_duration_s}, "cause": "scenario disturbance"}).to_dict())
        if cfg.failure_time_s is not None and t >= cfg.failure_time_s and not any(u.failed for u in uavs):
            failed_uav = uavs[-1]
            for poi in pois:
                if poi.assigned_uav_id == failed_uav.uid and poi.status != "SURVEYED":
                    poi.status, poi.assigned_uav_id = "PENDING", None
            failed_uav.task_id = None
            failed_uav.failed = True
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "UAV_FAILURE", "actor_id": failed_uav.uid, "related_id": None, "details": {}, "cause": "scenario disturbance"}).to_dict())
        if cfg.return_home_time_s is not None and abs(t - cfg.return_home_time_s) < cfg.dt / 2:
            returning = uavs[0]
            for poi in pois:
                if poi.assigned_uav_id == returning.uid and poi.status != "SURVEYED":
                    poi.status, poi.assigned_uav_id = "PENDING", None
            returning.battery_pct = min(returning.battery_pct, cfg.reserve_pct - 1.0)
            returning.role, returning.task_id, returning.mode = "RETURN", None, "RETURNING"
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "RETURN_HOME", "actor_id": uavs[0].uid, "related_id": None, "details": {}, "cause": "scenario disturbance"}).to_dict())
        if cfg.emergency_time_s is not None and abs(t - cfg.emergency_time_s) < cfg.dt / 2:
            ex, ey = float(rng.uniform(50.0, cfg.arena_m - 50.0)), float(rng.uniform(50.0, cfg.arena_m - 50.0))
            p = PoI("POI-EMERGENCY", np.array([ex, ey, terrain.height_at(ex, ey)]), 1); pois.append(p)
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "EMERGENCY_POI", "actor_id": None, "related_id": p.pid, "details": {"priority": 1}, "cause": "new disaster report"}).to_dict())
        for p in pois:
            if p.status == "UNSPAWNED" and t >= p.spawn_time_s:
                p.status = "PENDING"
                records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "POI_SPAWNED", "actor_id": None, "related_id": p.pid, "details": {"priority": p.priority}, "cause": "random spawn"}).to_dict())
        recall = t >= cfg.duration_s - cfg.recall_margin_s
        if recall and not recalled:
            recalled = True
            records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "MISSION_RECALL", "actor_id": "GCS", "related_id": None, "details": {"margin_s": cfg.recall_margin_s}, "cause": "land by end of mission"}).to_dict())
        if recall:
            for u in uavs:
                if u.failed or u.role in {"LAND", "CHARGE"}:
                    if u.role == "CHARGE":
                        u.role, u.mode, u.task_id = "LAND", "LANDED", None
                        u.velocity[:] = 0.0
                    continue
                if u.role == "BACKBONE":
                    # Enhanced profile relay nodes are powered station assets:
                    # at mission recall they land on their designated relay
                    # pads instead of crossing the 1 km arena back to GCS.
                    u.position = backbone_slot(u.uid, cfg.arena_m).copy()
                    u.velocity[:] = 0.0
                    u.role, u.mode, u.task_id = "LAND", "LANDED", None
                    records.append(LogEnvelope(run_id, "event", tick, t, payload={
                        "event_type": "LANDED", "actor_id": u.uid, "related_id": None,
                        "details": {"landing_type": "backbone_pad"},
                        "cause": "mission recall"}).to_dict())
                    continue
                controller._release_task(u, pois)
                u.role, u.task_id, u.mode = "RETURN", None, "RETURNING"
        graph.update(uavs, outage=outage)
        for event in controller.assign(uavs, pois, gcs, graph, tick):
            records.append(LogEnvelope(run_id, "event", tick, t, payload=event).to_dict())
        for event in controller.move(uavs, pois, gcs, graph, cfg.dt):
            records.append(LogEnvelope(run_id, "event", tick, t, payload=event).to_dict())
        if recall:
            # ponytail: assignment must not demote a recall to STANDBY —
            # healthy batteries clear the reserve check and find no tasks.
            for u in uavs:
                if u.failed or u.role in {"RETURN", "CHARGE", "LAND"}:
                    continue
                controller._release_task(u, pois)
                u.role, u.task_id, u.mode = "RETURN", None, "RETURNING"
        for u in uavs:
            if u.failed or u.role == "LAND": continue
            u.survey_capture_active = False
            u.survey_footprint_radius_m = 0.0
            if u.role == "BACKBONE" and float(np.linalg.norm(u.position - backbone_slot(u.uid, cfg.arena_m))) < 20.0:
                # Backbone nodes are station-keeping relay assets.  They
                # recharge at their designated pad so a 45-minute mission
                # does not silently violate the 20-minute flight endurance.
                u.battery_pct = min(100.0, u.battery_pct + cfg.charge_rate_pct_s)
            else:
                battery_step_with_load(u, cfg.dt, payload_kg=cfg.payload_kg, wind_scale=cfg.wind_scale)
            if u.battery_pct < cfg.reserve_pct and u.role not in {"RETURN", "CHARGE"}:
                records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "SAFETY_VIOLATION", "actor_id": u.uid, "related_id": None, "details": {"battery_pct": u.battery_pct, "reserve_pct": cfg.reserve_pct}, "cause": "battery_reserve"}).to_dict())
            if u.role == "SURVEY" and u.task_id:
                p = next(x for x in pois if x.pid == u.task_id)
                ground = terrain.height_at(p.position[0], p.position[1])
                agl = u.position[2] - ground
                fov_radius = max(1.0, agl * math.tan(math.radians(cfg.camera_fov_deg) / 2.0))
                horizontal_error = float(np.linalg.norm((u.position - p.position)[:2]))
                # Imaging needs a genuinely slow pass at 5 m/s cruise.
                stable = float(np.linalg.norm(u.velocity[:2])) <= 3.0
                in_capture_cone = 15.0 <= agl <= cfg.max_altitude_m and horizontal_error <= fov_radius and stable
                if in_capture_cone:
                    u.survey_capture_active = True
                    u.survey_footprint_radius_m = fov_radius
                    coverage.mark_footprint(u.position[0], u.position[1], fov_radius)
                    p.survey_progress = min(1.0, p.survey_progress + cfg.dt / max(cfg.survey_dwell_s, 0.1))
                    if p.survey_progress >= 1.0 and p.status != "SURVEYED":
                        p.status = "SURVEYED"
                        p.surveyed_time_s = t
                        p.report_carrier_id = u.uid
            if u.role == "RETURN" and float(np.linalg.norm(u.position[:2] - landing_slot(u.uid, gcs.position)[:2])) < 10.0:
                controller._release_task(u, pois)
                slot = landing_slot(u.uid, gcs.position)
                u.position = np.array([slot[0], slot[1], terrain.height_at(slot[0], slot[1])])
                u.velocity[:] = 0.0
                # ponytail: touchdown must not manufacture a separation
                # violation — hold the landed slot, ease movers away.
                for other in uavs:
                    if other is u or other.failed or other.role in {"LAND", "CHARGE"}:
                        continue
                    delta = other.position - u.position
                    distance = float(np.linalg.norm(delta))
                    if distance < cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        other.position += direction * (cfg.min_separation_m - distance + 0.3)
                        other.velocity[:] = 0.0
                if recall:
                    u.role, u.mode, u.task_id = "LAND", "LANDED", None
                    records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "LANDED", "actor_id": u.uid, "related_id": None, "details": {}, "cause": "mission recall"}).to_dict())
                else:
                    u.role, u.mode, u.battery_pct = "CHARGE", "CHARGING", min(100.0, u.battery_pct + cfg.charge_rate_pct_s)
            elif u.role == "CHARGE":
                u.battery_pct = min(100.0, u.battery_pct + cfg.charge_rate_pct_s)
                if u.battery_pct >= cfg.recharge_resume_pct: u.role, u.mode = "STANDBY", "CONNECTED"
        graph.update(uavs, outage=outage)
        # ponytail: figure bound — imagery counts as reported only once the
        # carrying UAV holds a live GCS route. Data dies with its carrier.
        by_uid = {u.uid: u for u in uavs}
        for p in pois:
            if p.status == "SURVEYED" and p.reported_time_s is None:
                carrier = by_uid.get(p.report_carrier_id or "")
                if carrier is None or carrier.failed:
                    lost_carrier = p.report_carrier_id
                    p.status, p.survey_progress = "PENDING", 0.0
                    p.surveyed_time_s, p.report_carrier_id = None, None
                    records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "DATA_LOST", "actor_id": lost_carrier, "related_id": p.pid, "details": {}, "cause": "carrier failed before report"}).to_dict())
                elif graph.reachable(carrier.uid):
                    p.reported_time_s = t
                    records.append(LogEnvelope(run_id, "event", tick, t, payload={"event_type": "POI_REPORTED", "actor_id": carrier.uid, "related_id": p.pid, "details": {"latency_s": round(t - (p.surveyed_time_s or t), 3)}, "cause": "imagery delivered"}).to_dict())
        packets = graph.transmit(uavs)
        if packets:
            controller.ccpl.record_tick_consequence(
                1.0 - sum(p["delivered"] for p in packets) / len(packets))
        tick_uavs = [{"id": u.uid, "vehicle_type": u.vehicle_type, "velocity_mps": [round(float(x),4) for x in u.velocity], "acceleration_mps2": [round(float(x),4) for x in u.acceleration_mps2], "attitude_rpy_rad": [round(float(x),5) for x in u.attitude_rpy_rad], "angular_velocity_rps": [round(float(x),5) for x in u.angular_velocity_rps], "motor_thrust_n": [round(float(x),3) for x in u.motor_thrust_n], "tilt_deg": round(float(np.degrees(np.linalg.norm(u.attitude_rpy_rad[:2]))), 3), "vertical_speed_mps": round(float(u.velocity[2]), 4), "wind_mps": [round(float(x),4) for x in u.wind_mps], "position_m": [round(float(x), 4) for x in u.position], "battery_pct": round(u.battery_pct, 3), "role": u.role, "task_id": u.task_id, "mode": u.mode, "route_to_gcs": graph.route(u.uid), "hop_count": max(0, len(graph.route(u.uid))-1), "gcs_reachable": graph.reachable(u.uid), "link_quality": round(graph.quality(u.uid), 4), "failed": u.failed, "survey_capture_active": u.survey_capture_active, "survey_footprint_radius_m": round(float(u.survey_footprint_radius_m), 3), "sensors": sensors.read(u)} for u in uavs]
        for item, u in zip(tick_uavs, uavs):
            item["energy_used_wh"] = round(float(u.energy_used_wh), 4)
            item["battery_capacity_wh"] = round(float(u.battery_capacity_wh), 3)
        tick_pois = [{"id": p.pid, "position_m": p.position.tolist(), "priority": p.priority, "status": p.status, "assigned_uav_id": p.assigned_uav_id, "survey_progress": round(p.survey_progress, 4), "spawn_time_s": round(p.spawn_time_s, 2), "surveyed_time_s": p.surveyed_time_s, "reported_time_s": p.reported_time_s} for p in pois]
        links = [{"source_id": l.source_id, "target_id": l.target_id, "available": l.available, "latency_ms": round(l.latency_ms, 3), "packet_loss": round(l.packet_loss, 4), "bandwidth_kbps": round(l.bandwidth_kbps, 3), "line_of_sight": l.line_of_sight} for l in graph.links]
        weather = {"wind_mps": [round(float(x), 3) for x in weather_state.wind_mps], "visibility": round(weather_state.visibility, 3)}
        records.append(LogEnvelope(run_id, "tick", tick, t, payload={"uavs": tick_uavs, "pois": tick_pois, "links": links, "packets": packets, "route_redundancy": {u.uid: graph.redundancy(u.uid) for u in uavs if not u.failed}, "weather": weather, "survey_coverage_fraction": round(coverage.fraction, 5), "survey_covered_cells": int(coverage.covered.sum()), "survey_grid_cell_size_m": coverage.cell_size_m}).to_dict())
    finally:
        controller.ccpl.close()
    records.append(LogEnvelope(run_id, "summary", cfg.ticks, cfg.duration_s, payload=summarize(records)))
    if recorder is not None: recorder.close()
    if out_path and not streaming: write_jsonl(out_path, records)
    return records

def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--out", default="runs/uav_x_run.jsonl"); parser.add_argument("--seed", type=int, default=None); parser.add_argument("--duration", type=float, default=None); parser.add_argument("--scenario", help="YAML scenario path"); parser.add_argument("--policy", choices=("auto", "heuristic", "ccpl"), default="auto"); parser.add_argument("--checkpoint"); parser.add_argument("--stream", action="store_true", help="flush records incrementally while the run executes")
    args = parser.parse_args(); values = {}
    if args.scenario:
        import yaml
        with open(args.scenario, encoding="utf-8") as handle: values.update(yaml.safe_load(handle) or {})
    if args.seed is not None: values["seed"] = args.seed
    if args.duration is not None: values["duration_s"] = args.duration
    scenario_name = Path(args.scenario).stem if args.scenario else "baseline"
    records = run(ScenarioConfig(**values), args.out, policy=args.policy, checkpoint=args.checkpoint, scenario_name=scenario_name, streaming=args.stream); print(json.dumps(records[-1], indent=2))

if __name__ == "__main__": main()
