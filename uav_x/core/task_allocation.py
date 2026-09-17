"""Deterministic decentralized-style heuristic controller."""
from __future__ import annotations
import numpy as np
from .models import GCS, PoI, ScenarioConfig, UAV, advance_dynamics, safe_destination
from .connectivity import ConnectivityGraph
from .ccpl_adapter import CCPLPolicy
from .obstacles import ObstacleField
from .environment import TerrainModel

class HeuristicController:
    def __init__(self, cfg: ScenarioConfig, policy: str = "auto",
                 checkpoint: str | None = None, obstacles: ObstacleField | None = None, terrain: TerrainModel | None = None):
        self.cfg = cfg
        self.obstacles = obstacles
        self.terrain = terrain
        self.cooldown: dict[str, int] = {}
        self.isolated_ticks: dict[str, int] = {}
        if policy not in {"auto", "heuristic", "ccpl"}:
            raise ValueError("policy must be auto, heuristic, or ccpl")
        self.ccpl = CCPLPolicy(seed=cfg.seed, checkpoint=checkpoint,
                               required=policy == "ccpl")
        if policy == "heuristic":
            self.ccpl.close()
            self.ccpl = CCPLPolicy(seed=cfg.seed, checkpoint=checkpoint,
                                   required=False)
            self.ccpl.backend = "heuristic-fallback"
            self.ccpl.agent = None

    def assign(self, uavs: list[UAV], pois: list[PoI], gcs: GCS, graph: ConnectivityGraph, tick: int) -> list[dict]:
        events = []
        for u in uavs:
            if u.failed:
                continue
            self.cooldown[u.uid] = max(0, self.cooldown.get(u.uid, 0) - 1)
            if u.role == "CHARGE":
                continue
            home_need = u.distance_to(gcs.position) / max(u.speed_mps, 1) * 0.12
            if u.battery_pct <= self.cfg.reserve_pct + home_need:
                if u.role != "RETURN": events.append(self._role(u, "RETURN", "battery reserve"))
                u.role, u.task_id, u.mode = "RETURN", None, "RETURNING"
                continue
            if not graph.reachable(u.uid):
                self.isolated_ticks[u.uid] = self.isolated_ticks.get(u.uid, 0) + 1
                if u.role not in {"RELAY", "RECOVER"}:
                    events.append(self._role(u, "RECOVER", "isolated route"))
                u.role, u.mode = "RECOVER", "ISOLATED"
                if u.task_id is not None:
                    p = next((x for x in pois if x.pid == u.task_id), None)
                    if p is not None and p.assigned_uav_id == u.uid and p.status != "SURVEYED":
                        p.status, p.assigned_uav_id = "PENDING", None
                u.task_id = None
                continue
            self.isolated_ticks[u.uid] = max(0, self.isolated_ticks.get(u.uid, 0) - 4)
            if u.mode == "ISOLATED": u.mode = "CONNECTED"
            if self._relay_needed(u, uavs, graph):
                if u.role != "RELAY": events.append(self._role(u, "RELAY", "route preservation"))
                u.role, u.task_id = "RELAY", None
                continue
            if self.cooldown[u.uid] > 0 and u.task_id:
                continue
            candidates = [p for p in pois if p.status in {"PENDING", "ASSIGNED"} and
                          (p.assigned_uav_id in {None, u.uid})]
            scored = [(self._score(u, p, gcs, graph), p) for p in candidates]
            scored = [(s, p) for s, p in scored if s > 0]
            if scored:
                # IDs may be numeric (POI-03) or semantic (POI-EMERGENCY).
                # Use the full stable string as a deterministic tie-breaker.
                _, poi = max(scored, key=lambda pair: (pair[0], pair[1].pid))
                if u.task_id != poi.pid:
                    events.append(self._role(u, "SURVEY", "highest feasible bid", poi.pid))
                u.role, u.task_id, u.mode = "SURVEY", poi.pid, "CONNECTED"
                poi.status, poi.assigned_uav_id = "ASSIGNED", u.uid
                self.cooldown[u.uid] = 4
            else:
                u.role, u.task_id = "STANDBY", None
        return events

    def move(self, uavs: list[UAV], pois: list[PoI], gcs: GCS, graph: ConnectivityGraph, dt: float) -> list[dict]:
        by_id = {p.pid: p for p in pois}
        events = []
        planned = {}
        for u in uavs:
            if u.failed: continue
            if u.role in {"RETURN", "CHARGE"}: destination = gcs.position
            elif u.role == "SURVEY" and u.task_id in by_id:
                poi = by_id[u.task_id]
                destination = poi.position.copy()
                agl = self.cfg.survey_altitude_agl_m
                if self.terrain is not None:
                    destination[2] = self.terrain.height_at(poi.position[0], poi.position[1]) + agl
                else:
                    destination[2] = poi.position[2] + agl
            elif u.role in {"RELAY", "RECOVER"}:
                if u.role == "RECOVER" and self.isolated_ticks.get(u.uid, 0) > 20:
                    destination = gcs.position
                else:
                    destination = self._relay_waypoint(u, uavs, gcs, graph)
            else: destination = u.position
            raw = np.asarray(destination, dtype=float)
            if self.terrain is not None:
                terrain_fix = self.terrain.terrain_clear(u.position, raw, clearance_m=14.0)
                if terrain_fix is not None:
                    raw, peak = terrain_fix
                    events.append({"event_type": "TERRAIN_AVOIDANCE", "actor_id": u.uid,
                                   "related_id": u.task_id, "details": {"peak_elevation_m": round(float(peak), 3), "clearance_m": 14.0},
                                   "cause": "predictive_terrain_intersection"})
            if self.obstacles is not None:
                detour = self.obstacles.detour(u.position, raw)
                if detour is not None:
                    raw, obstacle = detour
                    events.append({"event_type": "OBSTACLE_AVOIDANCE", "actor_id": u.uid,
                                   "related_id": u.task_id, "details": {"obstacle_id": obstacle.obstacle_id,
                                   "kind": obstacle.kind, "clearance_m": self.obstacles.clearance_m},
                                   "cause": "predictive_segment_intersection"})
            delta = raw - u.position; distance = float(np.linalg.norm(delta))
            step = min(distance, u.speed_mps * dt)
            predicted = u.position if distance < 1e-9 else u.position + delta / distance * step
            planned[u.uid] = (u, np.asarray(predicted, dtype=float), u.task_id)
        # Resolve conflicts in predicted positions before any vehicle moves.
        active = list(planned.values())
        for _ in range(10):
            for i, (a, ap, atask) in enumerate(active):
                for b, bp, btask in active[i + 1:]:
                    delta = ap - bp; distance = float(np.linalg.norm(delta))
                    if distance < self.cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        correction = direction * ((self.cfg.min_separation_m - distance) / 2.0 + 1.0)
                        ap += correction; bp -= correction
                        events.extend([{"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid, "related_id": atask, "details": {"minimum_separation_m": self.cfg.min_separation_m}, "cause": "separation"},
                                       {"event_type": "SAFETY_OVERRIDE", "actor_id": b.uid, "related_id": btask, "details": {"minimum_separation_m": self.cfg.min_separation_m}, "cause": "separation"}])
            for u, destination, task in active:
                before = destination.copy(); destination[:2] = np.clip(destination[:2], self.cfg.geofence_margin_m, self.cfg.arena_m - self.cfg.geofence_margin_m)
                if not np.allclose(before, destination):
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid, "related_id": task, "details": {}, "cause": "geofence"})
        for u, destination, _ in active:
            advance_dynamics(u, destination, dt)
        # The dynamics integrator can overshoot a planned point.  Apply a
        # second, state-level shield after integration so the recorded flight
        # trajectory—not only the planned waypoint—obeys separation and the
        # geofence constraints.
        moved = [u for u, _, _ in active]
        for _ in range(20):
            changed = False
            for i, a in enumerate(moved):
                for b in moved[i + 1:]:
                    delta = a.position - b.position
                    distance = float(np.linalg.norm(delta))
                    if distance < self.cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        correction = direction * ((self.cfg.min_separation_m - distance) / 2.0 + 0.05)
                        a.position += correction; b.position -= correction
                        a.velocity[:] = 0.0; b.velocity[:] = 0.0
                        events.extend([{"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid,
                                        "related_id": a.task_id, "details": {"minimum_separation_m": self.cfg.min_separation_m,
                                        "phase": "post_dynamics"}, "cause": "separation"},
                                       {"event_type": "SAFETY_OVERRIDE", "actor_id": b.uid,
                                        "related_id": b.task_id, "details": {"minimum_separation_m": self.cfg.min_separation_m,
                                        "phase": "post_dynamics"}, "cause": "separation"}])
                        changed = True
            for u in moved:
                before = u.position.copy()
                u.position[:2] = np.clip(u.position[:2], self.cfg.geofence_margin_m,
                                         self.cfg.arena_m - self.cfg.geofence_margin_m)
                if not np.allclose(before, u.position):
                    u.velocity[:2] = 0.0
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid,
                                   "related_id": u.task_id, "details": {"phase": "post_dynamics"},
                                   "cause": "geofence"})
            if not changed:
                break
        return events

    def _score(self, u: UAV, p: PoI, gcs: GCS, graph: ConnectivityGraph) -> float:
        d = u.distance_to(p.position)
        trip = (d + u.distance_to(gcs.position)) / max(u.speed_mps, 1)
        if u.battery_pct - trip * 0.12 <= self.cfg.reserve_pct: return 0.0
        base = (6 - p.priority) * 10.0 / (d + 1.0) * min(1.0, (u.battery_pct - self.cfg.reserve_pct) / 30.0)
        risk = (d / max(self.cfg.arena_m, 1.0)) * 30.0
        penalty = self.ccpl.penalty(risk=risk, distance=d, speed=float(np.linalg.norm(u.velocity)),
                                    battery=u.battery_pct / 100.0, connected=graph.reachable(u.uid),
                                    priority=p.priority)
        return base - penalty * 0.03

    def _relay_needed(self, u: UAV, uavs: list[UAV], graph: ConnectivityGraph) -> bool:
        return len(graph.route(u.uid)) >= 3 and graph.quality(u.uid) < 0.78

    def _relay_waypoint(self, u: UAV, uavs: list[UAV], gcs: GCS, graph: ConnectivityGraph) -> np.ndarray:
        connected = [x for x in uavs if not x.failed and x.uid != u.uid and graph.reachable(x.uid)
                     and x.battery_pct > self.cfg.reserve_pct + 8.0]
        if not connected: return gcs.position
        # Prefer a relay anchor that improves route quality while retaining
        # battery reserve.  The old nearest-to-GCS rule could repeatedly pick
        # a weak or nearly depleted relay and create oscillation.
        def relay_score(candidate: UAV) -> tuple[float, str]:
            distance = candidate.distance_to(gcs.position)
            quality = graph.quality(candidate.uid)
            redundancy = graph.redundancy(candidate.uid)
            battery_margin = max(0.0, candidate.battery_pct - self.cfg.reserve_pct) / 100.0
            score = quality * 100.0 + redundancy * 12.0 + battery_margin * 10.0 - distance * 0.08
            return score, candidate.uid
        anchor = max(connected, key=relay_score)
        return (anchor.position + gcs.position) / 2.0

    @staticmethod
    def _role(u: UAV, role: str, reason: str, task_id: str | None = None) -> dict:
        return {"event_type": "ROLE_CHANGE", "actor_id": u.uid, "related_id": task_id,
                "details": {"from": u.role, "to": role}, "cause": reason}
