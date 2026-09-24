"""Deterministic decentralized-style heuristic controller."""
from __future__ import annotations
import numpy as np
from .models import (GCS, PoI, ScenarioConfig, UAV, advance_dynamics,
                     backbone_slot, geofence_clip_xy, landing_slot, safe_destination)
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
        backbone_count = min(self.cfg.backbone_relay_count, len(uavs)) if self.cfg.relay_backbone_enabled else 0
        for index, u in enumerate(uavs):
            if u.failed or u.role == "LAND":
                continue
            self.cooldown[u.uid] = max(0, self.cooldown.get(u.uid, 0) - 1)
            if index < backbone_count:
                if u.role != "BACKBONE":
                    events.append(self._role(u, "BACKBONE", "relay lattice station"))
                self._release_task(u, pois)
                u.role, u.task_id, u.mode = "BACKBONE", None, "BACKBONE"
                continue
            if u.role == "CHARGE":
                continue
            home_need = u.distance_to(gcs.position) / max(u.speed_mps, 1) * 0.12
            return_threshold = self.cfg.reserve_pct + max(
                home_need, self.cfg.recharge_trigger_margin_pct
            )
            if u.battery_pct <= return_threshold:
                if u.role != "RETURN": events.append(self._role(u, "RETURN", "battery reserve"))
                self._release_task(u, pois)
                u.role, u.task_id, u.mode = "RETURN", None, "RETURNING"
                continue
            if not graph.reachable(u.uid):
                self.isolated_ticks[u.uid] = self.isolated_ticks.get(u.uid, 0) + 1
                # ponytail: figure scale needs ferrying — a tasked UAV keeps
                # imaging while isolated and reports on reconnect. Only an
                # idle isolated UAV seeks the mesh; the battery reserve bounds
                # the solo excursion, so nothing deadlocks.
                if u.task_id:
                    continue
                if u.role not in {"RELAY", "RECOVER"}:
                    events.append(self._role(u, "RECOVER", "isolated route"))
                u.role, u.mode = "RECOVER", "ISOLATED"
                self._release_task(u, pois)
                continue
            self.isolated_ticks[u.uid] = max(0, self.isolated_ticks.get(u.uid, 0) - 4)
            if u.mode == "ISOLATED": u.mode = "CONNECTED"
            if (u.task_id is None or self.cooldown[u.uid] <= 0) and self._relay_needed(u, uavs, graph):
                if u.role != "RELAY": events.append(self._role(u, "RELAY", "route preservation"))
                self._release_task(u, pois)
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
                self._release_task(u, pois)
                u.role, u.task_id = "STANDBY", None
        return events

    def move(self, uavs: list[UAV], pois: list[PoI], gcs: GCS, graph: ConnectivityGraph, dt: float) -> list[dict]:
        by_id = {p.pid: p for p in pois}
        events = []
        planned = {}
        parked = set()
        for u in uavs:
            if u.failed: continue
            if u.role in {"CHARGE", "LAND"}:
                parked.add(u.uid)
                planned[u.uid] = (u, u.position.copy(), u.task_id)
                continue
            if u.role in {"RETURN"}: destination = landing_slot(u.uid, gcs.position)
            elif u.role == "BACKBONE": destination = backbone_slot(u.uid, self.cfg.arena_m)
            elif u.role == "SURVEY" and u.task_id in by_id:
                poi = by_id[u.task_id]
                destination = poi.position.copy()
                agl = self.cfg.survey_altitude_agl_m
                if self.terrain is not None:
                    destination[2] = min(self.terrain.height_at(poi.position[0], poi.position[1]) + agl,
                                         self.cfg.max_altitude_m)
                else:
                    destination[2] = min(poi.position[2] + agl, self.cfg.max_altitude_m)
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
        # Trigger on a safety buffer above the hard minimum (not the minimum
        # itself) and correct out to a real margin beyond it, so the reported
        # trajectory-wide minimum separation reflects genuine headroom rather
        # than oscillating at the exact threshold every time two UAVs converge.
        trigger_m = self.cfg.min_separation_m * 1.25
        target_m = self.cfg.min_separation_m * 1.4
        active = [(u, p, t) for u, p, t in planned.values() if u.uid not in parked]
        anchors = [(u.position.copy(), u.uid) for u in uavs if u.uid in parked]
        for _ in range(10):
            for i, (a, ap, atask) in enumerate(active):
                for b, bp, btask in active[i + 1:]:
                    delta = ap - bp; distance = float(np.linalg.norm(delta))
                    if distance < trigger_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        correction = direction * ((target_m - distance) / 2.0 + 0.2)
                        ap += correction; bp -= correction
                        events.extend([{"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid, "related_id": atask, "details": {"minimum_separation_m": self.cfg.min_separation_m}, "cause": "separation"},
                                       {"event_type": "SAFETY_OVERRIDE", "actor_id": b.uid, "related_id": btask, "details": {"minimum_separation_m": self.cfg.min_separation_m}, "cause": "separation"}])
            # ponytail: parked UAVs hold still but still repel movers one-sided.
            for a, ap, atask in active:
                for anchor_pos, anchor_uid in anchors:
                    delta = ap - anchor_pos; distance = float(np.linalg.norm(delta))
                    if distance < trigger_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        ap += direction * (target_m - distance + 0.2)
                        events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid, "related_id": atask, "details": {"minimum_separation_m": self.cfg.min_separation_m, "anchor": anchor_uid}, "cause": "separation"})
            for u, destination, task in active:
                before = destination.copy(); destination[:2] = geofence_clip_xy(destination[:2], self.cfg.arena_m, self.cfg.geofence_margin_m, self.cfg.geofence_min_x_m)
                if not np.allclose(before, destination):
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid, "related_id": task, "details": {}, "cause": "geofence"})
        previous_positions = {u.uid: u.position.copy() for u, _, _ in active}
        for u, destination, _ in active:
            advance_dynamics(u, destination, dt)
        for uid in parked:
            u = next(x for x in uavs if x.uid == uid)
            u.velocity[:] = 0.0
        # The dynamics integrator can overshoot a planned point.  Apply a
        # second, state-level shield after integration so the recorded flight
        # trajectory—not only the planned waypoint—obeys separation and the
        # geofence constraints.
        moved = [u for u, _, _ in active]
        parked_pos = {u.uid: u.position.copy() for u in uavs if u.uid in parked}
        for _ in range(60):
            changed = False
            for i, a in enumerate(moved):
                for b in moved[i + 1:]:
                    delta = a.position - b.position
                    distance = float(np.linalg.norm(delta))
                    if distance < self.cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        correction = direction * ((self.cfg.min_separation_m - distance) / 2.0 + 0.3)
                        a.position += correction; b.position -= correction
                        a.velocity[:] = 0.0; b.velocity[:] = 0.0
                        events.extend([{"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid,
                                        "related_id": a.task_id, "details": {"minimum_separation_m": self.cfg.min_separation_m,
                                        "phase": "post_dynamics"}, "cause": "separation"},
                                       {"event_type": "SAFETY_OVERRIDE", "actor_id": b.uid,
                                        "related_id": b.task_id, "details": {"minimum_separation_m": self.cfg.min_separation_m,
                                        "phase": "post_dynamics"}, "cause": "separation"}])
                        changed = True
                for pid, anchor in parked_pos.items():
                    delta = a.position - anchor
                    distance = float(np.linalg.norm(delta))
                    if distance < self.cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        a.position += direction * (self.cfg.min_separation_m - distance + 0.05)
                        a.velocity[:] = 0.0
                        events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": a.uid,
                                        "related_id": a.task_id, "details": {"minimum_separation_m": self.cfg.min_separation_m,
                                        "phase": "post_dynamics", "anchor": pid}, "cause": "separation"})
                        changed = True
            for u in moved:
                before = u.position.copy()
                u.position[:2] = geofence_clip_xy(u.position[:2], self.cfg.arena_m, self.cfg.geofence_margin_m, self.cfg.geofence_min_x_m)
                if not np.allclose(before, u.position):
                    u.velocity[:2] = 0.0
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid,
                                   "related_id": u.task_id, "details": {"phase": "post_dynamics"},
                                   "cause": "geofence"})
            if not changed:
                break
        # Validate the integrated state, not only the planned waypoint.  The
        # dynamics model can move through an obstacle or below the terrain
        # between planning ticks, especially while climbing over a tower.
        for u in moved:
            previous = previous_positions[u.uid]
            if self.obstacles is not None:
                hit = self.obstacles.blocking(previous, u.position)
                containing = self.obstacles.containing(u.position)
                if hit is not None or containing is not None:
                    obstacle = containing or hit
                    u.position = previous
                    u.velocity[:] = 0.0
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid,
                                   "related_id": u.task_id,
                                   "details": {"obstacle_id": obstacle.obstacle_id,
                                                "phase": "post_dynamics",
                                                "action": "rollback_before_obstacle"},
                                   "cause": "obstacle"})
            if self.terrain is not None:
                terrain_height = self.terrain.height_at(u.position[0], u.position[1])
                minimum_z = terrain_height + 14.0
                if u.position[2] < minimum_z:
                    u.position[2] = minimum_z
                    u.velocity[2] = max(0.0, u.velocity[2])
                    events.append({"event_type": "SAFETY_OVERRIDE", "actor_id": u.uid,
                                   "related_id": u.task_id,
                                   "details": {"terrain_height_m": round(float(terrain_height), 3),
                                                "clearance_m": 14.0,
                                                "phase": "post_dynamics"},
                                   "cause": "terrain"})
        # ponytail: the terrain floor-clamp above can lift two UAVs into each
        # other after separation was enforced — one final movers-only pass.
        for _ in range(10):
            changed = False
            for i, a in enumerate(moved):
                for b in moved[i + 1:]:
                    delta = a.position - b.position
                    distance = float(np.linalg.norm(delta))
                    if distance < self.cfg.min_separation_m:
                        direction = delta / distance if distance > 1e-9 else np.array([1.0, 0.0, 0.0])
                        push = direction * ((self.cfg.min_separation_m - distance) / 2.0 + 0.3)
                        a.position += push; b.position -= push
                        a.velocity[:] = 0.0; b.velocity[:] = 0.0
                        changed = True
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
        # CCPL/fallback cost is a tie-breaker for otherwise feasible work.  At
        # low battery the old 0.03 factor could make every remaining PoI score
        # negative even when the vehicle had enough energy to reach it and
        # return, leaving the fleet idle instead of cycling through recharge.
        return base - penalty * 0.01

    @staticmethod
    def _release_task(u: UAV, pois: list[PoI]) -> None:
        """Return an unfinished assignment to the shared task pool.

        Role changes can happen before the next allocation pass (failure,
        recharge, isolation, or a battery decision).  Clearing only
        ``u.task_id`` leaves the PoI permanently marked ASSIGNED.
        """
        if u.task_id is None:
            return
        poi = next((candidate for candidate in pois if candidate.pid == u.task_id), None)
        if poi is not None and poi.assigned_uav_id == u.uid and poi.status != "SURVEYED":
            poi.status = "PENDING"
            poi.assigned_uav_id = None
        u.task_id = None

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
