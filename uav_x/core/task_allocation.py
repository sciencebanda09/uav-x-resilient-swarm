"""Deterministic decentralized-style heuristic controller."""
from __future__ import annotations
import numpy as np
from .models import GCS, PoI, ScenarioConfig, UAV, clamp_move
from .connectivity import ConnectivityGraph
from .ccpl_adapter import CCPLPolicy

class HeuristicController:
    def __init__(self, cfg: ScenarioConfig, policy: str = "auto",
                 checkpoint: str | None = None):
        self.cfg = cfg
        self.cooldown: dict[str, int] = {}
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
            home_need = u.distance_to(gcs.position) / max(u.speed_mps, 1) * 0.12
            if u.battery_pct <= self.cfg.reserve_pct + home_need:
                if u.role != "RETURN": events.append(self._role(u, "RETURN", "battery reserve"))
                u.role, u.task_id, u.mode = "RETURN", None, "RETURNING"
                continue
            if not graph.reachable(u.uid):
                if u.role not in {"RELAY", "RECOVER"}:
                    events.append(self._role(u, "RECOVER", "isolated route"))
                u.role, u.mode = "RECOVER", "ISOLATED"
                continue
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
                _, poi = max(scored, key=lambda pair: (pair[0], -int(pair[1].pid.split("-")[-1])))
                if u.task_id != poi.pid:
                    events.append(self._role(u, "SURVEY", "highest feasible bid", poi.pid))
                u.role, u.task_id, u.mode = "SURVEY", poi.pid, "CONNECTED"
                poi.status, poi.assigned_uav_id = "ASSIGNED", u.uid
                self.cooldown[u.uid] = 4
            else:
                u.role, u.task_id = "STANDBY", None
        return events

    def move(self, uavs: list[UAV], pois: list[PoI], gcs: GCS, graph: ConnectivityGraph, dt: float) -> None:
        by_id = {p.pid: p for p in pois}
        for u in uavs:
            if u.failed: continue
            if u.role in {"RETURN", "CHARGE"}: destination = gcs.position
            elif u.role == "SURVEY" and u.task_id in by_id: destination = by_id[u.task_id].position + np.array([0, 0, 50])
            elif u.role in {"RELAY", "RECOVER"}: destination = self._relay_waypoint(u, uavs, gcs, graph)
            else: destination = u.position
            clamp_move(u, destination, dt)

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
        connected = [x for x in uavs if not x.failed and graph.reachable(x.uid)]
        if not connected: return gcs.position
        anchor = min(connected, key=lambda x: x.distance_to(gcs.position))
        return (anchor.position + gcs.position) / 2.0

    @staticmethod
    def _role(u: UAV, role: str, reason: str, task_id: str | None = None) -> dict:
        return {"event_type": "ROLE_CHANGE", "actor_id": u.uid, "related_id": task_id,
                "details": {"from": u.role, "to": role}, "cause": reason}
