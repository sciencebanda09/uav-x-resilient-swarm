"""Time-varying multi-hop communication graph."""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from .models import GCS, UAV

@dataclass
class Link:
    source_id: str
    target_id: str
    available: bool
    latency_ms: float
    packet_loss: float
    bandwidth_kbps: float
    line_of_sight: bool = True

class ConnectivityGraph:
    def __init__(self, gcs: GCS, range_m: float, base_loss: float, rng: np.random.Generator, terrain=None):
        self.gcs, self.range_m, self.base_loss, self.rng, self.terrain = gcs, range_m, base_loss, rng, terrain
        self.links: list[Link] = []
        self.routes: dict[str, list[str]] = {}
        self.tx_attempts = 0
        self.tx_delivered = 0
        self.latency_samples_ms: list[float] = []

    def update(self, uavs: list[UAV], outage: bool = False) -> None:
        active = [u for u in uavs if not u.failed]
        self.links = []
        adjacency: dict[str, list[tuple[str, float]]] = {u.uid: [] for u in active}
        adjacency["GCS"] = []
        for i, a in enumerate(active):
            for b in active[i + 1:]:
                self._add_edge(a.uid, b.uid, a.position, b.position, adjacency, outage)
            self._add_edge(a.uid, "GCS", a.position, self.gcs.position, adjacency, outage)
        self.routes = self._bfs_routes(adjacency)

    def _add_edge(self, a_id: str, b_id: str, a: np.ndarray, b: np.ndarray,
                   adjacency: dict[str, list[tuple[str, float]]], outage: bool) -> None:
        d = float(np.linalg.norm(a - b))
        los = True if self.terrain is None else self.terrain.los_clear(a, b)
        available = d <= self.range_m and los and not outage
        loss = min(0.99, self.base_loss + d / max(self.range_m, 1) * 0.12)
        link = Link(a_id, b_id, available, 20.0 + d * 0.25, loss,
                    max(64.0, 1800.0 - d * 4.0), los)
        self.links.append(link)
        reverse = Link(b_id, a_id, available, link.latency_ms, loss, link.bandwidth_kbps, available)
        self.links.append(reverse)
        if available:
            adjacency[a_id].append((b_id, d)); adjacency[b_id].append((a_id, d))

    @staticmethod
    def _bfs_routes(adjacency: dict[str, list[tuple[str, float]]]) -> dict[str, list[str]]:
        routes: dict[str, list[str]] = {"GCS": ["GCS"]}
        queue = ["GCS"]
        while queue:
            current = queue.pop(0)
            for neighbor, _ in adjacency.get(current, []):
                if neighbor not in routes:
                    routes[neighbor] = [neighbor] + routes[current]
                    queue.append(neighbor)
        return routes

    def route(self, uid: str) -> list[str]:
        return self.routes.get(uid, [])

    def reachable(self, uid: str) -> bool:
        return uid in self.routes

    def quality(self, uid: str) -> float:
        route = self.route(uid)
        if len(route) < 2:
            return 0.0
        relevant = {(x.source_id, x.target_id): x for x in self.links}
        return min(1.0 - relevant[(route[i], route[i + 1])].packet_loss
                   for i in range(len(route) - 1))

    def redundancy(self, uid: str) -> int:
        """Approximate route redundancy by counting connected first-hop options."""
        route = self.route(uid)
        if len(route) < 2:
            return 0
        first = route[1]
        return sum(1 for l in self.links if l.source_id == uid and l.available and l.target_id != first)

    def transmit(self, uavs: list[UAV]) -> list[dict]:
        """Send one heartbeat per active UAV across its current route."""
        relevant = {(x.source_id, x.target_id): x for x in self.links}
        packets = []
        for uav in uavs:
            route = self.route(uav.uid)
            if uav.failed:
                continue
            self.tx_attempts += 1
            delivered = len(route) >= 2; latency = 0.0
            for i in range(len(route) - 1):
                link = relevant[(route[i], route[i + 1])]
                latency += link.latency_ms
                if self.rng.random() < link.packet_loss:
                    delivered = False
            if delivered:
                self.tx_delivered += 1
                self.latency_samples_ms.append(latency)
            packets.append({"source_id": uav.uid, "delivered": delivered, "latency_ms": round(latency, 3)})
        return packets
