"""Challenge-aligned metrics from canonical records."""
from __future__ import annotations
from collections import Counter

def summarize(records: list[dict]) -> dict:
    ticks = [r for r in records if r["record_type"] == "tick"]
    events = [r for r in records if r["record_type"] == "event"]
    if not ticks: return {}
    final = ticks[-1]
    pois = final.get("pois", [])
    surveyed = sum(p["status"] == "SURVEYED" for p in pois)
    total = len(pois) or 1
    all_uavs = [u for u in final.get("uavs", []) if not u.get("failed")]
    connected = sum(u["gcs_reachable"] for u in all_uavs)
    separations = []
    for a_i, a in enumerate(all_uavs):
        for b in all_uavs[a_i + 1:]:
            import math
            separations.append(math.dist(a["position_m"], b["position_m"]))
    packets = [p for r in ticks for p in r.get("packets", [])]
    latencies = [p["latency_ms"] for p in packets if p.get("delivered")]
    violations = [e for e in events if e.get("event_type") in {"SAFETY_VIOLATION", "SAFETY_OVERRIDE"}]
    completed_times = [r["time_s"] for r in ticks if any(p["status"] == "SURVEYED" for p in r.get("pois", []))]
    outage_events = [e for e in events if e.get("event_type") in {"UAV_FAILURE", "LINK_OUTAGE"}]
    recovery_times = []
    for outage in outage_events:
        later = next((r["time_s"] for r in ticks if r["time_s"] > outage["time_s"] and all(u["gcs_reachable"] for u in r.get("uavs", []) if not u.get("failed"))), None)
        if later is not None: recovery_times.append(later - outage["time_s"])
    redundancy = [v for r in ticks for v in r.get("route_redundancy", {}).values()]
    return {"mission_completion_rate": surveyed / total,
            "priority_weighted_mission_score": sum((6-p["priority"]) for p in pois if p["status"] == "SURVEYED"),
            "mission_completion_time_s": completed_times[-1] if surveyed == total and completed_times else None,
            "connectivity_availability": connected / max(1, len(all_uavs)),
            "communication_downtime_ticks": sum(1 for r in ticks if not all(u["gcs_reachable"] for u in r.get("uavs", []) if not u.get("failed"))),
            "packet_delivery_ratio": sum(p.get("delivered", False) for p in packets) / max(1, len(packets)),
            "mean_latency_ms": sum(latencies) / max(1, len(latencies)),
            "p95_latency_ms": sorted(latencies)[max(0, int(len(latencies) * 0.95) - 1)] if latencies else None,
            "relay_reallocations": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") == "RELAY" for e in events),
            "mean_route_redundancy": sum(redundancy) / max(1, len(redundancy)),
            "recovery_events": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") == "RECOVER" for e in events),
            "recovery_time_s_mean": sum(recovery_times) / max(1, len(recovery_times)) if recovery_times else None,
            "collision_count": sum(e.get("cause") == "separation" for e in events),
            "safety_override_count": len(violations),
            "battery_violations": sum(e.get("cause") == "battery_reserve" for e in violations),
            "geofence_violations": sum(e.get("cause") == "geofence" for e in violations),
            "minimum_inter_uav_separation_m": min(separations) if separations else None,
            "event_counts": dict(Counter(e.get("event_type") for e in events))}
