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
    return {"mission_completion_rate": surveyed / total,
            "priority_weighted_mission_score": sum((6-p["priority"]) for p in pois if p["status"] == "SURVEYED"),
            "connectivity_availability": connected / max(1, len(all_uavs)),
            "communication_downtime_ticks": sum(1 for r in ticks if not all(u["gcs_reachable"] for u in r.get("uavs", []) if not u.get("failed"))),
            "relay_reallocations": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") == "RELAY" for e in events),
            "recovery_events": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") == "RECOVER" for e in events),
            "collision_count": sum(e.get("event_type") == "SAFETY_OVERRIDE" and e.get("cause") == "collision" for e in events),
            "minimum_inter_uav_separation_m": min(separations) if separations else None,
            "event_counts": dict(Counter(e.get("event_type") for e in events))}
