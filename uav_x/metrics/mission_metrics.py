"""Challenge-aligned metrics from canonical records."""
from __future__ import annotations
from collections import Counter
import math

def summarize(records: list[dict]) -> dict:
    ticks = [r for r in records if r["record_type"] == "tick"]
    events = [r for r in records if r["record_type"] == "event"]
    if not ticks: return {}
    manifest_cfg = (records[0].get("config") or {}) if records and records[0].get("record_type") == "manifest" else {}
    min_sep = float(manifest_cfg.get("min_separation_m", 20.0))
    deadline = float(manifest_cfg.get("report_deadline_s", 10.0))
    final = ticks[-1]
    pois = final.get("pois", [])
    surveyed = sum(p["status"] == "SURVEYED" for p in pois)
    total = len(pois) or 1
    connectivity_samples = []
    for r in ticks:
        active_now = [u for u in r.get("uavs", []) if not u.get("failed")]
        if active_now:
            connectivity_samples.append(sum(u["gcs_reachable"] for u in active_now) / len(active_now))
    connectivity_availability = (sum(connectivity_samples) / len(connectivity_samples)) if connectivity_samples else 0.0
    # Evaluate separation over the complete trajectory, not only the final
    # frame.  A safe final frame must not hide an earlier near-collision.
    separations = []
    actual_collisions = 0
    for r in ticks:
        active = [u for u in r.get("uavs", []) if not u.get("failed")]
        for a_i, a in enumerate(active):
            for b in active[a_i + 1:]:
                distance = math.dist(a["position_m"], b["position_m"])
                separations.append(distance)
                if distance < min_sep:
                    actual_collisions += 1
    packets = [p for r in ticks for p in r.get("packets", [])]
    routed = [p for p in packets if p.get("hop_count", 0) > 0]
    latencies = [p["latency_ms"] for p in packets if p.get("delivered")]
    safety_violations = [e for e in events if e.get("event_type") == "SAFETY_VIOLATION"]
    safety_overrides = [e for e in events if e.get("event_type") == "SAFETY_OVERRIDE"]
    # Completion time is the first instant at which the FINAL surveyed set is
    # complete — late spawns (random spawn, emergency injection) must not be
    # excluded just because the original set finished earlier.
    completion_time = None
    final_ids = {p["id"] for p in pois if p["status"] == "SURVEYED"}
    if final_ids:
        for r in ticks:
            frame = {p["id"] for p in r.get("pois", []) if p["status"] == "SURVEYED"}
            if final_ids.issubset(frame):
                completion_time = r["time_s"]
                break
    # Emergency response: worst injection-to-survey delay for injected PoIs.
    injected = {e.get("related_id"): e["time_s"] for e in events if e.get("event_type") == "EMERGENCY_POI"}
    emergency_response = []
    for r in ticks:
        for p in r.get("pois", []):
            if p["id"] in injected and p["status"] == "SURVEYED" and p.get("surveyed_time_s") is not None:
                emergency_response.append(p["surveyed_time_s"] - injected[p["id"]])
    outage_events = [e for e in events if e.get("event_type") in {"UAV_FAILURE", "LINK_OUTAGE"}]
    recovery_times = []
    for outage in outage_events:
        # Recover when the post-event active fleet is fully reachable.  Ignore
        # the failed vehicle itself and require a non-empty active fleet.
        later = next((r["time_s"] for r in ticks if r["time_s"] > outage["time_s"] and
                      (active := [u for u in r.get("uavs", []) if not u.get("failed")]) and
                      all(u["gcs_reachable"] for u in active)), None)
        if later is not None: recovery_times.append(later - outage["time_s"])
    redundancy = [v for r in ticks for v in r.get("route_redundancy", {}).values()]
    # ponytail: figure bounds — report latency per PoI, landing roll-call.
    report_latencies = []
    for p in pois:
        if p["status"] == "SURVEYED" and p.get("surveyed_time_s") is not None and p.get("reported_time_s") is not None:
            report_latencies.append(p["reported_time_s"] - p["surveyed_time_s"])
    unreported = sum(p["status"] == "SURVEYED" and p.get("reported_time_s") is None for p in pois)
    final_uavs = final.get("uavs", [])
    landed = sum(u["role"] == "LAND" for u in final_uavs if not u.get("failed"))
    unlanded = sum(u["role"] != "LAND" for u in final_uavs if not u.get("failed"))
    return {"mission_completion_rate": surveyed / total,
            "priority_weighted_mission_score": sum((6-p["priority"]) for p in pois if p["status"] == "SURVEYED"),
            "mission_completion_time_s": completion_time,
            "report_latency_s_mean": sum(report_latencies) / len(report_latencies) if report_latencies else None,
            "report_latency_s_max": max(report_latencies) if report_latencies else None,
            "report_deadline_violations": sum(v > deadline for v in report_latencies) + unreported,
            "landed_count": landed,
            "unlanded_count": unlanded,
            "connectivity_availability": connectivity_availability,
            "communication_downtime_ticks": sum(1 for r in ticks if not all(u["gcs_reachable"] for u in r.get("uavs", []) if not u.get("failed"))),
            "packet_delivery_ratio": sum(p.get("delivered", False) for p in packets) / max(1, len(packets)),
            "connected_packet_delivery_ratio": sum(p.get("delivered", False) for p in routed) / max(1, len(routed)),
            "emergency_response_s_max": max(emergency_response) if emergency_response else None,
            "mean_latency_ms": sum(latencies) / max(1, len(latencies)),
            "p95_latency_ms": sorted(latencies)[max(0, int(len(latencies) * 0.95) - 1)] if latencies else None,
            "relay_reallocations": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") in {"RELAY", "RECOVER"} for e in events),
            "mean_route_redundancy": sum(redundancy) / max(1, len(redundancy)),
            "recovery_events": sum(e.get("event_type") == "ROLE_CHANGE" and e.get("details", {}).get("to") == "RECOVER" for e in events),
            "recovery_time_s_mean": sum(recovery_times) / max(1, len(recovery_times)) if recovery_times else None,
            "safety_intervention_count": sum(e.get("cause") == "separation" for e in events),
            "safety_override_count": len(safety_overrides),
            "actual_collision_count": actual_collisions,
            "battery_violations": sum(e.get("cause") == "battery_reserve" for e in safety_violations),
            "geofence_violations": sum(e.get("cause") == "geofence" for e in safety_violations),
            "obstacle_violations": sum(e.get("cause") in {"obstacle", "terrain"} for e in safety_violations),
            "obstacle_interventions": sum(e.get("cause") == "obstacle" for e in safety_overrides),
            "terrain_interventions": sum(e.get("cause") == "terrain" for e in safety_overrides),
            "minimum_inter_uav_separation_m": min(separations) if separations else None,
            "event_counts": dict(Counter(e.get("event_type") for e in events))}
