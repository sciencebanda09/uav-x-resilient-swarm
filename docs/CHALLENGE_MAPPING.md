# Challenge mapping

| Challenge criterion | UAV-X implementation |
|---|---|
| Mission completion | PoI survey progress and priority-weighted summary metric |
| Communication resilience | Time-varying graph, packet loss, latency, route-to-GCS records |
| Relay and role management | Deterministic local bidding and relay-preservation heuristic |
| Fault recovery | Outage, UAV failure, isolation, recovery, and return-home scenarios |
| Safety | Separation shield, geofence bounds, battery reserve, fallback roles |
| Reproducibility | Seeded Python runner, versioned JSONL schema, optional visualizers |

The summary record also reports packet delivery ratio, mean/P95 latency, completion time, route redundancy, recovery time, battery/geofence violations, safety overrides, minimum separation, and collision-avoidance interventions.
