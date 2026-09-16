# UAV-X: Resilient BVLOS Swarm Challenge

## Abstract

UAV-X is a deterministic, decentralized disaster-response swarm simulator. Each UAV surveys priority-weighted Points of Interest while preserving a multi-hop route to a Ground Control Station. Local task bidding, relay selection, battery-aware return-to-home, packet-level communication modelling, predictive separation, and deterministic safety shields allow the swarm to continue operating through outages, failures, and newly emerging tasks.

## System contribution

The system separates mission optimization from safety enforcement. A heuristic decentralized controller provides a reproducible baseline. CCPL is integrated as a constrained policy layer for high-level role scoring and delayed consequence handling. Geofence, battery reserve, communication fallback, and collision avoidance remain deterministic and policy-independent.

## Architecture

The Python simulator contains domain models, a time-varying communication graph, local task/relay control, disturbance injection, metrics, and a versioned JSONL log contract. A matplotlib visualizer provides the judge-facing path; optional MATLAB scripts consume the same frozen log for polished operations and replay views.

## Resilience mechanism

Every active UAV broadcasts a heartbeat across its current route. Each hop contributes latency and packet-loss probability. Relay candidates are selected using connectivity improvement, route redundancy, battery feasibility, and displaced mission value. If a route is lost, the UAV enters recovery and either repositions locally or returns home before its reserve threshold.

## Safety

Waypoints are predicted one tick ahead. Pairwise planned positions are separated before movement, and destinations are clipped to the geofence. Battery reserve triggers return-to-home. Safety overrides are logged with actor, cause, and simulation time.

## Evaluation

The simulator reports mission completion, priority-weighted score, completion time, packet delivery ratio, mean/P95 latency, connectivity downtime, route redundancy, relay reallocations, recovery time, collision-avoidance interventions, minimum separation, battery violations, and geofence violations.

## Reproducibility

```powershell
python -m pip install -r requirements.txt
python -m pytest -q tests
python -m uav_x.simulation.runner --scenario scenarios\baseline.yaml --policy heuristic --out runs/baseline.jsonl
python -m uav_x.visualization.operations_2d runs/baseline.jsonl --out artifacts/baseline.png
```

CCPL evaluation adds `requirements-ccpl.txt` and `--policy ccpl`; MATLAB is optional.
