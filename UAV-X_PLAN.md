# UAV-X Resilient BVLOS Swarm Challenge Plan

## Project direction

Build UAV-X as a new civilian disaster-response project, separate from the existing `aegis-x` counter-drone system. Reuse neutral engineering patterns and simulation ideas, but do not carry over defence terminology or engagement functionality.

The core concept is a fully decentralized UAV swarm that surveys disaster-response Points of Interest (PoIs) while preserving a resilient multi-hop route to the Ground Control Station (GCS).

Each UAV must be able to survey, relay, return home, recharge, recover from isolation, avoid collisions, and respond to new high-priority tasks without depending continuously on the GCS.

## Challenge requirements to target

The system must demonstrate:

- Survey of all assigned disaster locations.
- End-to-end communication with the GCS.
- Dynamic relay-UAV assignment.
- Network reconfiguration after link degradation, UAV failure, or return-to-home.
- Prioritization of newly emerging high-priority regions.
- Safe mission completion within the allotted time.
- No geofence violations, unsafe battery depletion, or loss of minimum separation.

The implementation should directly measure the challenge criteria: mission completion, communication resilience, autonomous role management, fault recovery, safety, and innovation.

## Reuse from `aegis-x`

Reuse only civilian-neutral patterns and components:

- Deterministic seeded simulation and configuration from `engine/config.py`.
- Terrain elevation and line-of-sight modelling from `engine/environment/terrain.py`.
- Weather and disturbance modelling from `engine/environment/weather.py`.
- Atmosphere and wind modelling where useful.
- Packet loss and delay modelling patterns from `engine/core/sensors.py`.
- Replay and metrics collection patterns from `sim/replay.py` and `sim/metrics.py`.
- Configuration-driven scenario files.
- Dashboard and visualization lessons from `interfaces/dev/matplotlib_dashboard.py`.
- Existing unit, integration, and validation test organization.

Do not reuse as UAV-X domain concepts:

- Threats, interceptors, kill models, weapons, EW effects, ROE, or engagement decisions.
- Counter-drone terminology.
- `SensorTrack` and `EngagementDecision` as the primary UAV-X contracts.
- ACPL until a civilian mission objective has been defined and validated.

## CCPL integration

The external [CCPL repository](https://github.com/sciencebanda09/CCPL) is the default constrained-learning component for UAV-X. The current adapter uses the author's existing `ACPLAdvisor` implementation from `aegis-x` when available and documents the standalone `ccpl-rl` installation path.

Use CCPL only as an optional policy-learning layer for high-level local decisions such as:

- Selecting `SURVEY`, `RELAY`, `RECOVER`, `RETURN`, or `CHARGE`.
- Choosing among candidate relay positions.
- Trading mission priority against battery reserve and connectivity risk.

Do not let a learned policy override hard safety constraints. Geofencing, collision avoidance, minimum separation, battery reserve, and loss-of-connectivity fallback must remain deterministic safety shields around the policy.

The heuristic controller remains the reproducible baseline and safety-bounded fallback. CCPL re-ranks feasible survey and relay actions using delayed-consequence penalties; it does not override hard safety constraints.

CCPL-specific gates:

- Pin the CCPL implementation/version used by the run and record its backend in the manifest.
- Pin the exact CCPL commit/version and record dependency versions in reproducibility metadata.
- Use its discrete-action and NumPy-observation interfaces, or write a thin UAV-X adapter rather than coupling the simulator to CCPL internals.
- Log policy decisions, safety-shield overrides, delayed consequences, constraint costs, and checkpoints.
- Report both mission reward and safety/communication constraint results; do not present synthetic CCPL results as proof of field performance.

Recommended experiment matrix:

1. Heuristic controller, no disturbances.
2. Heuristic controller with outages, failures, and emergency PoIs.
3. CCPL controller with the same scenarios and seeds.
4. CCPL controller with safety shield enabled and disabled only in simulation analysis.

The challenge submission should retain the heuristic fallback even if CCPL improves average performance.

## Proposed project structure

```text
uav_x/
  core/
    models.py              # UAV, PoI, GCS, battery, task, link state
    swarm_agent.py         # decentralized UAV decision logic
    task_allocation.py     # distributed task bidding and reassignment
    relay_manager.py       # relay placement and route preservation
    connectivity.py        # multi-hop graph and GCS reachability
    collision_avoidance.py
    battery.py
    geofence.py
    mission.py

  simulation/
    world.py
    scenarios.py
    disturbances.py        # failures, packet loss, outages, emergencies
    runner.py
    replay.py

  metrics/
    mission_metrics.py
    communication_metrics.py
    safety_metrics.py

  interfaces/
    matlab_export.py
    log_schema.py

  matlab/
    operations_2d.m         # map, tasks, UAV roles, links, alerts
    replay_3d.m             # terrain and aerial replay
    plot_metrics.m
    load_log.m

  scenarios/
    baseline.yaml
    outage_recovery.yaml
    uav_failure.yaml
    emergency_priority.yaml
    return_to_home.yaml

  tests/
  docs/
    architecture.md
    reproducibility.md
    challenge_mapping.md
```

## Decentralized autonomy

Each UAV maintains a local state containing:

- Position, velocity, battery, payload status, and mission role.
- A timestamped neighbor table.
- Estimated route to the GCS.
- Link quality and hop count.
- Known PoIs and their priorities.
- Local safety constraints.

At every simulation tick, each UAV should:

1. Predict local link quality and battery feasibility.
2. Determine whether it is a valid relay.
3. Broadcast a compact heartbeat/state packet.
4. Bid for nearby tasks using priority, distance, battery, and communication cost.
5. Change roles only when the new role provides a measurable benefit.
6. Enter a safe fallback mode if isolated.
7. Return home before reaching the battery reserve threshold.

Use local bidding plus distributed consensus/state exchange as the Stage 1 implementation. Avoid claiming perfect swarm consensus before it is demonstrated and measured.

## Communication model

Model the swarm as a time-varying graph:

- Nodes are the GCS and UAVs.
- Edges are valid radio links.
- Edge state includes latency, packet-loss probability, bandwidth, distance, and line of sight.
- A communication path is valid only when a UAV has a route to the GCS.
- Relay decisions must preserve survey coverage while maintaining at least one route to the GCS.

The simulator must support packet loss, variable latency, terrain-obstructed links, link outages, UAV disappearance, temporary network partitions, and recovery after links return.

## MATLAB demonstration

Use Python for the deterministic simulation core and MATLAB for visualization and analysis. Base MATLAB plotting must be sufficient; UAV Toolbox and Simulink should remain optional because toolbox availability is not yet confirmed.

The 2D operations view should show terrain/disaster map, GCS, geofence, UAV paths, prioritized PoIs, role labels, multi-hop links, link quality, battery state, mission score, connectivity percentage, and a live event timeline.

The 3D replay should show terrain, UAV altitude and flight paths, relay chains, link failures, route reconstruction, UAV failure or return-to-home, emergency PoI insertion, and before/after recovery.

## Required metrics and artifacts

Measure:

- Mission completion rate.
- Completion time.
- Priority-weighted mission score.
- Packet delivery ratio.
- End-to-end latency.
- Connectivity availability and communication downtime.
- Relay reallocations.
- Network recovery time.
- Reconfiguration efficiency.
- Performance after UAV/link failures.
- Collision count and minimum inter-UAV separation.
- Geofence violations.
- Battery reserve violations.
- Safe returns to the GCS.

Every run must produce machine-readable event logs, per-tick UAV state logs, communication graph logs, final metrics JSON, CSV files for MATLAB, configuration metadata, and the random seed.

## Stage 1 delivery sequence

The PDF lists 27 September 2026 as the Stage 1 submission deadline. Build the first demonstrator in this order:

### Milestone 1: deterministic baseline

- GCS, 6–10 UAVs, terrain, and 8–15 PoIs.
- Battery model.
- Single-hop and multi-hop communication.
- Initial role assignment.
- 2D and 3D MATLAB views.
- Reproducible logs and metrics.

### Milestone 2: decentralized autonomy

- Local task bidding.
- Relay role election.
- Route-aware role changes.
- Battery-aware return-to-home.
- Geofence and collision avoidance.

### Milestone 3: disturbance scenarios

Demonstrate communication outage, relay-UAV failure, UAV return for recharge, and insertion of a new high-priority PoI.

### Milestone 4: submission package

Prepare the 6–8 page technical proposal, software architecture, source code, one-command installation instructions, reproducible simulation command, MATLAB visualization instructions, scenario logs, demonstration video, and a challenge-to-feature mapping table.

## Positioning

Present UAV-X as:

> A resilient, decentralized aerial network and mission planner for disaster-response BVLOS operations.

`aegis-x` is the engineering reference and source of reusable simulation patterns. UAV-X must retain its own civilian identity, domain model, communication-aware objective, and evaluation metrics.
