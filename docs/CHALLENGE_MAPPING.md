# Challenge mapping

Evidence below is the 2700-second (45-minute), three-seed benchmark
(`python -m uav_x.benchmarks --all --duration 2700`), reproducible per
scenario via `python -m uav_x.simulation.runner --scenario <path>`. Full
distributions come from `python -m uav_x.validation`.

Figure mission scale: 1000×1000 m area, GCS at the operational center 75 m
west of the area edge, 100 m comm range, 5 m/s cruise, 20 m separation,
100 m ceiling, 20-minute endurance, 10 randomly spawning PoIs, takeoff and
landing at the center, 10 s detection-to-report bound.

| Challenge criterion (weight) | UAV-X implementation | Evidence |
|---|---|---|
| Mission Completion (25%) | Local task bidding scores candidate PoIs by priority, distance, and battery feasibility; a deterministic highest-bid assignment resolves conflicts each tick. Tasked UAVs keep imaging while isolated (ferrying) and report on reconnect; unfinished assignments release on return, recharge, failure, or data loss. | Three-seed 2700 s benchmark: `mission_completion_rate = 1.0` in 16/18 runs (baseline 1096–1180 s, outage 1096–1328 s, emergency 1363–1841 s incl. the injected PoI, recharge 1032–1672 s). The two 0.9 runs are UAV-failure scenarios where the failed carrier's unreported imagery is honestly requeued (`DATA_LOST`) and one PoI runs out of time. |
| Communication Resilience (25%) | Time-varying multi-hop graph (`connectivity.py`) rebuilt every tick from 100 m range, line-of-sight, and terrain occlusion; reliability-aware routing; bounded retransmission (`retransmit_limit: 2`). Disconnected surveyors ferry data and report on reconnect. | Raw heartbeat PDR 0.28–0.45 reflects ferry legs with no route by design; **connected PDR is 0.991–1.0 in every run**, mean latency 30–60 ms. Sensitivity sweep holds completion 1.0 through 0/5/10% base loss and degrades to 0.6 at 20%, disclosed as the operating boundary. |
| Autonomous Relay & Role Management (20%) | UAVs self-elect into RELAY when route redundancy/quality drops (tasked surveyors hold through a cooldown so imaging dwells survive); taskless isolated UAVs enter RECOVER toward the nearest connected node with a 20-tick GCS-direct escape; staggered landing slots keep the home mesh ordered. | `relay_reallocations` nonzero in disturbed runs (up to 133 in recharge cycling); `actual_collision_count: 0` in all 18 runs confirms reconfiguration never trades recovery for safety. |
| Fault Recovery & Swarm Reconfiguration (15%) | Data dies with its carrier (`DATA_LOST` → honest requeue, never phantom credit); battery reserve (30%) with full-speed return assumption recalls UAVs; end-of-mission recall (600 s margin) lands the fleet; `CHARGE` is a protected role. | `simultaneous_failure_outage` reaches 0.9–1.0 with zero safety violations; hidden-disturbance suite passes 11/12 (the fail still holds all safety zeros at 0.45 completion). Regression tests `test_isolated_uav_does_not_deadlock_indefinitely` and `test_simultaneous_failure_outage_completes_most_pois` lock this in. |
| Safety & Collision Avoidance (10%) | Predictive one-tick-ahead resolution triggers at 1.25× the 20 m minimum and corrects to 1.4×; a post-dynamics shield, a terrain-clamp follow-up pass, one-sided parked-UAV repulsion, landing-touchdown deconfliction, geofence with home-base corridor, and staggered slots round out the shield. | Zero `battery_violations`, zero `geofence_violations`, zero `actual_collision_count` (re-derived from raw trajectories, not event logs) across all 18 runs; `minimum_inter_uav_separation_m` 20.0–22.7 m against the 20 m floor. Regression test `test_separation_shield_maintains_margin_not_just_bare_minimum` locks this in. |
| Innovation & Technical Merit (5%) | Deterministic seeded simulation with a versioned JSONL log contract; dual safety accounting (event log + independent trajectory re-derivation); first-class validation/hidden/ablation reports; per-PoI report-latency and landing roll-call metrics; optional CCPL constrained-policy scorer above deterministic shields. | `mission_completion_time_s` covers the final surveyed set including late spawns (no pre-spawn shortcut); `emergency_response_s_max` (163–641 s) and `connected_packet_delivery_ratio` are reported alongside raw figures so judges see both the ferrying reality and the mesh quality. |

## Figure-constraint compliance

| Figure constraint | Implementation | Status |
|---|---|---|
| 45-min mission / land by 45 min | `duration_s: 2700`, recall at 600 s margin, `LAND` role, `landed/unlanded_count` | Met: 6/6 (or 5/5 + 1 failed) landed in all 18 runs |
| 20-min endurance | `battery_capacity_wh: 340` (~20 min hover), reserve 30%, resume 90% | Met: 0 battery violations; endurance math in proposal §5 |
| 100 m comm range | `radio_range_m: 100` default, hidden sweep 90–115 | Met |
| 1000×1000 m area, GCS 75 m west | `arena_m: 1000`, GCS at `(-75, 500)`, fence extended to home corridor | Met |
| Takeoff from center | Ground spawn at GCS slots | Met |
| 100 m ceiling | Waypoint cap + capture cone `AGL ≤ 100` | Met |
| 5 m/s cruise | `speed_mps: 5.0` | Met |
| 20 m separation | `min_separation_m: 20`, shields above | Met: 0 collisions, min 20.0–22.7 |
| 10 s detection-to-report | Per-PoI carrier + reachable-gated `POI_REPORTED`, `report_deadline_violations` | Measured, not met: max 518–754 s (ferry physics at 6×100 m over 1 km² cannot meet 10 s for far PoIs — disclosed boundary, §Known limitations) |
| 10 random PoIs | `poi_count: 10`, seeded random position/priority/spawn ≤600 s | Met |

## Tuning and fixes applied (figure retune)

- **World rescaled to the figure**: 500→1000 m arena, 220→100 m range, 12→5 m/s, 18→20 m separation, 720→340 Wh battery, 120→2700 s mission, GCS moved outside the west edge with ground takeoff and landing slots.
- **Ferrying model**: tasked UAVs hold tasks while isolated (disconnect is normal at this scale); idle isolated UAVs seek the mesh; reports gate on carrier reachability; dead-carrier data requeues via `DATA_LOST`.
- **Task-leak fix**: RELAY election now releases the PoI (previously a PoI stayed `ASSIGNED` to nobody forever).
- **Home-base geometry**: landing slots (25 m ring), parked-UAV one-sided repulsion, touchdown deconfliction, 2D arrival check with ground snap — a 5 m 3D arrival radius is unreachable under a 20 m shield.
- **Recall dominance**: assignment can no longer demote a recall to `STANDBY`; reserve raised 22→30%, resume 65→90% for viable far sorties, sag floor 0.35→0.6 so reserve math (full-speed return) holds, recall margin 600 s.
- **`connectivity_availability` time-averaged** across ticks, not last-tick.
- **`actual_collision_count`** re-derived from raw trajectories; **`mission_completion_time_s`** covers the final surveyed set including late spawns.

## Known limitations (disclosed, not hidden)

- The 10 s detection-to-report bound is violated on far PoIs (observed max 518–754 s): with 6×100 m links over 1 km² the swarm must ferry, and ferrying takes minutes. Connected delivery is 0.991–1.0, so the gap is reachability time, not link quality.
- Raw availability/PDR sit at 0.28–0.45 for the same reason — the signature of ferrying, reported alongside connected PDR rather than hidden.
- Failure scenarios lose the dead carrier's unreported imagery (0.9 in 2/18 runs); re-survey competes with the clock.
- Hidden case 7 (combined failure+outage+emergency at high loss) completes 0.45 with all safety zeros — the honest stress ceiling.
- High base packet loss (20%) halves completion despite retransmission.
- Terrain heightmap is a 500 m survey rescaled to the 1000 m arena; obstacle cluster keeps original coordinates (SW quadrant). Both disclosed; physics (LOS/occlusion) runs at arena scale.
- CCPL scoring is integrated but the heuristic controller remains the Stage 1 reference.

## Enhanced relay-backbone profile

The baseline evidence above intentionally retains the official figure
assumptions: six UAVs, 100 m radio range, and a 1 km² area. To close the
physical report-latency gap, the repository now includes
`scenarios/relay_backbone.yaml`. It changes the deployment architecture rather
than redefining the baseline metric: 15 UAVs are used, nine are station-keeping
relay nodes on a 3×3 lattice, the radio profile is 400 m, and relay pads supply
energy during the 45-minute mission. In the full seed-7 run this profile
achieves 10/10 PoI completion, 0.0 s measured report latency, zero deadline
violations, 15/15 landing roll-call, zero collisions, and zero battery,
geofence, or obstacle violations. This is an enhanced deployment option, not a
claim that the original six-UAV/100 m figure has changed.
