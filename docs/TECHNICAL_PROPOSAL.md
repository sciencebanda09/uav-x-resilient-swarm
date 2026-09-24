# UAV-X: Resilient BVLOS Swarm for Disaster Response
### PUSHPAK Grand Challenge 2026 — Grand Challenge 1 (IISERB @ Techfest, IIT Bombay) — Stage 1 Preliminary Design Verification

**Team:** [Fill: names, roles, affiliation — max 5 members] | **Contact:** [fill email]
**Repo:** UAV_X | **Simulator:** deterministic Python PoC (`uav_x/`) | **Log contract:** `uav-x-log/v1` JSONL
**Reproduce:** `pip install -r requirements.txt` → `python -m pytest -q` → `python tools/run_stage1.py`

---

## 1. Mission understanding and scope

A major earthquake/landslide has destroyed terrestrial comms. A Ground Control Station (GCS) at an operational center outside the affected area deploys a fleet of UAVs to survey Points of Interest (PoIs), collect imagery/situational data, and continuously relay it back over a resilient multi-hop aerial network. The figure mission fixes the envelope: 1000×1000 m operational area with the center 75 m west of its edge, 45-minute operation, 20-minute max flight time, 100 m max comm range, 100 m height ceiling, 5 m/s cruise, 20 m minimum separation, 10 randomly spawning PoIs (position and time), takeoff and landing at the center, and at most 10 s between PoI detection and reporting to the center.

The swarm must autonomously: (a) survey all assigned PoIs, (b) maintain end-to-end GCS connectivity, (c) dynamically assign relays, (d) reconfigure on degradation/failure/return-to-home, (e) prioritize newly emerging high-priority regions, (f) finish safely with all UAVs landed by 45 minutes. UAV-X answers this with a simulation-first PoC: a deterministic Python swarm simulator with terrain-aware flight, a time-varying multi-hop connectivity graph, priority/battery/risk-aware task bidding, self-elected relay/recovery roles, ferrying with reachable-gated reporting, predictive safety shields, battery reserve handling, landing slots, disturbance injection (outage, UAV failure, emergency PoI, recharge, combined), versioned JSONL logging, benchmark/validation/hidden-disturbance/ablation harnesses, matplotlib operations views, and a WebGL replay path. It is explicitly a PoC, not flight-certified BVLOS software.

## 2. System architecture

```text
ScenarioConfig/YAML → Runner → World (Terrain + Weather + Obstacles + UAV/PoI state)
  → HeuristicController/CCPL (allocation + relay election)
  → Safety layer (terrain/obstacle/geofence/separation pre-check + post-dynamics shield + terrain-clamp follow-up)
  → Dynamics (accel/tilt/yaw/drag/thrust) → Energy (hover+drag+climb+payload, 340 Wh ≈ 20 min)
  → ConnectivityGraph (100 m range + LOS + terrain occlusion + loss + bandwidth, rebuilt every tick)
  → SensorModel (GPS/IMU/camera metadata) → CoverageGrid (ground footprints)
  → uav-x-log/v1 JSONL → metrics / benchmarks / viewer export / Three.js replay
```

Module map: `uav_x/core/` (models, dynamics, terrain, connectivity, allocation), `uav_x/simulation/` (runner, sensors, disturbances, coverage), `uav_x/metrics/`, `uav_x/interfaces/` (schema, streaming, viewer export), `uav_x/visualization/` (operations views), `scenarios/` (6 challenge YAMLs, 2700 s), `tools/` (stage-1 pipeline, hidden/ablation evals, video builder), `viewer/` (Three.js replay), `matlab/` (optional replay scripts on frozen logs).

Key design split: **mission optimization is replaceable; safety is not.** The heuristic controller and the optional CCPL policy only propose roles/destinations. Geofence clipping (with home-base corridor), battery-reserve return, recall landing, communication fallback, and collision avoidance are deterministic, policy-independent, and logged with actor/cause/time. The simulator is authoritative; the viewer renders only exported replay and invents no positions, routes, events, or metrics.

Coordinates are ENU metres with the operational area at x ∈ [0, 1000], y ∈ [0, 1000] and the GCS at (−75, 500). Terrain is a 500 m survey rescaled to the arena (disclosed approximation); survey destinations sit at `min(terrain_height + 50, 100)` MSL. The flight model is lightweight but physical: 5 m/s cruise, 8 m/s climb, bounded accel/tilt/yaw, air-relative quadratic wind drag, payload-dependent hover/climb/parasite power against a 340 Wh pack. A PoI advances only inside the camera footprint, within 15–100 m AGL, below 3 m/s ground speed, for the 8 s dwell; the CoverageGrid (20 m cells) separately records total ground mapped.

## 3. Communication-aware autonomy and ferrying

Every tick the connectivity graph is rebuilt from pairwise 100 m range, line-of-sight, and terrain occlusion. Each active UAV holds a GCS route; each hop contributes latency and loss probability. Routing is reliability-aware (expected delivery, not hop count) with bounded retransmissions (`retransmit_limit: 2`). At 6×100 m links over 1 km² the mesh physically cannot span the area, so disconnected operation is normal: a tasked UAV keeps imaging while isolated and its imagery counts as reported only once the carrying UAV holds a live GCS route (`POI_REPORTED` with measured latency); data dies with its carrier (`DATA_LOST` → honest requeue, never phantom credit). Taskless isolated UAVs enter RECOVER toward the nearest connected node with a 20-tick GCS-direct escape. Heartbeats, per-link loss/latency, redundancy, availability, downtime, PDR (raw and connected), and mean/P95 latency are logged per tick.

Ablations confirm each piece earns its keep (2700 s): removing retransmissions costs −0.03 to −0.08 PDR with no mission gain; hop-only routing is PDR-neutral here; slow recharge costs PDR but the 90% resume ceiling protects completion. Packet-loss sensitivity holds completion 1.0 through 0/5/10% base loss and degrades honestly to 0.6 at 20% — disclosed as the operating boundary.

## 4. Task allocation, prioritization, and fault recovery

Each tick, unassigned PoIs (10 randomly positioned/prioritized, spawning over the first 600 s) are scored per UAV by priority weight `(6 − priority)`, distance, battery feasibility, and risk/connectivity; highest bid wins deterministically. A fresh bid is protected by a short cooldown so relay elections cannot flap imaging passes. Assignments release on return, recharge, landing, failure, or data loss so no PoI stalls assigned-to-nobody. Emergency injection adds a priority-1 PoI mid-run (1200 s in the benchmark scenario) that outbids everything; its injection-to-survey delay is reported as `emergency_response_s_max` (163–641 s observed).

Six reproducible 2700 s scenarios cover the challenge: `baseline`, `emergency_priority`, `outage_recovery` (120 s blackout at 900 s), `return_home_recharge` (35% start charge), `uav_failure` (loss at 800 s), `simultaneous_failure_outage` (both). A randomized hidden-disturbance suite (12 cases, seed 2026, range 90–115 m, loss 0–12%, spread failure/outage/emergency times, wind, GPS noise) guards against overfitting to scripted timings: 11/12 pass with zero collisions/battery/geofence violations throughout.

## 5. Safety, energy, and landing

Pre-movement: one-tick-ahead waypoint prediction, pairwise conflict resolution triggering at 1.25× `min_separation_m` (20 m) and correcting to 1.4×, destination clipping to the fence (west edge extended to −100 m for the home corridor), terrain/obstacle look-ahead with 14 m clearance. Post-dynamics: an independent shield re-verifies separation from raw trajectories, a follow-up pass repairs violations introduced by the terrain floor-clamp, parked (CHARGE/LAND) UAVs repel movers one-sidedly, and touchdown snaps ease movers away. Battery reserve (30%) triggers RETURN → CHARGE → resume at 90% (2.5%/s swap-style charge, disclosed abstraction); the sag floor (0.6) keeps reserve math (full-speed return) valid. End-of-mission recall (600 s margin) dominates assignment and lands every active UAV into staggered 25 m-ring slots (2D arrival, ground snap): `landed/unlanded_count` is a first-class metric. All overrides emit events.

Independently verified ground truth (recomputed from raw per-tick positions, not the event log): **0 actual collisions, 0 battery violations, 0 geofence violations, 0 obstacle violations across all 18 benchmark runs**; minimum separation holds 20.0–22.7 m — genuine margin above the 20 m floor. Regression tests lock the margin, the no-deadlock recovery, and the combined failure/outage completion.

## 6. Results (2700 s, seeds 7/17/27 — `reports/stage1_benchmark.json`, `validation.json`)

| Scenario | Completion | Time (s) | Landed | Availability / PDR (raw) | Connected PDR | Report max | Safety |
|---|---|---|---|---|---|---|---|
| baseline | 1.0 ×3 | 1096–1180 | 6/6 ×3 | 0.33–0.42 / 0.33–0.42 | 0.997–0.998 | 547–611 s | 0 coll, sep ≥20.3 |
| outage_recovery | 1.0 ×3 | 1096–1328 | 6/6 ×3 | 0.34–0.40 / 0.34–0.40 | 0.997–0.998 | 550–692 s | 0 coll, sep ≥20.1 |
| uav_failure | 1.0/0.9/1.0 | 1120–1768 | 5/5/5 | 0.28–0.37 / 0.29–0.36 | 0.996–0.998 | 558–611 s | 0 coll, sep ≥20.3 |
| emergency_priority | 1.0 ×3 | 1363–1841 | 6/6 ×3 | 0.30–0.40 / 0.30–0.40 | 0.996–0.998 | 547–692 s | 0 coll, sep ≥20.2 |
| return_home_recharge | 1.0 ×3 | 1032–1672 | 6/6 ×3 | 0.28–0.45 / 0.28–0.45 | 0.998–1.000 | 518–676 s | 0 coll, sep ≥20.2 |
| simultaneous_failure_outage | 1.0/0.9/1.0 | 1163–1765 | 5/5/5 | 0.28–0.37 / 0.28–0.36 | 0.991–0.997 | 558–806 s | 0 coll, sep ≥20.0 |

Monte Carlo (baseline ×3): completion 1.0, delivery 0.376 (ferry signature), latency 34.6 ms. Hidden: 11/12 (mean completion 0.93, zero safety violations; the fail holds zeros at 0.45). Emergency response 163–641 s. The two 0.9 runs are dead-carrier data losses that run out of clock — honest degradation, disclosed.

## 7. Reproducibility and submission artifacts

```powershell
$env:OPENBLAS_NUM_THREADS='1'; $env:OMP_NUM_THREADS='1'; $env:MPLBACKEND='Agg'
python -m pip install -r requirements.txt        # numpy, matplotlib, pyyaml only
python -m pytest -q                               # 30 regression tests (physics, scenarios, obstacles, sensing, connectivity, metrics, coverage)
python -m uav_x.simulation.runner --scenario scenarios\baseline.yaml --policy heuristic --out runs\baseline2700.jsonl
python -m uav_x.visualization.operations_2d runs\baseline2700.jsonl --out artifacts\operations_2d.png
python -m uav_x.benchmarks --all --duration 2700 --out reports\stage1_benchmark.json
python -m uav_x.validation --duration 2700 --out reports\validation.json
python tools\run_stage1.py                        # full evidence: tests + benchmark + validation + hidden (12) + ablations + failure/recovery log + viewer export + video → reports/stage1_package_summary.json
python tools\build_stage1_video.py runs\stage1_failure_recovery.jsonl --out artifacts\stage1_failure_recovery.gif --stride 30 --fps 12
```

Every run is deterministic for fixed seed+config; the log records seed, config hash, per-tick UAV/PoI/link/weather/events/coverage, and final metrics. CCPL path (`requirements-ccpl.txt`, `--policy ccpl`, `python -m uav_x.train_ccpl --episodes 25`) is optional; `auto` falls back to heuristic with an explicit record; MATLAB (`matlab/*.m`) consumes frozen logs only. Deliverables in repo: source, install docs, architecture (§2), PoC sim + 6 scenarios + JSONL logs (`runs/`), benchmarks/validation/hidden/ablation reports (`reports/`), operations PNGs, failure/recovery GIF, WebGL replay (`runs/stage1_failure_recovery_replay.json`), and full demo video (`artifacts/stage1_full_demo.mp4`, §8).

## 8. Demonstration video

`artifacts/stage1_full_demo.mp4` (~16 s, 1280×720, 12 fps): operations stills of the baseline and failure/recovery missions followed by the failure→recovery reel built by `tools/build_stage1_video.py` from `runs/stage1_failure_recovery.jsonl` (simultaneous failure+outage, stride 30 over 2700 s) — 4-panel judge view with map (GCS at the operational center, links shaded by quality, PoI stars, UAV trails, camera footprints, geofence with home corridor), mission status, battery/role bars, and completion-vs-availability timeline with event markers phasing NORMAL SURVEY → COMMUNICATION OUTAGE → COMBINED DISTURBANCE → SWARM RECOVERY → LANDING. Renderer invents no state; all frames are canonical-log replay. Narrated voiceover version to be recorded against this frozen build before email submission.

## 9. Novelty, limitations, and Stage 2 plan

Novelty: deterministic seeded 45-minute mission sim + versioned log contract as reviewable evidence; ferrying with reachable-gated reporting and dead-carrier requeue instead of phantom connectivity; dual safety accounting (event log + independent trajectory re-derivation); completion-time over the final surveyed set; landing roll-call and report-latency as first-class metrics; connected-vs-raw PDR decomposition; validation/hidden/ablation reports as artifacts rather than a single demo.

Limitations (disclosed): the 10 s detection-to-report bound is violated on far PoIs (max 518–806 s) — 6×100 m links over 1 km² must ferry, and ferrying takes minutes; raw availability/PDR (0.28–0.45) is the ferrying signature, reported beside connected PDR (0.991–1.0); failure scenarios lose dead-carrier imagery (0.9 in 2/18 runs); hidden case 7 completes 0.45 with all safety zeros; 20% base loss halves completion; terrain is a rescaled 500 m survey with original-coordinate obstacles; CCPL remains non-reference; 2.5%/s charging is a swap-style abstraction; full suite costs ~100 machine-minutes (thread caps documented). Stage 2: relay-chain positioning for idle UAVs (the principled attack on report latency), loss-adaptive retransmit, multi-process decentralization of the single-process controller that already models local decisions, PX4 SITL bridge on the same JSONL contract, and hardware-path sizing toward the Dec 16–18 finale unseen scenario.

## 10. Detailed models and algorithms

**Dynamics (per UAV, dt = 1 s).** Commanded velocity toward destination is clipped to 5 m/s cruise (2.5+ m/s floor under battery sag), vertical to 8 m/s climb; acceleration clipped (7.5 × battery factor, floor 0.6); yaw rate and tilt (28°) limited. Wind `w(x,y,t)` gives air-relative velocity; quadratic drag with payload-scaled mass; thrust solves hover + climb + drag for the 8.5 kg + 1.5 kg stack. Energy per tick: `P = P_hover(m) + P_climb·max(0,vz) + P_parasite·|v_air|³` terms integrated against 340 Wh (≈20 min hover at ~1016 W). Reserve logic (30% + trip estimate) triggers RETURN before integration, so no UAV can be commanded dry — verified by zero `battery_violations` in 18 runs.

**Connectivity.** For pair (i,j): in-range if `dist ≤ 100 m`; LOS via terrain-segment sampling plus obstacle AABB; per-link `packet_loss = base_loss + occlusion + distance_factor`, latency = propagation + queue + retransmit penalty. Route = reliability-weighted Dijkstra to GCS; `route_redundancy` counts alternate first hops. Availability = reachable-tick fraction (time-averaged); downtime = complement ticks.

**Controller pseudocode (heuristic, each tick):**

```text
spawn due PoIs → inject scripted disturbances → recall check (t ≥ 2700−600: force RETURN, dominate assignment)
release stale assignments (failed/returned/landed/charged/data-lost UAVs)
for each connected, uncooled, taskless UAV: bid on each PENDING/ASSIGNED PoI = (6−priority)*10/(d+1) * battery_factor − risk − ccpl_cost*0.01
assign highest bid deterministically (UAV-id tiebreak → reproducible)
for each UAV: tasked+isolated → hold (ferry) | taskless+isolated → RECOVER | weak route+idle → RELAY
              battery ≤ reserve+max(trip,margin) → RETURN | recalled → RETURN
safety layer clips/predicts/shields all destinations (fence with home corridor, 20 m separation with parked repulsion)
```

CCPL variant replaces only bid/relay scoring (`uav_x/train_ccpl.py`, 25 episodes); all shields unchanged. Stage 1 reports heuristic as reference.

**Sensor/survey.** GPS noise ~0.2–1.0 m in hidden eval, camera FOV 70°, AGL band 15–100 m, dwell 8 s, speed gate 3 m/s. Footprint radius `= AGL·tan(FOV/2)`; `survey_capture_active` only when footprint contains PoI. On completion the surveyor becomes the data carrier; `POI_REPORTED` fires on first carrier-reachable tick. CoverageGrid (20 m, 1000 m arena) tracks area mapped separately from task completion.

## 11. Scenario and disturbance specification

| File | Fleet | PoIs | Disturbance | What it proves |
|---|---|---|---|---|
| `baseline.yaml` | 6 | 10 random | none | nominal completion/safety/landing baseline |
| `outage_recovery.yaml` | 6 | 10 | 120 s blackout at 900 s | relay election + availability accounting |
| `uav_failure.yaml` | 6 | 10 | 1 UAV fails at 800 s | assignment release + data-loss requeue |
| `emergency_priority.yaml` | 6 | 10+1 at 1200 s | priority-1 PoI injected | preemption + response-time metric |
| `return_home_recharge.yaml` | 6 at 35% | 10 | battery-starved start | protected CHARGE + 90% resume, no dry flight |
| `simultaneous_failure_outage.yaml` | 6 | 10 | failure at 800 s + outage at 850 s | combined recovery under ferrying |

Hidden eval (12 cases, seed 2026) randomizes range 90–115 m, loss 0–12%, failure/outage/emergency times across the mission, wind, GPS noise, initial charge; pass gate adds `unlanded_count == 0`. Validation adds Monte Carlo (3 seeds) + loss sweep (0/5/10/20%).

## 12. Verification matrix (requirements → tests → evidence)

| Challenge metric | Regression test | Benchmark evidence |
|---|---|---|
| completion rate/time, priority score | `test_scenarios.py`, `test_coverage.py` (2700 s horizons) | §6 table, per-seed runs in `stage1_benchmark.json` |
| PDR/latency/availability/downtime | connectivity tests, loss sweep | raw 0.28–0.45, connected 0.991–1.0, latency 30–60 ms |
| relay reallocations, redundancy | relay-count tests | reallocs in disturbed/recharge runs |
| recovery time, reconfiguration | no-deadlock + combined-scenario tests | 0.9–1.0 under combined failure/outage |
| emergency response | injection event + `emergency_response_s_max` | 163–641 s, disclosed vs 10 s bound |
| report latency vs 10 s bound | carrier-gated reporting, `report_deadline_violations` | max 518–806 s, disclosed boundary |
| landing by 45 min | recall/landing behavior in mission runs | 6/6 (or 5/5 + failed) in all 18 runs |
| collisions, min separation | margin + dynamics + obstacle tests | 0 collisions (trajectory re-derived), sep 20.0–22.7 |
| battery/geofence/obstacle | reserve/CHARGE/recall tests | 0 violations all runs |
| determinism/reproducibility | full `pytest -q` in `tools/run_stage1.py` (30 tests) | seed+hash in every JSONL header, identical reruns |

## 13. Compliance, risks, and Stage 2 work plan

**Safety/compliance.** Simulation-only Stage 1: no outdoor flight, no DGCA authorization needed yet; open-source stack only (numpy/matplotlib/pyyaml, optional ROS/Gazebo external). Synthetic terrain/obstacles, no personal data. Original code; CCPL dependency isolated to `requirements-ccpl.txt`.

**Risks.** (1) Report-latency bound — ferrying is structural at this scale; Stage 2 relay chains are the principled fix, disclosed as such. (2) Single-process controller vs true decentralization — current code models local decisions centrally for reproducibility; Stage 2 splits into per-UAV processes with message-delay emulation. (3) Sim-to-real gap — Stage 2 adds PX4 SITL/Gazebo HITL on the same JSONL contract before hardware. (4) Compute — full suite ~100 machine-minutes; thread caps and the two-tier (smoke + full) workflow keep iteration fast; only narration and team details are manual.

**Stage 2 plan (Oct–Dec 2026).** Idle-UAV relay-chain positioning, loss-adaptive retransmit, multi-process decentralization, PX4 SITL bridge, extended hidden suite (loss bursts, GPS-denial windows, multi-failure), prototype sizing for the Dec 16–18 finale unseen scenario.

## References

PUSHPAK Grand Challenge 1 problem statement + mission figure (Techfest 2026-27, UAV-X: Resilient BVLOS Swarm); `scenarios/*.yaml`; `reports/stage1_benchmark.json`, `validation.json`, `hidden_disturbances.json`, `ablations.json`, `stage1_package_summary.json`; `docs/CHALLENGE_MAPPING.md`, `REPRODUCIBILITY.md`, `SUBMISSION_CHECKLIST.md`.

## Appendix A. Metrics glossary (standardized log format)

`mission_completion_rate`, `priority_weighted_mission_score`, `mission_completion_time_s` (first instant the final surveyed set — incl. late spawns — is complete), `report_latency_s_mean/max`, `report_deadline_violations` (>10 s + unreported), `landed/unlanded_count`, `connectivity_availability` (time-averaged), `communication_downtime_ticks`, `packet_delivery_ratio` (all heartbeats), `connected_packet_delivery_ratio` (routed only), `mean/p95_latency_ms`, `emergency_response_s_max`, `mean_route_redundancy`, `relay_reallocations`, `recovery_events/time`, `safety_intervention/override_count`, `actual_collision_count` (trajectory re-derived), `battery/geofence/obstacle_violations`, `minimum_inter_uav_separation_m`, `survey_coverage_fraction`, per-type `event_counts`.

## Appendix B. Related work and positioning

Market-based task allocation (CBBA-style auctions) structures our bidding, extended with live route-quality and battery-feasibility terms evaluated against the current graph each tick. FANET relay literature motivates reliability-weighted routing and bounded retransmission; our contribution is closing the loop with allocation under ferrying at figure scale in one deterministic rerunnable harness. Safety-shield practice (predict-then-clip, sticky reserve states, landing slots) follows UAV geofencing norms; dual accounting (event log + trajectory re-derivation) is a deliberate anti-gaming choice. Learned-policy work (including CCPL) stays a scorer above deterministic shields — autonomy where it helps, guarantees where it matters.

## Appendix C. Team, timeline, and submission contents

**Team (fill before email):** Member 1 — controls/simulation lead; Member 2 — networking/graphs; Member 3 — sensing/visualization; Member 4 — validation/benchmarks; Member 5 — integration/video. Affiliation, year/program, contact emails, and Techfest registration IDs go here.

| Date | Item |
|---|---|
| 22 Aug – 27 Sep 2026 | Stage 1 work window; freeze build 26 Sep |
| ≤ 27 Sep 2026 | Email submission to `pushpak_gc2026@aero.iitb.ac.in`: 6–8 page proposal (this doc, export to PDF), architecture (§2), source ZIP, `requirements.txt` + install steps (§7), PoC logs (`runs/stage1_failure_recovery.jsonl`, `runs/baseline2700.jsonl`), benchmark/validation reports, demo video (`artifacts/stage1_full_demo.mp4`) |
| 02 Oct 2026 | Stage 1 results; INR 1 Lakh per qualifying team (≤15) |
| 03 Oct – 02 Dec 2026 | Stage 2 hidden-disturbance hardening (§13) |

**Video narration script (90 s, record over the MP4):** 0–15 s mission + GCS/fleet/takeoff setup; 15–35 s nominal survey + relay mesh + ferrying; 35–60 s outage + failure injection, RECOVER/RELAY election, DATA_LOST requeue; 60–80 s recovery to full completion, landing roll-call, safety zeros; 80–90 s repro command + log contract close. Export proposal to PDF (11pt, figures from `artifacts/operations_2d.png`, `failure_recovery_operations.png`, `stage1_full_demo.mp4` stills) and attach alongside code + video.
