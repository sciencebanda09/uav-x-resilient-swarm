# UAV-X Stage 1 submission checklist

This checklist tracks challenge deliverables without replacing the required
technical proposal.

## Product evidence

- [x] Source code
- [x] Installation instructions
- [x] Working proof-of-concept simulation
- [x] Software architecture documentation
- [x] Reproducible JSONL logs
- [x] Six disturbance scenarios
- [x] Automated tests
- [x] Benchmark command: `python -m uav_x.benchmarks --all --duration 2700`
- [x] Replay GIF and WebGL viewer assets
- [x] Final 6–8 page technical proposal (`docs/TECHNICAL_PROPOSAL.md`, ~3100 words + tables/figures → export to PDF; fill team names before email)
- [x] Demonstration video (`artifacts/stage1_full_demo.mp4`, ~16 s, 1280×720: baseline + failure/recovery stills and reel; narration script in proposal App. C)

## Claims to keep precise

- The current simulator is a deterministic proof of concept, not flight-certified BVLOS software.
- The current policy controller is centralized in its implementation: one
  simulation-side `HeuristicController` assigns roles and tasks. It models
  distributed-style local relay and task decisions for evaluation, but it is
  not yet a decentralized multi-process or multi-agent deployment.
- Packet-loss sensitivity and all known operating limits should be disclosed in the final submission.
