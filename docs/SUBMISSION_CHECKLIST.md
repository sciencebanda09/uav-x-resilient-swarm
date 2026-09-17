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
- [x] Benchmark command: `python -m uav_x.benchmarks --all --duration 120`
- [x] Replay GIF and WebGL viewer assets
- [ ] Final 6–8 page technical proposal
- [ ] Demonstration video

## Claims to keep precise

- The current simulator is a deterministic proof of concept, not flight-certified BVLOS software.
- The current policy controller is centralized in its implementation, although it models distributed-style relay and task decisions.
- Packet-loss sensitivity and all known operating limits should be disclosed in the final submission.
