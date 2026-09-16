# UAV-X

Python-only deterministic proof-of-concept for the Resilient BVLOS Swarm Challenge.

## Quick start

```powershell
python -m pip install -r requirements.txt
python -m uav_x.simulation.runner --out uav_x_run.jsonl
python -m uav_x.visualization.operations_2d uav_x_run.jsonl --out uav_x_operations.png
```

The default `auto` policy uses the standalone CCPL package when installed and records an explicit heuristic fallback otherwise. To require CCPL, install `requirements-ccpl.txt` and run `python -m uav_x.simulation.runner --policy ccpl`. Hard safety constraints remain deterministic and outside the policy. MATLAB is optional. The Python simulator, logs, metrics, and matplotlib view are the authoritative reproducibility path.
