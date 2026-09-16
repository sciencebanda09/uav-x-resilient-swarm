# UAV-X

Python-only deterministic proof-of-concept for the Resilient BVLOS Swarm Challenge.

## Quick start

```powershell
python -m pip install -r requirements.txt
python -m uav_x.simulation.runner
python -m uav_x.visualization.operations_2d runs/uav_x_run.jsonl
python -m uav_x.visualization.replay_2d runs/uav_x_run.jsonl
python -m uav_x.visualization.replay_3d runs/uav_x_run.jsonl
python -m uav_x.validation --out reports/validation.json
```

For the realistic WebGL viewer:

```powershell
py -3.11 -m uav_x.interfaces.viewer_export runs/uav_x_run.jsonl --out runs/replay.json
cd viewer
npm install
npm run dev
```

Open the local Vite URL, then load `runs/replay.json`. The viewer uses the optimized fallback terrain and procedural quadrotor when photogrammetry/GLTF assets are not present.

The default `auto` policy uses the standalone CCPL package when installed and records an explicit heuristic fallback otherwise. To require CCPL, install `requirements-ccpl.txt` and run `python -m uav_x.simulation.runner --policy ccpl`. Hard safety constraints remain deterministic and outside the policy. MATLAB is optional. The Python simulator, logs, metrics, and matplotlib view are the authoritative reproducibility path.

Train a delayed-consequence CCPL checkpoint:

```powershell
python -m pip install -r requirements-ccpl.txt
python -m uav_x.train_ccpl --episodes 25
python -m uav_x.simulation.runner --policy ccpl --checkpoint checkpoints/uav_x_ccpl.pkl --out runs/ccpl_run.jsonl
```
