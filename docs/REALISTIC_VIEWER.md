# Realistic scene viewer

The `viewer/` application is a lightweight Three.js/WebGL front-end. It is intentionally separate from the Python simulator so the simulator remains reproducible on machines without a GPU or Node.js.

## Run

```powershell
py -3.11 -m uav_x.simulation.runner --duration 120 --out runs/realistic_demo.jsonl
py -3.11 -m uav_x.interfaces.viewer_export runs/realistic_demo.jsonl --out runs/realistic_demo.json
cd viewer
npm install
npm run dev
```

Load `runs/realistic_demo.json` in the viewer. The default scene is a 500 m bounded fallback terrain with obstacle metadata. Replace the scene assets referenced by `scene/scene_manifest.json` with optimized photogrammetry and GLTF files when available.

For an 8 GB machine, keep the active scene bounded, use collision proxy meshes, and provide low/medium/high LODs. Do not commit large source photogrammetry datasets; keep only the manifest, lightweight fallback assets, and acquisition instructions in Git.

The viewer renders from the Python log and must not invent state. UAV positions, roles, battery, PoIs, links, and events all come from the canonical replay export. The procedural buildings, tower, bridge, and rubble use the same `scene/obstacles.json` AABB definitions consumed by predictive avoidance in the simulator.

Optional GLB assets are loaded only when their URLs are populated in
`viewer/public/scene/scene_manifest.json`; otherwise the deterministic fallback
is used. This keeps the public repository small and means a judge never needs a
large photogrammetry download to reproduce the demo.
