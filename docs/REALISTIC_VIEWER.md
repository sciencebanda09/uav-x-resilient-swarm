# Realistic scene viewer

The `viewer/` application is a lightweight Three.js/WebGL front-end. It is intentionally separate from the Python simulator so the simulator remains reproducible on machines without a GPU or Node.js.

## Run

```powershell
py -3.11 -m uav_x.simulation.runner --duration 300 --out runs/realistic_demo.jsonl
py -3.11 -m uav_x.interfaces.viewer_export runs/realistic_demo.jsonl --out runs/realistic_demo.json
cd viewer
npm install
npm run dev
```

Load `runs/realistic_demo.json` in the viewer (a short 300 s slice; full-mission
logs are `runs/baseline2700.jsonl` and `runs/stage1_failure_recovery.jsonl`).
The default scene is a 500 m terrain survey rescaled to the 1000 m arena with
obstacle metadata. Replace the scene assets referenced by `scene/scene_manifest.json` with optimized photogrammetry and GLTF files when available.

The viewer starts with an automatic Low/Medium/High graphics preset. On an
8 GB machine, keep `Auto` or `Low` selected. Low mode reduces terrain detail,
shadows, trail length, repeated scene objects, and visible radio links. The
viewer also supports 0.5×–5× playback speeds and incremental replay loading.

The repository includes a Blender-free tiled terrain pipeline. Rebuild the
lightweight scene tiles with:

```powershell
py -3.11 tools\build_realistic_scene.py
```

The default viewer keeps the satellite-textured terrain GLB because it is more
visually natural on an 8 GB machine. The generated tiled terrain can be tested
with `http://localhost:5173/?tiles=1`; it loads nearby tiles and unloads distant
ones. If tiles are missing, the viewer falls back to the procedural terrain and
the original compact terrain GLB.

For an 8 GB machine, keep the active scene bounded, use collision proxy meshes, and provide low/medium/high LODs. Do not commit large source photogrammetry datasets; keep only the manifest, lightweight fallback assets, and acquisition instructions in Git.

The viewer renders from the Python log and must not invent state. UAV positions, roles, battery, PoIs, links, and events all come from the canonical replay export. The procedural buildings, tower, bridge, and rubble use the same `scene/obstacles.json` AABB definitions consumed by predictive avoidance in the simulator.

The replay manifest is authoritative for the displayed arena size and GCS
position. The viewer frames the complete mission area, shows altitude stems and
role rings for each UAV, and distinguishes `BACKBONE` relay UAVs from survey
aircraft. The enhanced relay-backbone replay therefore reads as an actual
deployment: aircraft leave the home point, occupy the relay lattice, survey
PoIs, recover from disturbances, and return to their landing pads.

Optional GLB assets are loaded only when their URLs are populated in
`viewer/public/scene/scene_manifest.json`; otherwise the deterministic fallback
is used. This keeps the public repository small and means a judge never needs a
large photogrammetry download to reproduce the demo.
