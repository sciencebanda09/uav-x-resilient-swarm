# Real Wayanad scene pipeline

The public viewer includes a compact 500 m terrain derivative generated from
NASA/USGS SRTM elevation data for Wayanad, Kerala. Raw downloads stay outside
Git under `scene/raw/`.

## Rebuild the asset

```powershell
py -3.11 -m pip install -r requirements-scene.txt
py -3.11 tools/fetch_scene_assets.py
py -3.11 tools/build_scene_glb.py
cd viewer
npm run build
```

The downloader also supports a best-effort Sentinel-2 search when the optional
STAC dependencies in `requirements-scene.txt` are installed. If imagery is not
available, the GLB still uses real elevation and the viewer retains its
procedural vegetation/material fallback.

The viewer loads `viewer/public/scene/terrain.glb` when present. If loading
fails, it switches automatically to deterministic procedural terrain. UAV-X
simulation coordinates remain ENU metres; the GLB uses x=east, y=altitude,
z=north, matching the browser coordinate conversion.

Attribution and source links are recorded in [scene/attribution.json](../scene/attribution.json).
