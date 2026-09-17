"""Build a Blender-free tiled terrain scene from the canonical heightmap.

The output is intentionally lightweight: terrain tiles are GLB files with
vertex colors, while the simulator continues to use the canonical JSON
heightmap and obstacle metadata.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def tile_mesh(trimesh, elevation: np.ndarray, arena: float, row: int, col: int,
              tiles: int, samples: int):
    h, w = elevation.shape
    y0 = int(round(row * (h - 1) / tiles))
    y1 = int(round((row + 1) * (h - 1) / tiles))
    x0 = int(round(col * (w - 1) / tiles))
    x1 = int(round((col + 1) * (w - 1) / tiles))
    rows = np.linspace(y0, y1, samples, dtype=int)
    cols = np.linspace(x0, x1, samples, dtype=int)
    verts = []
    colors = []
    for iy in rows:
        for ix in cols:
            x = ix / (w - 1) * arena
            z = iy / (h - 1) * arena
            height = float(elevation[iy, ix])
            # A restrained terrain palette keeps the scene readable without a
            # large texture atlas or a Blender material bake.
            t = np.clip(height / max(1.0, float(elevation.max())), 0.0, 1.0)
            color = [int(38 + 75 * t), int(58 + 58 * t), int(39 + 42 * t), 255]
            verts.append([x, height, z])
            colors.append(color)
    faces = []
    for y in range(samples - 1):
        for x in range(samples - 1):
            a = y * samples + x
            faces.extend(((a, a + samples, a + 1),
                          (a + 1, a + samples, a + samples + 1)))
    mesh = trimesh.Trimesh(vertices=np.asarray(verts, dtype=float),
                            faces=np.asarray(faces, dtype=np.int64), process=False)
    mesh.visual.vertex_colors = np.asarray(colors, dtype=np.uint8)
    return mesh


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--heightmap", default="viewer/public/scene/terrain_heightmap.json")
    parser.add_argument("--out", default="viewer/public/scene/tiles")
    parser.add_argument("--tiles", type=int, default=4)
    args = parser.parse_args()
    try:
        import trimesh
    except ImportError as exc:
        raise SystemExit("Install requirements-scene.txt first") from exc

    heightmap_path = Path(args.heightmap)
    payload = json.loads(heightmap_path.read_text(encoding="utf-8"))
    elevation = np.asarray(payload["elevation_m"], dtype=float)
    arena = float(payload.get("arena_m", 500.0))
    root = Path(args.out)
    manifest_tiles = []
    for lod, samples in (("low", 17), ("medium", 25), ("high", 33)):
        output = root / lod
        output.mkdir(parents=True, exist_ok=True)
        for row in range(args.tiles):
            for col in range(args.tiles):
                mesh = tile_mesh(trimesh, elevation, arena, row, col, args.tiles, samples)
                name = f"tile_{col}_{row}.glb"
                mesh.export(output / name)
                x0, x1 = col * arena / args.tiles, (col + 1) * arena / args.tiles
                z0, z1 = row * arena / args.tiles, (row + 1) * arena / args.tiles
                if lod == "low":
                    manifest_tiles.append({
                        "id": f"tile_{col}_{row}",
                        "bounds_m": [x0, z0, x1, z1],
                        "lod_urls": {
                            "low": f"/scene/tiles/low/{name}",
                            "medium": f"/scene/tiles/medium/{name}",
                            "high": f"/scene/tiles/high/{name}",
                        },
                    })
    manifest = {
        "scene_id": "wayanad_kerala_tiled",
        "asset_mode": "blender_free_tiled",
        "coordinate_frame": "ENU",
        "arena_m": arena,
        "tile_size_m": arena / args.tiles,
        "terrain_heightmap_url": "/scene/terrain_heightmap.json",
        "tiles": manifest_tiles,
        "fallback": {"gltf_url": "/scene/terrain.glb", "albedo_url": "/scene/sentinel_preview.jpg"},
        "obstacles": {"metadata_url": "/scene/obstacles.json"},
        "attribution": ["NASA/USGS SRTM", "Copernicus Sentinel-2 preview"],
    }
    (root.parent / "scene_manifest_tiled.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Generated {len(manifest_tiles) * 3} terrain tiles in {root}")


if __name__ == "__main__":
    main()
