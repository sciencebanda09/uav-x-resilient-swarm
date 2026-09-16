"""Build a compact GLB terrain from downloaded Wayanad SRTM input."""
from __future__ import annotations
import argparse,json,shutil
from pathlib import Path
import numpy as np

def read_hgt(path:Path)->np.ndarray:
    data=np.fromfile(path,dtype=">i2");n=int(np.sqrt(data.size));return data.reshape(n,n).astype(float)
def main()->None:
    ap=argparse.ArgumentParser();ap.add_argument("--raw",default="scene/raw/wayanad");ap.add_argument("--out",default="viewer/public/scene");args=ap.parse_args()
    try:import trimesh
    except ImportError as exc:raise SystemExit("Install requirements-scene.txt first") from exc
    raw,out=Path(args.raw),Path(args.out);out.mkdir(parents=True,exist_ok=True);dem=read_hgt(raw/"N11E076.hgt");step=max(1,dem.shape[0]//101);dem=dem[::step,::step];dem=np.nan_to_num(dem,nan=float(np.nanmean(dem)));dem-=dem.min();dem=dem/dem.max()*75;n,m=dem.shape
    verts=np.array([[x,dem[j,i],z] for j,z in enumerate(np.linspace(0,500,n)) for i,x in enumerate(np.linspace(0,500,m))]);faces=[]
    for j in range(n-1):
      for i in range(m-1):
        a=j*m+i
        # x/east crossed with z/north must point upward (+y).
        faces.extend(((a,a+m,a+1),(a+1,a+m,a+m+1)))
    mesh=trimesh.Trimesh(vertices=verts,faces=np.asarray(faces),process=False)
    # Explicit UVs let the browser overlay the optional Sentinel preview.
    uv=np.array([[x/500.0,z/500.0] for z in np.linspace(0,500,n) for x in np.linspace(0,500,m)])
    mesh.visual=trimesh.visual.texture.TextureVisuals(uv=uv)
    mesh.export(out/"terrain.glb"); (out/"terrain_heightmap.json").write_text(json.dumps({"arena_m":500,"grid":int(n),"elevation_m":dem.round(3).tolist()},separators=(",", ":")),encoding="utf-8"); texture=None
    if (raw/"sentinel_preview.jpg").exists(): texture="/scene/sentinel_preview.jpg";shutil.copy2(raw/"sentinel_preview.jpg",out/"sentinel_preview.jpg")
    manifest={"scene_id":"wayanad_kerala_real","asset_mode":"real","coordinate_frame":"ENU","arena_m":500,"terrain":{"gltf_url":"/scene/terrain.glb","albedo_url":texture,"source":"NASA SRTM N11E076 + Copernicus Sentinel-2 preview","resolution_m":round(500/max(n,m),2)},"obstacles":{"metadata_url":"/scene/obstacles.json"},"attribution":["NASA/USGS SRTM","OpenStreetMap contributors","Copernicus Sentinel-2"]};(out/"scene_manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8");print(f"Generated {out/'terrain.glb'} ({len(verts)} vertices)")
if __name__=="__main__":main()
