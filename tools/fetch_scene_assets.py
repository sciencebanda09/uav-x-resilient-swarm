"""Download public Wayanad scene inputs; raw data is gitignored."""
from __future__ import annotations
import argparse, gzip, json, shutil
from pathlib import Path
import requests

BBOX = (11.67, 76.08, 11.70, 76.16)

def download(url: str, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with requests.get(url, stream=True, timeout=120, headers={"User-Agent": "UAV-X-scene-builder/0.1"}) as r:
        r.raise_for_status()
        with path.open("wb") as h:
            for chunk in r.iter_content(1024 * 1024):
                if chunk: h.write(chunk)

def main() -> None:
    ap=argparse.ArgumentParser();ap.add_argument("--out",default="scene/raw/wayanad");ap.add_argument("--skip-satellite",action="store_true");args=ap.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    hgt=out/"N11E076.hgt";gz=out/"N11E076.hgt.gz"
    if not hgt.exists():
        download("https://s3.amazonaws.com/elevation-tiles-prod/skadi/N11/N11E076.hgt.gz",gz)
        with gzip.open(gz,"rb") as src,hgt.open("wb") as dst:shutil.copyfileobj(src,dst)
    south,west,north,east=BBOX; q=f"[out:json][timeout:60];(way[building]({south},{west},{north},{east});way[highway]({south},{west},{north},{east});way[waterway]({south},{west},{north},{east}););out geom;"
    osm=out/"osm.json"
    if not osm.exists(): osm.write_text(requests.post("https://overpass-api.de/api/interpreter",data=q,timeout=180).text,encoding="utf-8")
    if not args.skip_satellite:
        try:
            from pystac_client import Client
            import planetary_computer
            catalog=Client.open("https://planetarycomputer.microsoft.com/api/stac/v1");items=catalog.search(collections=["sentinel-2-l2a"],bbox=[west,south,east,north],query={"eo:cloud_cover":{"lt":20}}).item_collection();item=min(items,key=lambda x:x.properties.get("eo:cloud_cover",100));assets={k:planetary_computer.sign(v.href) for k,v in item.assets.items() if k in {"B02","B03","B04"}};preview=item.assets.get("rendered_preview") or item.assets.get("thumbnail")
            if preview:
                download(planetary_computer.sign(preview.href),out/"sentinel_preview.jpg")
            (out/"sentinel_assets.json").write_text(json.dumps({"item":item.id,"assets":assets,"preview":"sentinel_preview.jpg" if preview else None,"bbox":BBOX},indent=2),encoding="utf-8")
        except Exception as exc:(out/"satellite_warning.txt").write_text(f"Sentinel-2 unavailable: {exc}\n",encoding="utf-8")
    (out/"source_manifest.json").write_text(json.dumps({"scene":"wayanad_kerala","bbox":BBOX,"sources":{"srtm":"NASA/USGS SRTM 1 arc-second","osm":"OpenStreetMap Overpass API","satellite":"Copernicus Sentinel-2 L2A (best effort)"}},indent=2),encoding="utf-8");print(f"Scene inputs written to {out}")
if __name__=="__main__":main()
