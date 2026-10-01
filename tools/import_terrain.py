import argparse
import csv
import json
import math
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.merge import merge
from rasterio.transform import from_origin
from rasterio.warp import reproject


def main():
    parser = argparse.ArgumentParser(description="Crop Geoportal NMT tiles to the bus 108 corridor.")
    parser.add_argument("--source-crs", required=True, help="ASCII grids do not declare a CRS; e.g. EPSG:2180")
    parser.add_argument("--spacing", type=float, default=5.0)
    parser.add_argument("--margin", type=float, default=250.0)
    args = parser.parse_args()
    if not 1 <= args.spacing <= 20 or args.margin < 50:
        parser.error("Use spacing 1..20 metres and margin >= 50 metres.")
    root = Path(__file__).resolve().parents[1]
    route = json.loads((root / "data/route_108.json").read_text(encoding="utf-8-sig"))
    coordinates = np.array([point for direction in route["directions"] for point in direction["points"]])
    transform = Transformer.from_crs("EPSG:4326", args.source_crs, always_xy=True)
    east, north = transform.transform(coordinates[:, 0], coordinates[:, 1])
    source_margin = args.margin + 100.0
    bounds = (
        math.floor((min(east) - source_margin) / args.spacing) * args.spacing,
        math.floor((min(north) - source_margin) / args.spacing) * args.spacing,
        math.ceil((max(east) + source_margin) / args.spacing) * args.spacing,
        math.ceil((max(north) + source_margin) / args.spacing) * args.spacing,
    )
    sources = []
    with ExitStack() as stack:
        rasters = []
        for filename in sorted((root / "data").glob("*.asc")):
            raster = stack.enter_context(rasterio.open(filename))
            if raster.crs and raster.crs != rasterio.crs.CRS.from_string(args.source_crs):
                raise ValueError(f"CRS mismatch: {filename.name}: {raster.crs}")
            overlap = not (raster.bounds.right < bounds[0] or raster.bounds.left > bounds[2]
                           or raster.bounds.top < bounds[1] or raster.bounds.bottom > bounds[3])
            print(f"{filename.name}: {raster.width}x{raster.height}, {raster.res}, intersects={overlap}")
            if overlap:
                rasters.append(raster)
                sources.append({"file": filename.name, "bounds": list(raster.bounds), "resolution": list(raster.res)})
        if not rasters:
            raise ValueError("No tile intersects the route. Check the source CRS and downloaded area.")
        mosaic, affine = merge(rasters, bounds=bounds, res=args.spacing, nodata=-9999,
                               resampling=Resampling.bilinear, dtype="float32")
    heights = mosaic[0]
    valid = np.isfinite(heights) & (heights != -9999)
    columns = np.floor((east - affine.c) / affine.a).astype(int)
    rows = np.floor((north - affine.f) / affine.e).astype(int)
    route_valid = valid[rows, columns]
    manifest_path = next((root / "data").glob("pobieracz_nmt_*.txt"), None)
    surveys = []
    if manifest_path:
        with manifest_path.open(encoding="utf-8-sig", newline="") as manifest_file:
            surveys = list(csv.DictReader(manifest_file))
    report = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_crs": args.source_crs,
        "crs_note": "Explicit import parameter; ASC headers contain no CRS. No vertical datum conversion.",
        "source_manifest": manifest_path.name if manifest_path else None,
        "source_survey_dates": sorted({entry["aktualnosc"] for entry in surveys}),
        "attribution": "GUGiK / Geoportal NMT; source metadata retained alongside original downloads.",
        "sources": sources,
        "spacing_m": args.spacing,
        "bounds": list(bounds),
        "columns": int(heights.shape[1]), "rows": int(heights.shape[0]),
        "valid_fraction": float(valid.mean()),
        "route_samples": int(len(route_valid)),
        "route_missing": int((~route_valid).sum()),
        "height_min": float(heights[valid].min()), "height_max": float(heights[valid].max()),
        "route_height_min": float(heights[rows[route_valid], columns[route_valid]].min()) if route_valid.any() else None,
        "route_height_max": float(heights[rows[route_valid], columns[route_valid]].max()) if route_valid.any() else None,
    }
    print(json.dumps(report, indent=2))
    output = root / ".cache/terrain-report.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not route_valid.all():
        raise ValueError("NMT coverage has gaps on the route; see .cache/terrain-report.json. Existing game data unchanged.")
    origin_lon, origin_lat = coordinates[0]
    metres_per_degree = 111320.0
    radius = metres_per_degree * 180.0 / math.pi
    local_crs = (f"+proj=eqc +lat_ts={origin_lat} +lon_0={origin_lon} "
                 f"+y_0={-origin_lat * metres_per_degree} +R={radius} +units=m +no_defs")
    local_east = (coordinates[:, 0] - origin_lon) * metres_per_degree * math.cos(math.radians(origin_lat))
    local_south = -(coordinates[:, 1] - origin_lat) * metres_per_degree
    game_transform = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
    projected_east, projected_north = game_transform.transform(coordinates[:, 0], coordinates[:, 1])
    if not (np.allclose(projected_east, local_east, atol=0.001, rtol=0)
            and np.allclose(projected_north, -local_south, atol=0.001, rtol=0)):
        raise ValueError("Export projection does not match the game's coordinate convention.")
    west = math.floor((min(local_east) - args.margin) / args.spacing) * args.spacing
    top = math.floor((min(local_south) - args.margin) / args.spacing) * args.spacing
    width = math.ceil((max(local_east) + args.margin - west) / args.spacing) + 1
    height = math.ceil((max(local_south) + args.margin - top) / args.spacing) + 1
    runtime = np.full((height, width), -9999, dtype=np.float32)
    reproject(heights, runtime, src_transform=affine, src_crs=args.source_crs, src_nodata=-9999,
              dst_transform=from_origin(west - args.spacing / 2, -top + args.spacing / 2, args.spacing, args.spacing),
              dst_crs=local_crs, dst_nodata=-9999, resampling=Resampling.bilinear)
    if np.any(runtime == -9999) or not np.all(np.isfinite(runtime)):
        raise ValueError("Runtime crop has NoData cells. More surrounding tiles or a smaller margin are needed.")
    report.update({"origin_lon": float(origin_lon), "origin_lat": float(origin_lat),
                   "grid_x": west, "grid_z": top, "columns": width, "rows": height,
                   "height_min": float(runtime.min()), "height_max": float(runtime.max()),
                   "height_file": "res://data/terrain_108.bin",
                   "encoding": "little-endian float32 metres, row-major, north to south, cell centres"})
    binary_path = root / "data/terrain_108.bin"
    temporary_binary = binary_path.with_suffix(".bin.tmp")
    temporary_binary.write_bytes(runtime.astype("<f4").tobytes())
    temporary_binary.replace(binary_path)
    (root / "data/terrain_108.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Runtime heightfield: {width}x{height}, {binary_path.stat().st_size / 1048576:.2f} MiB, {args.spacing:g} m spacing")


if __name__ == "__main__":
    main()