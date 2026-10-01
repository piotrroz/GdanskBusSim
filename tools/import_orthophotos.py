import argparse
import csv
import json
import math
import warnings
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rasterio
from pyproj import CRS, Transformer
from rasterio.enums import ColorInterp, Resampling
from rasterio.errors import NotGeoreferencedWarning
from rasterio.features import rasterize
from rasterio.transform import from_origin
from rasterio.vrt import WarpedVRT


def write_png(filename, pixels):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", NotGeoreferencedWarning)
        with rasterio.open(filename, "w", driver="PNG", width=pixels.shape[2], height=pixels.shape[1],
                           count=3, dtype="uint8") as output:
            output.write(pixels)
        with rasterio.open(filename) as verify:
            if not np.array_equal(verify.read(), pixels):
                raise ValueError("PNG round-trip failed.")


def grid_coverage(coordinates, transform, affine, valid):
    east, north = transform.transform(coordinates[:, 0], coordinates[:, 1])
    columns = np.floor((east - affine.c) / affine.a).astype(int)
    rows = np.floor((north - affine.f) / affine.e).astype(int)
    inside = (columns >= 0) & (rows >= 0) & (columns < valid.shape[1]) & (rows < valid.shape[0])
    covered = np.zeros(len(coordinates), dtype=bool)
    covered[inside] = valid[rows[inside], columns[inside]]
    return covered


def inspect_sources(root, stack):
    manifests = sorted((root / "data").glob("pobieracz_ortofoto_*.txt"))
    entries = {}
    for manifest in manifests:
        with manifest.open(encoding="utf-8-sig", newline="") as stream:
            entries.update({entry["nazwa_pliku"]: entry for entry in csv.DictReader(stream)})
    sources = []
    rasters = []
    for filename in sorted((root / "data").glob("*.tif")):
        raster = stack.enter_context(rasterio.open(filename))
        if raster.crs is None:
            raise ValueError(f"{filename.name}: missing embedded CRS; refusing to guess.")
        if not all(channel in raster.colorinterp for channel in (ColorInterp.red, ColorInterp.green, ColorInterp.blue)):
            raise ValueError(f"{filename.name}: expected explicitly tagged RGB channels.")
        if any(dtype != "uint8" for dtype in raster.dtypes):
            raise ValueError(f"{filename.name}: expected 8-bit imagery; a calibrated conversion is required.")
        entry = entries.get(filename.name, {})
        if entry.get("uklad_wspolrzednych") == "PL-1992":
            embedded = CRS(raster.crs)
            reference = CRS.from_epsg(2180)
            operation = embedded.coordinate_operation
            expected = reference.coordinate_operation
            parameters = {parameter.name: parameter.value for parameter in operation.params} if operation else {}
            matches = operation and operation.method_name == expected.method_name and all(
                math.isclose(parameters.get(parameter.name, math.inf), parameter.value, rel_tol=0, abs_tol=1e-8)
                for parameter in expected.params)
            if not matches or not math.isclose(embedded.ellipsoid.semi_major_metre, reference.ellipsoid.semi_major_metre, abs_tol=0.001):
                raise ValueError(f"{filename.name}: embedded projection disagrees with PL-1992 metadata.")
        rasters.append(raster)
        sources.append({
            "file": filename.name, "bytes": filename.stat().st_size,
            "crs": raster.crs.to_string(), "width": raster.width, "height": raster.height,
            "bounds": list(raster.bounds), "resolution": list(raster.res),
            "bands": [channel.name for channel in raster.colorinterp],
            "survey_date": entry.get("aktualnosc"), "sheet": entry.get("godlo"),
        })
    if not rasters:
        raise ValueError("No source .tif files found in data/.")
    return rasters, sources, [manifest.name for manifest in manifests]


def main():
    parser = argparse.ArgumentParser(description="Prepare Geoportal RGB orthophotos for the existing terrain grid.")
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--metres-per-pixel", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.metres_per_pixel) or args.metres_per_pixel < 0.5:
        parser.error("Runtime imagery requires at least 0.5 metres per pixel.")
    root = Path(__file__).resolve().parents[1]
    terrain = json.loads((root / "data/terrain_108.json").read_text(encoding="utf-8-sig"))
    origin_lon = terrain["origin_lon"]
    origin_lat = terrain["origin_lat"]
    metres_per_degree = 111320.0
    local_crs = (f"+proj=eqc +lat_ts={origin_lat} +lon_0={origin_lon} "
                 f"+y_0={-origin_lat * metres_per_degree} +R={metres_per_degree * 180.0 / math.pi} +units=m +no_defs")
    width_m = (terrain["columns"] - 1) * terrain["spacing_m"]
    height_m = (terrain["rows"] - 1) * terrain["spacing_m"]
    width = math.ceil(width_m / args.metres_per_pixel)
    height = math.ceil(height_m / args.metres_per_pixel)
    if max(width, height) > 4096:
        parser.error("Runtime texture would exceed 4096 pixels; increase --metres-per-pixel.")
    affine = from_origin(terrain["grid_x"], -terrain["grid_z"], width_m / width, height_m / height)
    route = json.loads((root / "data/route_108.json").read_text(encoding="utf-8-sig"))
    coordinates = np.array([point for direction in route["directions"] for point in direction["points"]])
    with ExitStack() as stack:
        rasters, sources, manifests = inspect_sources(root, stack)
        covered = np.zeros(len(coordinates), dtype=bool)
        for raster in rasters:
            transform = Transformer.from_crs("EPSG:4326", raster.crs, always_xy=True)
            east, north = transform.transform(coordinates[:, 0], coordinates[:, 1])
            covered |= ((east >= raster.bounds.left) & (east < raster.bounds.right)
                        & (north > raster.bounds.bottom) & (north <= raster.bounds.top))
        report = {
            "schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
            "attribution": "GUGiK / Geoportal orthophotos; original download metadata retained.",
            "source_manifests": manifests, "sources": sources,
            "source_survey_dates": sorted({source["survey_date"] for source in sources if source["survey_date"]}),
            "source_mib": round(sum(source["bytes"] for source in sources) / 1048576, 2),
            "route_shape_samples": len(covered), "route_shape_samples_in_source_bounds": int(covered.sum()),
            "origin_lon": origin_lon, "origin_lat": origin_lat,
            "grid_x": terrain["grid_x"], "grid_z": terrain["grid_z"],
            "width_m": width_m, "height_m": height_m, "width": width, "height": height,
            "metres_per_pixel": [width_m / width, height_m / height],
        }
        if args.inspect_only:
            print(json.dumps(report, indent=2))
            return
        rgb = np.empty((3, height, width), dtype=np.uint8)
        rgb[:] = np.array([130, 154, 112], dtype=np.uint8)[:, None, None]
        valid = np.zeros((height, width), dtype=bool)
        for raster in rasters:
            with WarpedVRT(raster, crs=local_crs, transform=affine, width=width, height=height,
                           resampling=Resampling.bilinear, add_alpha=True) as warped:
                channels = [warped.colorinterp.index(channel) + 1 for channel in (ColorInterp.red, ColorInterp.green, ColorInterp.blue)]
                pixels = warped.read(channels)
                alpha_index = warped.colorinterp.index(ColorInterp.alpha) + 1
                mask = warped.read(alpha_index) > 0
                rgb[:, mask] = pixels[:, mask]
                valid |= mask
            print(f"Reprojected {Path(raster.name).name}")
        if not valid.any():
            raise ValueError("Imagery does not cover the terrain crop; no runtime output replaced.")
        report["valid_fraction"] = float(valid.mean())
        report["texture_file"] = "res://data/orthophoto_108.png"
        report["uncovered_color"] = "#829a70"
        report["layout"] = "Image edges match terrain mesh bounds; north at top, east at right. No height changes."
        transform = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
        east, north = transform.transform(coordinates[:, 0], coordinates[:, 1])
        expected_east = (coordinates[:, 0] - origin_lon) * metres_per_degree * math.cos(math.radians(origin_lat))
        expected_north = (coordinates[:, 1] - origin_lat) * metres_per_degree
        if not (np.allclose(east, expected_east, atol=0.001, rtol=0) and np.allclose(north, expected_north, atol=0.001, rtol=0)):
            raise ValueError("Image projection does not match the terrain coordinate convention.")
        covered = grid_coverage(coordinates, transform, affine, valid)
        report["route_shape_samples_with_imagery"] = int(covered.sum())
        report["stops_without_imagery"] = []
        for direction in route["directions"]:
            stops = direction["stops"]
            stop_coordinates = np.array([[stop["lon"], stop["lat"]] for stop in stops])
            stop_coverage = grid_coverage(stop_coordinates, transform, affine, valid)
            report["stops_without_imagery"].extend(stop["name"] for stop, present in zip(stops, stop_coverage) if not present)
        missing_coordinates = coordinates[~covered]
        suggested_sheets = []
        for filename in sorted((root / "data").glob("*.asc")):
            sheet = filename.stem.split("_", 2)[-1]
            if sheet in {source["sheet"] for source in sources} or not len(missing_coordinates):
                continue
            with rasterio.open(filename) as tile:
                tile_transform = Transformer.from_crs("EPSG:4326", terrain["source_crs"], always_xy=True)
                tile_east, tile_north = tile_transform.transform(missing_coordinates[:, 0], missing_coordinates[:, 1])
                inside = ((tile_east >= tile.bounds.left) & (tile_east <= tile.bounds.right)
                          & (tile_north >= tile.bounds.bottom) & (tile_north <= tile.bounds.top))
                if inside.any():
                    suggested_sheets.append(sheet)
        report["suggested_missing_sheets"] = suggested_sheets
        report["suggestion_basis"] = "Existing NMT sheet extents containing uncovered route points; search equivalent orthophoto sheets."
        cache = root / ".cache"
        cache.mkdir(exist_ok=True)
        image_path = root / "data/orthophoto_108.png"
        temporary_image = cache / "orthophoto_108.tmp.png"
        write_png(temporary_image, rgb)
        preview = rgb[:, ::3, ::3].copy()
        route_lines = []
        for direction in route["directions"]:
            path = np.array(direction["points"])
            path_east, path_north = transform.transform(path[:, 0], path[:, 1])
            route_lines.append({"type": "LineString", "coordinates": list(zip(path_east, path_north))})
        line_mask = rasterize(route_lines, out_shape=preview.shape[1:], transform=affine * affine.scale(3, 3),
                              fill=0, default_value=1, all_touched=True).astype(bool)
        expanded = line_mask.copy()
        expanded[1:] |= line_mask[:-1]
        expanded[:-1] |= line_mask[1:]
        expanded[:, 1:] |= line_mask[:, :-1]
        expanded[:, :-1] |= line_mask[:, 1:]
        preview[:, expanded] = np.array([255, 65, 70], dtype=np.uint8)[:, None]
        write_png(cache / "orthophoto_coverage.png", preview)
        temporary_image.replace(image_path)
        (root / "data/orthophoto_108.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Runtime: {width}x{height}, {image_path.stat().st_size / 1048576:.2f} MiB, {valid.mean():.1%} imagery coverage")
        print(f"Actual route coverage: {covered.sum()}/{len(covered)} shape points")
        print("Missing stops: " + ", ".join(report["stops_without_imagery"]))
        print("Suggested sheets: " + ", ".join(suggested_sheets))


if __name__ == "__main__":
    main()