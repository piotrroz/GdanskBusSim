import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import laspy
import numpy as np
from laspy.vlrs.known import GeoKeyDirectoryVlr, WktCoordinateSystemVlr
from pyproj import CRS, Transformer
from rasterio.features import rasterize
from rasterio.transform import from_origin
from scipy.ndimage import maximum_filter, gaussian_filter
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, LineString
from shapely.ops import split, triangulate


def header_crs(header):
    for record in header.vlrs:
        if isinstance(record, WktCoordinateSystemVlr) and record.string.strip("\x00 \r\n\t\"'"):
            return record.parse_crs()
        if isinstance(record, GeoKeyDirectoryVlr):
            crs = record.parse_crs()
            if crs:
                return crs
    return None


def source_files(folder):
    files = sorted(folder.glob("*.laz"))
    return [filename for filename in files
            if not (filename.name.endswith(".copc.laz") and filename.with_name(filename.name.replace(".copc.laz", ".laz")).exists())]


def inspect(folder):
    entries = {}
    for manifest in sorted(folder.glob("pobieracz_las_*.txt")):
        with manifest.open(encoding="utf-8-sig", newline="") as stream:
            entries.update({entry["nazwa_pliku"]: entry for entry in csv.DictReader(stream)})
    sources = []
    for filename in source_files(folder):
        with laspy.open(filename) as reader:
            header = reader.header
            crs = header_crs(header)
            sample = reader.read_points(min(100000, header.point_count))
            classes, counts = np.unique(np.asarray(sample.classification), return_counts=True)
            sources.append({
                "file": filename.name, "bytes": filename.stat().st_size,
                "points": int(header.point_count), "format": int(header.point_format.id),
                "minimum": header.mins.tolist(), "maximum": header.maxs.tolist(),
                "crs": crs.to_string() if crs else None,
                "survey_year": entries.get(filename.name, {}).get("aktualnosc_rok"),
                "sample_class_counts": {str(int(code)): int(count) for code, count in zip(classes, counts)},
                "has_rgb": "red" in header.point_format.dimension_names,
            })
            print(f"{filename.name}: {header.point_count:,} points, CRS={crs.to_string() if crs else 'undeclared'}, classes={sources[-1]['sample_class_counts']}")
    if not sources:
        raise ValueError("No LAZ files found in data/.")
    return sources


def ground_at(east, south, terrain, heights):
    horizontal = (east - terrain["grid_x"]) / terrain["spacing_m"]
    vertical = (south - terrain["grid_z"]) / terrain["spacing_m"]
    columns = np.clip(np.floor(horizontal).astype(int), 0, terrain["columns"] - 2)
    rows = np.clip(np.floor(vertical).astype(int), 0, terrain["rows"] - 2)
    horizontal = np.clip(horizontal - columns, 0, 1)
    vertical = np.clip(vertical - rows, 0, 1)
    northwest = heights[rows, columns]
    northeast = heights[rows, columns + 1]
    southwest = heights[rows + 1, columns]
    southeast = heights[rows + 1, columns + 1]
    return np.where(horizontal + vertical <= 1, northwest + (northeast - northwest) * horizontal + (southwest - northwest) * vertical,
                    southeast + (southwest - southeast) * (1 - horizontal) + (northeast - southeast) * (1 - vertical))


def scene_context(root, corridor):
    terrain = json.loads((root / "data/terrain_108.json").read_text(encoding="utf-8-sig"))
    heights = np.fromfile(root / "data/terrain_108.bin", dtype="<f4").reshape(terrain["rows"], terrain["columns"])
    origin_lon, origin_lat = terrain["origin_lon"], terrain["origin_lat"]
    metres_per_degree = 111320.0
    local_crs = (f"+proj=eqc +lat_ts={origin_lat} +lon_0={origin_lon} "
                 f"+y_0={-origin_lat * metres_per_degree} +R={metres_per_degree * 180 / math.pi} +units=m +no_defs")
    project = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
    route = json.loads((root / "data/route_108.json").read_text(encoding="utf-8-sig"))
    samples = []
    for direction in route["directions"]:
        coordinates = np.array(direction["points"])
        east, north = project.transform(coordinates[:, 0], coordinates[:, 1])
        path = np.column_stack([east, -north])
        for start, finish in zip(path[:-1], path[1:]):
            count = max(2, math.ceil(np.linalg.norm(finish - start) / 4) + 1)
            samples.extend(np.linspace(start, finish, count))
    route_tree = cKDTree(np.array(samples))
    resolution = 2.0
    width = math.ceil((terrain["columns"] - 1) * terrain["spacing_m"] / resolution)
    depth = math.ceil((terrain["rows"] - 1) * terrain["spacing_m"] / resolution)
    grid_x, grid_z = terrain["grid_x"], terrain["grid_z"]
    affine = from_origin(grid_x, -grid_z, resolution, resolution)
    map_data = json.loads((root / "data/map_108.json").read_text(encoding="utf-8-sig"))
    buildings, shapes, road_shapes = [], [], []
    for element in map_data["elements"]:
        tags = element.get("tags", {})
        geometry = element.get("geometry", [])
        if len(geometry) < 2:
            continue
        east, north = project.transform([point["lon"] for point in geometry], [point["lat"] for point in geometry])
        if "highway" in tags:
            road_shapes.append((LineString(np.column_stack([east, north])).buffer(6), 1))
        if "building" not in tags or len(geometry) < 4:
            continue
        polygon = Polygon(np.column_stack([east, -np.array(north)]))
        if not polygon.is_valid or polygon.area < 25:
            continue
        boundary = np.array(polygon.exterior.coords)
        if route_tree.query(boundary)[0].min() > corridor + 30:
            continue
        buildings.append((str(element["id"]), polygon))
        shapes.append((Polygon(np.column_stack([east, north])), len(buildings)))
    if not shapes:
        raise ValueError("No OSM buildings overlap the requested corridor.")
    labels = rasterize(shapes, out_shape=(depth, width), transform=affine, dtype="int32")
    roads = rasterize(road_shapes, out_shape=(depth, width), transform=affine, dtype="uint8")
    return terrain, heights, local_crs, route_tree, resolution, buildings, labels, roads


def roof_geometry(polygon, sample_xy, sample_z):
    center = np.array(polygon.centroid.coords[0])
    flat_height = float(np.median(sample_z))
    model = None
    rectangle = polygon.minimum_rotated_rectangle
    corners = np.array(rectangle.exterior.coords)[:4]
    if len(sample_xy) >= 25 and polygon.area / rectangle.area > 0.82 and np.ptp(sample_z) > 1.8:
        candidates = []
        for direction in [corners[1] - corners[0], corners[2] - corners[1]]:
            axis = direction / np.linalg.norm(direction)
            offsets = (sample_xy - center) @ axis
            span = max(abs(offsets.min()), abs(offsets.max()))
            if span < 2:
                continue
            fitted = least_squares(lambda parameters: parameters[0] - parameters[1] * abs(offsets - parameters[2]) - sample_z,
                                   [float(np.percentile(sample_z, 90)), 0.4, 0.0],
                                   bounds=([sample_z.min(), 0.15, -span * 0.35], [sample_z.max() + 2, 1.3, span * 0.35]), loss="soft_l1")
            residual = float(np.sqrt(np.mean(fitted.fun ** 2)))
            flat_error = float(np.sqrt(np.mean((sample_z - flat_height) ** 2)))
            if residual < 0.9 and residual < flat_error * 0.6:
                candidates.append((residual, axis, fitted.x))
        if candidates:
            model = min(candidates, key=lambda candidate: candidate[0])
    def roof_height(coordinate):
        if model is None:
            return flat_height
        _, axis, parameters = model
        return float(parameters[0] - parameters[1] * abs((coordinate - center) @ axis - parameters[2]))
    pieces = [polygon]
    if model is not None:
        _, axis, parameters = model
        ridge_center = center + axis * parameters[2]
        along = np.array([-axis[1], axis[0]]) * 1000
        pieces = list(split(polygon, LineString([ridge_center - along, ridge_center + along])).geoms)
    triangles = []
    for piece in pieces:
        for triangle in triangulate(piece):
            if piece.buffer(0.0001).covers(triangle):
                triangles.append([[float(coordinate[0]), roof_height(np.array(coordinate)), float(coordinate[1])]
                                  for coordinate in list(triangle.exterior.coords)[:3]])
    wall = []
    boundary = list(polygon.exterior.coords)
    for start, finish in zip(boundary[:-1], boundary[1:]):
        start, finish = np.array(start), np.array(finish)
        wall.append([float(start[0]), roof_height(start), float(start[1])])
        if model is not None:
            _, axis, parameters = model
            start_offset = (start - center) @ axis - parameters[2]
            end_offset = (finish - center) @ axis - parameters[2]
            if start_offset * end_offset < 0:
                crossing = start + (finish - start) * start_offset / (start_offset - end_offset)
                wall.append([float(crossing[0]), roof_height(crossing), float(crossing[1])])
    return wall, triangles, "fitted_gable" if model is not None else "measured_flat", float(model[0]) if model else None


def generate(root, report, source_crs, corridor):
    terrain, heights, local_crs, route_tree, resolution, buildings, labels, roads = scene_context(root, corridor)
    depth, width = labels.shape
    roof_sum = np.zeros(depth * width, dtype=np.float64)
    roof_count = np.zeros(depth * width, dtype=np.uint32)
    canopy = np.zeros(depth * width, dtype=np.float32)
    canopy_count = np.zeros(depth * width, dtype=np.uint32)
    ground_differences, class_counts = [], np.zeros(256, dtype=np.int64)
    grid_x, grid_z = terrain["grid_x"], terrain["grid_z"]
    for tile_index, source in enumerate(report["sources"]):
        with laspy.open(root / "data" / source["file"]) as reader:
            crs = header_crs(reader.header)
            if crs is None:
                if source_crs is None:
                    raise ValueError("Source CRS undeclared; specify --source-crs explicitly.")
                crs = CRS.from_user_input(source_crs)
            elif source_crs and not crs.equals(CRS.from_user_input(source_crs), ignore_axis_order=True):
                raise ValueError("Declared source CRS conflicts with the override.")
            source["processing_crs"] = crs.to_string()
            transform = Transformer.from_crs(crs, local_crs, always_xy=True)
            for chunk in reader.chunk_iterator(500000):
                classes = np.asarray(chunk.classification)
                class_counts += np.bincount(classes, minlength=256)
                usable = np.isin(classes, [2, 4, 5, 6]) & ~np.asarray(chunk.withheld, dtype=bool)
                if not usable.any():
                    continue
                east, north = transform.transform(np.asarray(chunk.x)[usable], np.asarray(chunk.y)[usable])
                east, south = np.asarray(east), -np.asarray(north)
                elevation = np.asarray(chunk.z)[usable]
                classes = classes[usable]
                columns = np.floor((east - grid_x) / resolution).astype(int)
                rows = np.floor((south - grid_z) / resolution).astype(int)
                inside = (columns >= 0) & (rows >= 0) & (columns < width) & (rows < depth)
                inside &= np.isfinite(elevation)
                east, south, elevation, classes, columns, rows = [values[inside] for values in [east, south, elevation, classes, columns, rows]]
                near = route_tree.query(np.column_stack([east, south]), workers=-1)[0] <= corridor
                east, south, elevation, classes, columns, rows = [values[near] for values in [east, south, elevation, classes, columns, rows]]
                floor = ground_at(east, south, terrain, heights)
                relative = elevation - floor
                ground = relative[classes == 2][::100]
                ground_differences.extend(ground.tolist())
                indices = rows * width + columns
                roof = (classes == 6) & (relative > 2) & (relative < 90) & (labels[rows, columns] > 0)
                np.add.at(roof_sum, indices[roof], elevation[roof])
                np.add.at(roof_count, indices[roof], 1)
                vegetation = np.isin(classes, [4, 5]) & (relative > 3.0) & (relative < 38.0)
                np.maximum.at(canopy, indices[vegetation], relative[vegetation])
                np.add.at(canopy_count, indices[vegetation], 1)
        print(f"Processed {tile_index + 1}/{len(report['sources'])}: {source['file']}", flush=True)
    if len(ground_differences) < 100:
        raise ValueError("Insufficient classified ground in the corridor; check CRS/coverage.")
    median_error = float(np.median(np.abs(ground_differences)))
    if median_error > 1.0:
        raise ValueError(f"LiDAR/NMT mismatch ({median_error:.2f} m median ground error); inspect CRS and vertical datum.")
    scenery = []
    occupied = (roof_count >= 3).reshape(labels.shape)
    for identifier, (osm_id, polygon) in enumerate(buildings, start=1):
        rows, columns = np.where((labels == identifier) & occupied)
        total_cells = int(np.count_nonzero(labels == identifier))
        if len(rows) < 12 or len(rows) / max(1, total_cells) < 0.55:
            continue
        indices = rows * width + columns
        sample_xy = np.column_stack([grid_x + (columns + 0.5) * resolution, grid_z + (rows + 0.5) * resolution])
        sample_z = roof_sum[indices] / roof_count[indices]
        lower, upper = np.percentile(sample_z, [5, 95])
        keep = (sample_z >= lower - 0.5) & (sample_z <= upper + 0.5)
        boundary = np.array(polygon.exterior.coords)
        base_height = float(np.max(ground_at(boundary[:, 0], boundary[:, 1], terrain, heights)))
        if np.median(sample_z) - base_height < 2.5:
            continue
        wall, triangles, kind, residual = roof_geometry(polygon, sample_xy[keep], sample_z[keep])
        if not triangles or min(vertex[1] for vertex in wall) < base_height + 2:
            continue
        scenery.append({"osm_id": osm_id, "base_height": round(base_height, 2), "roof_type": kind,
                        "roof_fit_rmse": residual, "sample_cells": len(rows),
                        "coverage": round(len(rows) / max(1, total_cells), 3),
                        "wall_top": np.round(wall, 2).tolist(), "roof_triangles": np.round(triangles, 2).tolist()})
    canopy = canopy.reshape(labels.shape)
    canopy_count = canopy_count.reshape(labels.shape)
    supported = (canopy_count >= 4) & (labels == 0) & (roads == 0)
    smoothed = gaussian_filter(np.where(supported, canopy, 0), sigma=0.8)
    peaks = supported & (smoothed > 3.0) & (smoothed == maximum_filter(smoothed, size=5))
    candidates = np.column_stack(np.where(peaks))
    candidates = sorted(candidates, key=lambda cell: (-float(smoothed[tuple(cell)]), int(cell[0]), int(cell[1])))
    trees, accepted = [], {}
    for row, column in candidates:
        east = grid_x + (column + 0.5) * resolution
        south = grid_z + (row + 0.5) * resolution
        if route_tree.query([east, south])[0] < 9.0:
            continue
        bucket = (int(east // 6), int(south // 6))
        neighbours = [point for horizontal in range(bucket[0] - 1, bucket[0] + 2) for vertical in range(bucket[1] - 1, bucket[1] + 2)
                      for point in accepted.get((horizontal, vertical), [])]
        if any((east - point[0]) ** 2 + (south - point[1]) ** 2 < 36 for point in neighbours):
            continue
        tree_height = float(canopy[row, column])
        ground = float(ground_at(np.array([east]), np.array([south]), terrain, heights)[0])
        trees.append({"position": [round(east, 2), round(ground, 2), round(south, 2)],
                      "height": round(tree_height, 2), "radius": round(float(np.clip(tree_height * 0.22, 1.6, 4.5)), 2)})
        accepted.setdefault(bucket, []).append((east, south))
    if not scenery or not trees:
        raise ValueError("No usable building/vegetation scenery; existing output unchanged.")
    report.update({"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
                   "origin_lon": terrain["origin_lon"], "origin_lat": terrain["origin_lat"],
                   "terrain_grid": {key: terrain[key] for key in ["grid_x", "grid_z", "columns", "rows", "spacing_m", "generated_at"]},
                   "attribution": "GUGiK / Geoportal LiDAR, 2018; building footprints (c) OpenStreetMap contributors, ODbL.",
                   "source_crs_override": source_crs, "corridor_m": corridor, "aggregation_m": resolution,
                   "class_counts": {str(index): int(count) for index, count in enumerate(class_counts) if count},
                   "median_absolute_ground_error_m": median_error, "ground_check_samples": len(ground_differences),
                   "building_count": len(scenery), "gable_count": sum(building["roof_type"] == "fitted_gable" for building in scenery),
                   "tree_count": len(trees), "buildings": scenery, "trees": trees,
                   "limitations": "2018 survey. Tree crown proxies, not identified species or surveyed trunks. Roof fits are approximations; unsupported OSM buildings retain original geometry. No ground/collision replacement."})
    destination = root / "data/lidar_108.json"
    temporary = root / ".cache/lidar_108.tmp.json"
    temporary.write_text(json.dumps(report, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    verify = json.loads(temporary.read_text(encoding="utf-8"))
    assert verify["tree_count"] == len(trees) and verify["building_count"] == len(scenery)
    temporary.replace(destination)
    print(f"SCENERY: {len(scenery)} measured buildings ({report['gable_count']} fitted gables), {len(trees)} canopy proxies, {destination.stat().st_size / 1048576:.2f} MiB")
    print(f"GROUND AGREEMENT: median absolute error {median_error:.3f} m over {len(ground_differences)} samples")


def main():
    parser = argparse.ArgumentParser(description="Inspect classified Geoportal LiDAR for route 108 scenery.")
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--source-crs", help="Explicit CRS for survey files with missing/blank coordinate metadata")
    parser.add_argument("--corridor", type=float, default=180.0)
    args = parser.parse_args()
    if not math.isfinite(args.corridor) or not 30 <= args.corridor <= 300:
        parser.error("Use a corridor between 30 and 300 metres.")
    root = Path(__file__).resolve().parents[1]
    sources = inspect(root / "data")
    report = {"sources": sources, "point_count": sum(source["points"] for source in sources),
              "source_mib": round(sum(source["bytes"] for source in sources) / 1048576, 2)}
    (root / ".cache").mkdir(exist_ok=True)
    (root / ".cache/lidar-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Survey: {len(sources)} unique tiles, {report['point_count']:,} points, {report['source_mib']} MiB")
    if not args.inspect_only:
        generate(root, report, args.source_crs, args.corridor)


if __name__ == "__main__":
    main()