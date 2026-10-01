import argparse
import hashlib
import json
import math
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import laspy
import numpy as np
from pyproj import CRS, Transformer
from scipy.spatial import Delaunay, cKDTree
from rasterio.enums import ColorInterp, Resampling
from rasterio.transform import from_origin
from rasterio.vrt import WarpedVRT
from shapely import contains_xy, covers, polygons
from shapely.geometry import Point, Polygon

from import_lidar import ground_at, header_crs, source_files
from import_orthophotos import inspect_sources, write_png


def local_projection(terrain):
    metres_per_degree = 111320.0
    return (f"+proj=eqc +lat_ts={terrain['origin_lat']} +lon_0={terrain['origin_lon']} "
            f"+y_0={-terrain['origin_lat'] * metres_per_degree} "
            f"+R={metres_per_degree * 180 / math.pi} +units=m +no_defs")


def select_building(root, northing, easting, source_crs):
    data = json.loads((root / "data/map_108.json").read_text(encoding="utf-8-sig"))
    project = Transformer.from_crs("EPSG:4326", source_crs, always_xy=True)
    target = Point(easting, northing)
    matches = []
    for element in data["elements"]:
        if "building" not in element.get("tags", {}) or len(element.get("geometry", [])) < 4:
            continue
        polygon = Polygon([project.transform(point["lon"], point["lat"]) for point in element["geometry"]])
        if polygon.is_valid and polygon.covers(target):
            matches.append((polygon.area, element, polygon))
    if not matches:
        raise ValueError("No building contains the coordinate. Use northing, easting order and the correct CRS.")
    _, element, footprint = min(matches, key=lambda match: match[0])
    return element, footprint


def extract_points(root, footprint, source_crs, local_crs, terrain, heights):
    minimum_east, minimum_north, maximum_east, maximum_north = footprint.bounds
    records, sources = [], []
    transform = Transformer.from_crs(source_crs, local_crs, always_xy=True)
    for filename in source_files(root / "data"):
        with laspy.open(filename) as reader:
            header = reader.header
            crs = header_crs(header)
            if crs and not crs.equals(CRS.from_user_input(source_crs), ignore_axis_order=True):
                raise ValueError(f"{filename.name}: source CRS disagrees with explicit override.")
            if header.maxs[0] < minimum_east or header.mins[0] > maximum_east or header.maxs[1] < minimum_north or header.mins[1] > maximum_north:
                continue
            retained = 0
            for chunk in reader.chunk_iterator(500000):
                east, north = np.asarray(chunk.x), np.asarray(chunk.y)
                keep = ((np.asarray(chunk.classification) == 6) & ~np.asarray(chunk.withheld, dtype=bool)
                        & (east >= minimum_east) & (east <= maximum_east)
                        & (north >= minimum_north) & (north <= maximum_north))
                if not keep.any():
                    continue
                east, north, elevation = east[keep], north[keep], np.asarray(chunk.z)[keep]
                inside = contains_xy(footprint, east, north)
                east, north, elevation = east[inside], north[inside], elevation[inside]
                local_east, local_north = transform.transform(east, north)
                local_east, local_south = np.asarray(local_east), -np.asarray(local_north)
                relative = elevation - ground_at(local_east, local_south, terrain, heights)
                valid = np.isfinite(elevation) & (relative > 2) & (relative < 100)
                records.append(np.column_stack([local_east[valid], local_south[valid], elevation[valid]]))
                retained += int(valid.sum())
            sources.append({"file": filename.name, "retained_building_returns": retained,
                            "embedded_crs": crs.to_string() if crs else None})
            print(f"{filename.name}: {retained:,} target building returns", flush=True)
    if not records or sum(len(record) for record in records) < 100:
        raise ValueError("Insufficient classified building returns at this footprint; existing landmark unchanged.")
    return np.concatenate(records), sources


def detailed_roof(footprint, samples, max_edge=2.0):
    unique_xy, inverse = np.unique(samples[:, :2], axis=0, return_inverse=True)
    elevation = np.full(len(unique_xy), -np.inf)
    np.maximum.at(elevation, inverse, samples[:, 2])
    tree = cKDTree(unique_xy)
    spacing = float(np.median(tree.query(unique_xy, k=2)[0][:, 1]))
    boundary = []
    for start, finish in zip(list(footprint.exterior.coords)[:-1], list(footprint.exterior.coords)[1:]):
        count = max(1, math.ceil(math.dist(start, finish) / max(spacing, 0.2)))
        boundary.extend(np.linspace(start, finish, count, endpoint=False))
    boundary = np.array(boundary)
    distances, nearest = tree.query(boundary)
    if np.quantile(distances, 0.95) > 2.0:
        raise ValueError("Too little edge coverage for a detailed landmark; refusing to stretch missing geometry.")
    boundary_z = elevation[nearest]
    combined = np.column_stack([np.concatenate([unique_xy, boundary]), np.concatenate([elevation, boundary_z])])
    xy, inverse = np.unique(combined[:, :2], axis=0, return_inverse=True)
    elevations = np.full(len(xy), -np.inf)
    np.maximum.at(elevations, inverse, combined[:, 2])
    faces = Delaunay(xy).simplices
    triangles = xy[faces]
    longest_edge = np.maximum.reduce([np.linalg.norm(triangles[:, index] - triangles[:, (index + 1) % 3], axis=1) for index in range(3)])
    inside = covers(footprint.buffer(0.0001), polygons(triangles))
    keep = inside & (longest_edge <= max_edge)
    indices = faces[keep]
    supported_area = sum(polygon.area for polygon in polygons(xy[indices]))
    coverage = float(supported_area / footprint.area)
    if coverage < 0.9:
        raise ValueError(f"Only {coverage:.1%} roof coverage at max edge {max_edge:g}m; source gaps need inspection.")
    vertices = np.column_stack([xy[:, 0], elevations, xy[:, 1]])
    wall = np.column_stack([boundary[:, 0], boundary_z, boundary[:, 1]])
    stats = {"input_returns": len(samples), "unique_source_xy": len(unique_xy),
             "duplicate_xy_removed": len(samples) - len(unique_xy),
             "median_source_spacing_m": spacing, "source_returns_per_m2": len(samples) / footprint.area,
             "synthetic_boundary_vertices": len(boundary), "boundary_p95_distance_m": float(np.quantile(distances, 0.95)),
             "vertex_count": len(vertices), "triangle_count": len(indices), "roof_area_coverage": coverage,
             "max_triangle_edge_m": max_edge, "rejected_triangles": int((~keep).sum()),
             "roof_min_elevation": float(elevations.min()), "roof_max_elevation": float(elevations.max())}
    return vertices, indices, wall, stats


def roof_texture(root, footprint, local_crs, osm_id):
    minimum_east, minimum_south, maximum_east, maximum_south = footprint.bounds
    resolution = 0.25
    left = math.floor((minimum_east - 2) / resolution) * resolution
    top = math.floor((minimum_south - 2) / resolution) * resolution
    width = math.ceil((maximum_east + 2 - left) / resolution)
    height = math.ceil((maximum_south + 2 - top) / resolution)
    if max(width, height) > 4096:
        raise ValueError("Native landmark roof texture exceeds 4096 pixels; select a smaller building.")
    affine = from_origin(left, -top, resolution, resolution)
    pixels = np.zeros((3, height, width), dtype=np.uint8)
    valid = np.zeros((height, width), dtype=bool)
    used = []
    with ExitStack() as stack:
        rasters, descriptions, _ = inspect_sources(root, stack)
        for raster, description in zip(rasters, descriptions):
            with WarpedVRT(raster, crs=local_crs, transform=affine, width=width, height=height,
                           resampling=Resampling.bilinear, add_alpha=True) as warped:
                mask = warped.read(warped.colorinterp.index(ColorInterp.alpha) + 1) > 0
                if not mask.any():
                    continue
                channels = [warped.colorinterp.index(channel) + 1 for channel in (ColorInterp.red, ColorInterp.green, ColorInterp.blue)]
                colors = warped.read(channels)
                pixels[:, mask] = colors[:, mask]
                valid |= mask
                used.append({"file": description["file"], "survey_date": description["survey_date"], "resolution": description["resolution"]})
    if not valid.all():
        raise ValueError("Native landmark texture has missing imagery; existing landmark unchanged.")
    temporary = root / ".cache" / f"landmark_{osm_id}.png"
    write_png(temporary, pixels)
    destination = root / "data" / f"landmark_{osm_id}.png"
    return {"file": f"res://data/{destination.name}", "grid_x": left, "grid_z": top,
            "width_m": width * resolution, "height_m": height * resolution,
            "width": width, "height": height, "metres_per_pixel": resolution, "sources": used}


def resolve_selection(root, selection):
    element, footprint = select_building(root, selection["northing"], selection["easting"], selection["source_crs"])
    if selection.get("expected_osm_id") and str(element["id"]) != str(selection["expected_osm_id"]):
        raise ValueError(f"{selection['key']}: coordinate resolved to unexpected OSM building {element['id']}.")
    return element, footprint


def input_fingerprint(root, selection, element):
    digest = hashlib.sha256(json.dumps({"selection": selection, "building": element}, sort_keys=True).encode())
    for relative in ["tools/import_landmark.py", "tools/import_lidar.py", "tools/import_orthophotos.py", "data/terrain_108.json", "data/terrain_108.bin"]:
        digest.update((root / relative).read_bytes())
    inputs = source_files(root / "data") + sorted((root / "data").glob("*.tif")) + sorted((root / "data").glob("pobieracz_*.txt"))
    for filename in inputs:
        stat = filename.stat()
        digest.update(f"{filename.name}:{stat.st_size}:{stat.st_mtime_ns}".encode())
    return digest.hexdigest()


def registry_path(root):
    return root / "data/landmarks_108.json"


def read_registry(root):
    path = registry_path(root)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {"schema_version": 1, "landmarks": []}


def publish_landmark(root, record, staged_texture):
    destination = registry_path(root)
    registry = read_registry(root)
    registry["landmarks"] = [item for item in registry["landmarks"] if str(item["osm_id"]) != record["osm_id"]] + [record]
    temporary = root / ".cache/landmarks_108.tmp.json"
    temporary.write_text(json.dumps(registry, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    texture = root / record["roof_texture"]["file"].removeprefix("res://")
    previous_texture = texture.read_bytes() if texture.exists() else None
    try:
        staged_texture.replace(texture)
        temporary.replace(destination)
    except OSError:
        if previous_texture is not None:
            texture.write_bytes(previous_texture)
        elif texture.exists():
            texture.unlink()
        raise


def build_landmark(root, selection, inspect_only=False, force=False):
    northing, easting, source_crs = selection["northing"], selection["easting"], selection["source_crs"]
    element, footprint = resolve_selection(root, selection)
    fingerprint = input_fingerprint(root, selection, element)
    previous = next((item for item in read_registry(root)["landmarks"] if str(item["osm_id"]) == str(element["id"])), None)
    if not inspect_only and not force and previous and previous.get("input_fingerprint") == fingerprint:
        image = root / previous["roof_texture"]["file"].removeprefix("res://")
        if image.exists() and hashlib.sha256(image.read_bytes()).hexdigest() == previous.get("texture_sha256"):
            print(f"UNCHANGED {selection['key']}: OSM {element['id']} (use --force to rebuild)")
            return
    print(f"Target: {element['tags'].get('name', 'unnamed')} / OSM {element['id']} / {footprint.area:.1f} m2", flush=True)
    terrain = json.loads((root / "data/terrain_108.json").read_text(encoding="utf-8-sig"))
    heights = np.fromfile(root / "data/terrain_108.bin", dtype="<f4").reshape(terrain["rows"], terrain["columns"])
    local_crs = local_projection(terrain)
    samples, sources = extract_points(root, footprint, source_crs, local_crs, terrain, heights)
    transform = Transformer.from_crs(source_crs, local_crs, always_xy=True)
    edge = np.array(footprint.exterior.coords)
    east, north = transform.transform(edge[:, 0], edge[:, 1])
    local_footprint = Polygon(np.column_stack([east, -np.array(north)]))
    vertices, indices, wall, stats = detailed_roof(local_footprint, samples)
    print(json.dumps(stats, indent=2))
    (root / ".cache").mkdir(exist_ok=True)
    np.savez_compressed(root / ".cache" / f"landmark_{element['id']}_source.npz", samples=samples)
    if inspect_only:
        return
    local_edge = np.array(local_footprint.exterior.coords)
    base = float(np.max(ground_at(local_edge[:, 0], local_edge[:, 1], terrain, heights)))
    texture = roof_texture(root, local_footprint, local_crs, element["id"])
    staged_texture = root / ".cache" / f"landmark_{element['id']}.png"
    record = {"osm_id": str(element["id"]), "selection_key": selection["key"], "name": element["tags"].get("name", selection.get("name", "Landmark")),
              "input_fingerprint": fingerprint, "texture_sha256": hashlib.sha256(staged_texture.read_bytes()).hexdigest(),
              "roof_type": "native_lidar_tin", "base_height": round(base, 3),
              "roof_vertices": np.round(vertices, 3).tolist(), "roof_indices": indices.ravel().tolist(),
              "wall_top": np.round(wall, 3).tolist(), "sampling": stats, "sources": sources,
              "roof_texture": texture,
              "source_crs_override": source_crs, "coordinate_order": "northing,easting",
              "target": [northing, easting], "origin_lon": terrain["origin_lon"], "origin_lat": terrain["origin_lat"],
              "terrain_grid": {key: terrain[key] for key in ["grid_x", "grid_z", "columns", "rows", "spacing_m", "generated_at"]},
              "generated_at": datetime.now(timezone.utc).isoformat(),
              "attribution": "GUGiK / Geoportal LiDAR 2018; OSM building footprint (c) OpenStreetMap contributors / ODbL.",
              "limitations": "2.5D roof envelope from class 6 returns, no voxel averaging or gable fitting. Duplicate XY keeps highest return. Boundary heights use nearest returns. Facades/occluded surfaces are not scanned reconstruction."}
    publish_landmark(root, record, staged_texture)
    print(f"Landmark saved: {registry_path(root).stat().st_size / 1048576:.2f} MiB; other buildings unchanged")


def load_selections(path, selected=None):
    configuration = json.loads(path.read_text(encoding="utf-8-sig"))
    if configuration.get("schema_version") != 1:
        raise ValueError("Unsupported landmark selection schema.")
    selections = []
    keys = set()
    for entry in configuration["landmarks"]:
        entry = {"source_crs": configuration["source_crs"], **entry}
        key = entry.get("key", "")
        if not key or any(character not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for character in key) or key in keys:
            raise ValueError("Selection keys must be unique lowercase identifiers.")
        if not all(math.isfinite(float(entry[coordinate])) for coordinate in ["northing", "easting"]):
            raise ValueError(f"{key}: coordinates must be finite.")
        CRS.from_user_input(entry["source_crs"])
        keys.add(key)
        selections.append(entry)
    if selected and not set(selected) <= keys:
        raise ValueError("Unknown landmark selection: " + ", ".join(sorted(set(selected) - keys)))
    return [entry for entry in selections if not selected or entry["key"] in selected]


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Build selected landmarks at native LiDAR point density.")
    parser.add_argument("--config", type=Path, default=root / "data/landmark_selections.json")
    parser.add_argument("--select", action="append", help="Build one named selection; repeat for several. Default: all configured landmarks.")
    parser.add_argument("--list", action="store_true", help="Resolve configured coordinates to OSM footprints without decoding LAZ.")
    parser.add_argument("--northing", type=float)
    parser.add_argument("--easting", type=float)
    parser.add_argument("--source-crs")
    parser.add_argument("--inspect-only", action="store_true")
    parser.add_argument("--force", action="store_true", help="Rebuild selected landmarks even when sources and settings are unchanged.")
    args = parser.parse_args()
    if args.northing is not None or args.easting is not None or args.source_crs is not None:
        if args.select or any(value is None for value in [args.northing, args.easting, args.source_crs]):
            parser.error("Supply all three coordinate arguments, without --select, or use the configuration.")
        if not math.isfinite(args.northing) or not math.isfinite(args.easting):
            parser.error("Coordinates must be finite.")
        selections = [{"key": "coordinate", "northing": args.northing, "easting": args.easting, "source_crs": args.source_crs}]
    else:
        selections = load_selections(args.config, args.select)
    failures = []
    for selection in selections:
        try:
            if args.list:
                element, footprint = resolve_selection(root, selection)
                print(f"{selection['key']}: {element['tags'].get('name', 'unnamed')} / OSM {element['id']} / {footprint.area:.1f} m2")
            else:
                build_landmark(root, selection, args.inspect_only, args.force)
        except (ValueError, OSError) as error:
            failures.append(selection["key"])
            print(f"FAILED {selection['key']}: {error}", flush=True)
    if failures:
        raise SystemExit("Failed selections (previous outputs retained): " + ", ".join(failures))


if __name__ == "__main__":
    main()