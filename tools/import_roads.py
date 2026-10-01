import argparse
import json
import math
import shutil
import warnings
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import laspy
import numpy as np
import rasterio
from pyproj import CRS, Transformer
from scipy.ndimage import map_coordinates, median_filter
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree
from shapely.geometry import LineString, Point

from import_landmark import local_projection
from import_lidar import ground_at, header_crs, source_files
from import_orthophotos import inspect_sources, write_png


def clean_polyline(points, minimum_spacing=0.05):
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("Road path needs at least two finite XY points.")
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) >= minimum_spacing]
    cleaned = points[keep]
    if len(cleaned) < 2:
        raise ValueError("Road path collapses after duplicate removal.")
    return cleaned


def remove_short_backtracks(points, maximum_leg=3.0, minimum_turn_degrees=75.0):
    points = clean_polyline(points)
    while len(points) > 2:
        candidates = []
        for index in range(1, len(points) - 1):
            incoming = points[index] - points[index - 1]
            outgoing = points[index + 1] - points[index]
            incoming_length, outgoing_length = np.linalg.norm(incoming), np.linalg.norm(outgoing)
            angle = math.degrees(math.acos(np.clip(np.dot(incoming, outgoing) / (incoming_length * outgoing_length), -1, 1)))
            if min(incoming_length, outgoing_length) <= maximum_leg and angle >= minimum_turn_degrees:
                shortcut = LineString([points[index - 1], points[index + 1]])
                candidates.append((shortcut.distance(Point(points[index])), index))
        if not candidates:
            break
        _, remove = min(candidates)
        points = np.delete(points, remove, axis=0)
    return points


def rounded_polyline(points, spacing=0.5, maximum_trim=6.0, trim_fraction=0.48):
    points = remove_short_backtracks(points)
    dense = [points[0]]
    for index in range(1, len(points) - 1):
        incoming = points[index] - points[index - 1]
        outgoing = points[index + 1] - points[index]
        incoming_length, outgoing_length = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        incoming, outgoing = incoming / incoming_length, outgoing / outgoing_length
        trim = min(maximum_trim, incoming_length * trim_fraction, outgoing_length * trim_fraction)
        angle = math.acos(np.clip(np.dot(incoming, outgoing), -1, 1))
        if angle < math.radians(2) or trim < 0.1:
            dense.append(points[index])
            continue
        entry = points[index] - incoming * trim
        exit_point = points[index] + outgoing * trim
        if np.linalg.norm(dense[-1] - entry) > 0.01:
            dense.append(entry)
        curve_length = np.linalg.norm(entry - points[index]) + np.linalg.norm(points[index] - exit_point)
        for step in range(1, max(2, math.ceil(curve_length / (spacing * 0.35))) + 1):
            parameter = step / max(2, math.ceil(curve_length / (spacing * 0.35)))
            dense.append((1 - parameter) ** 2 * entry + 2 * (1 - parameter) * parameter * points[index] + parameter ** 2 * exit_point)
    dense.append(points[-1])
    line = LineString(clean_polyline(dense, 0.001))
    distances = np.arange(0, line.length, spacing)
    if not len(distances) or distances[-1] < line.length:
        distances = np.r_[distances, line.length]
    result = np.array([[point.x, point.y] for point in (line.interpolate(distance) for distance in distances)])
    if max(line.distance(Point(point)) for point in points) > maximum_trim * 0.55:
        raise ValueError("Corner smoothing deviated too far from the source route.")
    return result


def road_cross_sections(centerline, half_width=4.5, lateral_spacing=0.5):
    centerline = clean_polyline(centerline, 0.001)
    tangents = np.empty_like(centerline)
    tangents[0] = centerline[1] - centerline[0]
    tangents[-1] = centerline[-1] - centerline[-2]
    tangents[1:-1] = centerline[2:] - centerline[:-2]
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    offsets = np.linspace(-half_width, half_width, round(half_width * 2 / lateral_spacing) + 1)
    sections = centerline[:, None, :] + normals[:, None, :] * offsets[None, :, None]
    return sections, tangents, offsets


def collect_ground_points(root, paths, source_crs, local_crs, corridor=6.5):
    to_source = Transformer.from_crs(local_crs, source_crs, always_xy=True)
    route_local = np.concatenate(paths)
    source_east, source_north = to_source.transform(route_local[:, 0], -route_local[:, 1])
    route_tree = cKDTree(np.column_stack([source_east, source_north]))
    to_local = Transformer.from_crs(source_crs, local_crs, always_xy=True)
    records, sources = [], []
    for filename in source_files(root / "data"):
        with laspy.open(filename) as reader:
            header = reader.header
            embedded = header_crs(header)
            if embedded and not embedded.equals(CRS.from_user_input(source_crs), ignore_axis_order=True):
                raise ValueError(f"{filename.name}: embedded CRS conflicts with --source-crs.")
            corners = np.array([[header.mins[0], header.mins[1]], [header.mins[0], header.maxs[1]],
                                [header.maxs[0], header.mins[1]], [header.maxs[0], header.maxs[1]]])
            if route_tree.query(corners)[0].min() > math.dist(header.mins[:2], header.maxs[:2]) + corridor:
                continue
            retained = 0
            for chunk in reader.chunk_iterator(500000):
                classes = np.asarray(chunk.classification)
                keep = (classes == 2) & ~np.asarray(chunk.withheld, dtype=bool)
                if not keep.any():
                    continue
                east, north, elevation = np.asarray(chunk.x)[keep], np.asarray(chunk.y)[keep], np.asarray(chunk.z)[keep]
                near = route_tree.query(np.column_stack([east, north]), workers=-1)[0] <= corridor
                if not near.any():
                    continue
                local_east, local_north = to_local.transform(east[near], north[near])
                values = np.column_stack([local_east, -np.asarray(local_north), elevation[near]])
                values = values[np.isfinite(values).all(axis=1)]
                records.append(values)
                retained += len(values)
            if retained:
                sources.append({"file": filename.name, "retained_ground_returns": retained,
                                "embedded_crs": embedded.to_string() if embedded else None})
                print(f"{filename.name}: {retained:,} road-corridor ground returns", flush=True)
    if not records:
        raise ValueError("No classified ground returns found near the route.")
    points = np.concatenate(records)
    return points, sources


def sample_road_elevation(sections, ground_points, terrain, terrain_heights):
    flat = sections.reshape(-1, 2)
    fallback = ground_at(flat[:, 0], flat[:, 1], terrain, terrain_heights)
    tree = cKDTree(ground_points[:, :2])
    distances, neighbours = tree.query(flat, k=8, distance_upper_bound=1.25, workers=-1)
    finite = np.isfinite(distances)
    values = np.where(finite, ground_points[np.minimum(neighbours, len(ground_points) - 1), 2], np.nan)
    with warnings.catch_warnings(), np.errstate(all="ignore"):
        warnings.simplefilter("ignore", RuntimeWarning)
        estimate = np.nanmedian(values, axis=1)
        lower = np.nanpercentile(values, 25, axis=1)
        upper = np.nanpercentile(values, 75, axis=1)
    support = ((finite.sum(axis=1) >= 4) & (distances[:, 0] <= 0.75) & ((upper - lower) <= 0.30)
               & np.isfinite(estimate) & (np.abs(estimate - fallback) <= 0.15))
    elevations = np.where(support, estimate, fallback).reshape(sections.shape[:2])
    local_median = median_filter(elevations, size=(9, 3), mode="nearest")
    elevations = np.where(np.abs(elevations - local_median) <= 0.25, elevations, local_median)
    if elevations.shape[0] >= 41:
        elevations = savgol_filter(elevations, 41, 2, axis=0, mode="interp")
    if elevations.shape[1] >= 5:
        elevations = savgol_filter(elevations, 5, 2, axis=1, mode="interp")
    center = elevations.shape[1] // 2
    maximum_step = 0.04
    for column in range(center + 1, elevations.shape[1]):
        elevations[:, column] = elevations[:, column - 1] + np.clip(elevations[:, column] - elevations[:, column - 1], -maximum_step, maximum_step)
    for column in range(center - 1, -1, -1):
        elevations[:, column] = elevations[:, column + 1] + np.clip(elevations[:, column] - elevations[:, column + 1], -maximum_step, maximum_step)
    elevations = limit_longitudinal_grade(elevations, 0.5, 0.12)
    accepted_difference = np.where(support, estimate - fallback, np.nan)
    return elevations + 0.18, support.reshape(sections.shape[:2]), accepted_difference


def limit_longitudinal_grade(elevations, spacing, maximum_grade, iterations=5):
    values = np.asarray(elevations, dtype=np.float64).copy()
    maximum_step = spacing * maximum_grade
    for _ in range(iterations):
        forward = values.copy()
        for row in range(1, len(forward)):
            forward[row] = np.clip(forward[row], forward[row - 1] - maximum_step, forward[row - 1] + maximum_step)
        backward = values.copy()
        for row in range(len(backward) - 2, -1, -1):
            backward[row] = np.clip(backward[row], backward[row + 1] - maximum_step, backward[row + 1] + maximum_step)
        values = (forward + backward) * 0.5
    return values


def vertex_normals(positions):
    longitudinal = np.empty_like(positions)
    lateral = np.empty_like(positions)
    longitudinal[0] = positions[1] - positions[0]
    longitudinal[-1] = positions[-1] - positions[-2]
    longitudinal[1:-1] = positions[2:] - positions[:-2]
    lateral[:, 0] = positions[:, 1] - positions[:, 0]
    lateral[:, -1] = positions[:, -1] - positions[:, -2]
    lateral[:, 1:-1] = positions[:, 2:] - positions[:, :-2]
    normals = np.cross(longitudinal, lateral)
    normals /= np.maximum(np.linalg.norm(normals, axis=2, keepdims=True), 1e-9)
    normals[normals[:, :, 1] < 0] *= -1
    return normals


def sample_imagery(rasters, descriptions, local_crs, east, south):
    shape = east.shape
    pixels = np.full((3, east.size), 90, dtype=np.uint8)
    covered = np.zeros(east.size, dtype=bool)
    east, south = east.ravel(), south.ravel()
    for raster, description in zip(rasters, descriptions):
        transform = Transformer.from_crs(local_crs, raster.crs, always_xy=True)
        source_east, source_north = transform.transform(east, -south)
        inverse = ~raster.transform
        columns, rows = inverse * (source_east, source_north)
        valid = ((columns >= 0) & (rows >= 0) & (columns < raster.width - 1) & (rows < raster.height - 1))
        if not valid.any():
            continue
        minimum_column, maximum_column = math.floor(columns[valid].min()), math.ceil(columns[valid].max()) + 1
        minimum_row, maximum_row = math.floor(rows[valid].min()), math.ceil(rows[valid].max()) + 1
        window = rasterio.windows.Window(minimum_column, minimum_row, maximum_column - minimum_column + 1, maximum_row - minimum_row + 1)
        channels = [raster.colorinterp.index(channel) + 1 for channel in
                    (rasterio.enums.ColorInterp.red, rasterio.enums.ColorInterp.green, rasterio.enums.ColorInterp.blue)]
        source = raster.read(channels, window=window)
        coordinates = np.vstack([rows[valid] - minimum_row, columns[valid] - minimum_column])
        for band in range(3):
            pixels[band, valid] = np.clip(map_coordinates(source[band], coordinates, order=1, mode="nearest"), 0, 255).astype(np.uint8)
        covered[valid] = True
    return pixels.reshape((3,) + shape), covered.reshape(shape)


def write_direction(root, staging, direction, centerline, sections, elevations, support, rasters, imagery_descriptions, local_crs):
    positions = np.empty(sections.shape[:2] + (3,), dtype=np.float32)
    positions[:, :, 0] = sections[:, :, 0]
    positions[:, :, 1] = elevations
    positions[:, :, 2] = sections[:, :, 1]
    normals = vertex_normals(positions)
    distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(centerline, axis=0), axis=1))]
    chunks, texture_pixels, texture_covered = [], 0, 0
    chunk_length = 256.0
    boundaries = list(np.arange(0, distances[-1], chunk_length)) + [distances[-1]]
    for chunk_index in range(len(boundaries) - 1):
        start = np.searchsorted(distances, boundaries[chunk_index], side="left")
        end = np.searchsorted(distances, boundaries[chunk_index + 1], side="right")
        if chunk_index and start > 0:
            start -= 1
        chunk_positions = positions[start:end]
        chunk_normals = normals[start:end]
        binary = np.concatenate([chunk_positions, chunk_normals], axis=2).astype("<f4")
        binary_name = f"{direction['shape_id']}_{chunk_index:02d}.bin"
        (staging / binary_name).write_bytes(binary.tobytes())
        start_distance, end_distance = distances[start], distances[end - 1]
        texture_height = max(2, math.ceil((end_distance - start_distance) / 0.25) + 1)
        texture_width = 37
        texture_distances = np.linspace(start_distance, end_distance, texture_height)
        indices = np.searchsorted(distances, texture_distances, side="right") - 1
        indices = np.clip(indices, 0, len(centerline) - 2)
        fractions = (texture_distances - distances[indices]) / np.maximum(distances[indices + 1] - distances[indices], 1e-6)
        texture_center = centerline[indices] * (1 - fractions[:, None]) + centerline[indices + 1] * fractions[:, None]
        tangent = centerline[indices + 1] - centerline[indices]
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        offsets = np.linspace(-4.5, 4.5, texture_width)
        texture_coordinates = texture_center[:, None, :] + normal[:, None, :] * offsets[None, :, None]
        image, covered = sample_imagery(rasters, imagery_descriptions, local_crs,
                                         texture_coordinates[:, :, 0], texture_coordinates[:, :, 1])
        texture_name = f"{direction['shape_id']}_{chunk_index:02d}.png"
        write_png(staging / texture_name, image)
        texture_pixels += covered.size
        texture_covered += int(covered.sum())
        chunks.append({"position_file": "res://data/roads_108/" + binary_name,
                       "texture_file": "res://data/roads_108/" + texture_name,
                   "sections": int(end - start), "columns": int(sections.shape[1]), "stride": 6,
                       "start_distance_m": float(start_distance), "end_distance_m": float(end_distance),
                       "lidar_supported_fraction": float(support[start:end].mean())})
    return {"id": direction["id"], "shape_id": direction["shape_id"], "headsign": direction["headsign"],
            "sections": int(len(centerline)), "columns": int(sections.shape[1]), "length_m": float(distances[-1]),
            "spacing_m": 0.5, "lateral_spacing_m": 0.5, "half_width_m": 4.5, "surface_lift_m": 0.18,
            "vertex_count": int(positions.shape[0] * positions.shape[1]),
            "triangle_count": int((positions.shape[0] - 1) * (positions.shape[1] - 1) * 2),
            "lidar_supported_fraction": float(support.mean()),
            "texture_coverage": texture_covered / texture_pixels, "chunks": chunks}


def generate(root, route, terrain, source_crs):
    terrain_heights = np.fromfile(root / "data/terrain_108.bin", dtype="<f4").reshape(terrain["rows"], terrain["columns"])
    local_crs = local_projection(terrain)
    project = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
    prepared = []
    for direction in route["directions"]:
        coordinates = np.asarray(direction["points"])
        east, north = project.transform(coordinates[:, 0], coordinates[:, 1])
        centerline = rounded_polyline(np.column_stack([east, -np.asarray(north)]))
        sections, _, _ = road_cross_sections(centerline)
        prepared.append((direction, centerline, sections))
    ground_points, lidar_sources = collect_ground_points(root, [item[1] for item in prepared], source_crs, local_crs)
    staging = root / ".cache/roads-staging"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    directions, quality = [], []
    with ExitStack() as stack:
        rasters, imagery_descriptions, manifests = inspect_sources(root, stack)
        for direction, centerline, sections in prepared:
            elevations, support, difference = sample_road_elevation(sections, ground_points, terrain, terrain_heights)
            supported_difference = difference[np.isfinite(difference)]
            grade = np.abs(np.diff(elevations[:, elevations.shape[1] // 2])) / 0.5
            crossfall = np.abs(elevations[:, -1] - elevations[:, 0]) / 9.0
            metrics = {"shape_id": direction["shape_id"], "ground_return_count": int(len(ground_points)),
                       "lidar_supported_fraction": float(support.mean()),
                       "median_absolute_lidar_nmt_difference_m": float(np.median(np.abs(supported_difference))),
                       "p95_absolute_lidar_nmt_difference_m": float(np.percentile(np.abs(supported_difference), 95)),
                       "maximum_centerline_grade": float(grade.max()), "p99_crossfall": float(np.percentile(crossfall, 99))}
            if (metrics["lidar_supported_fraction"] < 0.60 or metrics["median_absolute_lidar_nmt_difference_m"] > 0.5
                    or metrics["maximum_centerline_grade"] > 0.18 or metrics["p99_crossfall"] > 0.08):
                raise ValueError(f"{direction['shape_id']}: insufficient or misaligned LiDAR road support: {metrics}")
            directions.append(write_direction(root, staging, direction, centerline, sections, elevations, support,
                                              rasters, imagery_descriptions, local_crs))
            quality.append(metrics)
            print(json.dumps(metrics, indent=2), flush=True)
    metadata = {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
                "origin_lon": terrain["origin_lon"], "origin_lat": terrain["origin_lat"],
                "terrain_grid": {key: terrain[key] for key in ["grid_x", "grid_z", "columns", "rows", "spacing_m", "generated_at"]},
                "source_crs_override": source_crs, "lidar_sources": lidar_sources,
                "imagery_manifests": manifests, "quality": quality, "directions": directions,
                "attribution": "ZTM Gdansk GTFS route shape / CC BY; GUGiK Geoportal LiDAR 2018 and orthophotos 2021.",
                "limitations": "Nine-metre route-driving strips, not surveyed carriageway boundaries. LiDAR class-2 robust elevations with NMT fallback; 25cm curvilinear imagery includes baked objects/shadows. Bridges require explicit deck review."}
    (staging / "roads_108.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    destination = root / "data/roads_108"
    backup = root / ".cache/roads-backup"
    if backup.exists():
        shutil.rmtree(backup)
    if destination.exists():
        destination.replace(backup)
    try:
        staging.replace(destination)
    except OSError:
        if backup.exists() and not destination.exists():
            backup.replace(destination)
        raise
    if backup.exists():
        shutil.rmtree(backup)
    print(f"ROAD OUTPUT: {len(directions)} directions, {sum(item['vertex_count'] for item in directions):,} vertices, {sum(len(item['chunks']) for item in directions)} chunks")


def main():
    parser = argparse.ArgumentParser(description="Generate smooth, LiDAR-supported route road surfaces.")
    parser.add_argument("--source-crs", default="EPSG:2180")
    parser.add_argument("--inspect-only", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    route = json.loads((root / "data/route_108.json").read_text(encoding="utf-8-sig"))
    terrain = json.loads((root / "data/terrain_108.json").read_text(encoding="utf-8-sig"))
    project = Transformer.from_crs("EPSG:4326", local_projection(terrain), always_xy=True)
    report = []
    for direction in route["directions"]:
        coordinates = np.asarray(direction["points"])
        east, north = project.transform(coordinates[:, 0], coordinates[:, 1])
        source = np.column_stack([east, -np.asarray(north)])
        smooth = rounded_polyline(source)
        sections, tangents, offsets = road_cross_sections(smooth)
        turns = np.degrees(np.arccos(np.clip(np.sum(tangents[:-1] * tangents[1:], axis=1), -1, 1)))
        source_line, smooth_line = LineString(clean_polyline(source)), LineString(smooth)
        cleaned = remove_short_backtracks(source)
        segments = np.diff(cleaned, axis=0)
        normals = np.column_stack([-segments[:, 1], segments[:, 0]]) / np.linalg.norm(segments, axis=1)[:, None]
        old_gaps = np.linalg.norm((cleaned[1:-1] + normals[:-1] * 4.5) - (cleaned[1:-1] + normals[1:] * 4.5), axis=1)
        item = {"shape_id": direction["shape_id"], "source_points": int(len(source)), "sections": int(len(smooth)),
            "columns": int(len(offsets)), "length_m": smooth_line.length,
                "maximum_source_deviation_m": max(smooth_line.distance(Point(point)) for point in clean_polyline(source)),
                "maximum_turn_per_section_deg": float(turns.max()),
            "maximum_section_edge_length_m": float(np.max(np.linalg.norm(sections[1:, 0] - sections[:-1, 0], axis=1))),
            "shared_join_gap_m": 0.0, "old_join_gap_baseline_m": float(old_gaps.max())}
        report.append(item)
        print(json.dumps(item, indent=2))
    (root / ".cache").mkdir(exist_ok=True)
    (root / ".cache/road-geometry-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if not args.inspect_only:
        generate(root, route, terrain, args.source_crs)


if __name__ == "__main__":
    main()