"""Build the drivable road network around route 108 from OSM carriageways and the 1 m NMT.

Surfaces are engineered rather than scanned: heights come from a road-masked, outlier-filtered
smoothing of the bare-earth NMT, and bridge decks get a fitted vertical profile with clearance.
"""
import argparse
import hashlib
import json
import math
import shutil
from collections import defaultdict
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import laspy
import numpy as np
import rasterio
import shapely
from affine import Affine
from pyproj import Transformer
from rasterio.enums import MergeAlg, Resampling
from rasterio.features import rasterize
from rasterio.merge import merge
from scipy.ndimage import binary_dilation, distance_transform_edt, gaussian_filter, gaussian_filter1d, map_coordinates, median_filter
from shapely.geometry import LineString, MultiLineString, Point
import shapely.affinity
import shapely.ops
from shapely.ops import linemerge

from import_landmark import local_projection
from import_lidar import source_files
from import_orthophotos import write_png

RASTER_CELL = 1.0
MESH_CELL = 2.0
CORRIDOR_M = 150.0
SURFACE_LIFT = 0.03
SMOOTHING_SIGMA_M = 3.0
OUTLIER_M = 0.25
CLOSING_M = 3.0
ROUTE_HALF_WIDTH = 3.5
VERGE_MAX_M = 6.0
TILE_M = 128.0
BRIDGE_RAIL_CLEARANCE_M = 6.8
BRIDGE_ROAD_CLEARANCE_M = 4.7
TUNNEL_COVER_M = 5.6
TUNNEL_HEIGHT_M = 4.9
LAYER_BLEND_M = 12.0
DECK_CLASSES = (0, 1, 6, 17)
BRIDGE_VOID_M = 2.0
MAXIMUM_GRADE = 0.06
ROAD_STRIDE = 12
VERGE_STRIDE = 4
DRIVABLE = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
            "service", "busway", "road", "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
MAJOR = {"motorway", "trunk", "primary", "secondary", "tertiary"}
EDGE_LINES, CENTRE_LINE, SOLID_CENTRE = 2, 4, 8


def number(value):
    if value is None:
        return None
    try:
        return float(str(value).split(";")[0].replace(",", ".").replace("m", "").strip())
    except ValueError:
        return None


def road_profile(tags):
    """Carriageway width, lane count and marking flags from OSM tags; None for non-drivable ways."""
    highway = tags.get("highway")
    tunnel = tags.get("tunnel") in ("yes", "culvert")
    if highway not in DRIVABLE or tags.get("area") == "yes" or tags.get("tunnel") not in (None, "no", "yes"):
        return None
    base = highway.removesuffix("_link")
    link = highway.endswith("_link")
    roundabout = tags.get("junction") in ("roundabout", "circular")
    oneway = tags.get("oneway") in ("yes", "1", "-1", "true") or roundabout
    lanes = number(tags.get("lanes"))
    lanes = int(round(lanes)) if lanes and 1 <= lanes <= 8 else None
    if lanes is None:
        if base == "service" or link or roundabout:
            lanes = 1
        elif base in ("motorway", "trunk", "primary", "secondary"):
            lanes = 2
        else:
            lanes = 1 if oneway else 2
    lane_width = {"motorway": 3.75, "trunk": 3.5, "primary": 3.5, "secondary": 3.25, "tertiary": 3.0}.get(base, 2.75)
    if base == "service":
        width = {"driveway": 3.0, "parking_aisle": 5.5, "alley": 3.0, "drive-through": 3.0}.get(tags.get("service"), 4.0)
    elif base == "living_street":
        width = 4.5
    else:
        width = lanes * lane_width + (0.5 if base in MAJOR else 0.0)
    if roundabout:
        width = max(width, 6.0)
    explicit = number(tags.get("width"))
    if explicit and 2.0 <= explicit <= 40.0:
        width = explicit
    flags = 1 if oneway else 0
    if base in MAJOR or roundabout:
        flags |= EDGE_LINES
    if not oneway and base in MAJOR and lanes >= 2:
        flags |= CENTRE_LINE | (SOLID_CENTRE if base in ("motorway", "trunk", "primary") else 0)
    bridge = tags.get("bridge") not in (None, "no")
    layer = number(tags.get("layer"))
    return {"highway": highway, "width": float(width), "lanes": int(lanes), "oneway": oneway, "flags": flags,
            "bridge": bridge, "tunnel": tunnel, "layer": int(layer) if layer is not None else (1 if bridge else -1 if tunnel else 0)}


def normalized_smoothing(values, mask, sigma):
    """Gaussian smoothing that only averages masked cells, so road edges are not pulled toward verges."""
    weights = gaussian_filter(mask.astype(np.float32), sigma)
    total = gaussian_filter(np.where(mask, values, 0).astype(np.float32), sigma)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(weights > 1e-3, total / weights, np.nan).astype(np.float32)


def upper_hull(s, h):
    """Upper concave envelope of a profile, evaluated at s: spans dips such as tracks under a bridge."""
    order = np.argsort(s, kind="stable")
    hull = []
    for index in order:
        while len(hull) >= 2:
            (s0, h0), (s1, h1) = (s[hull[-2]], h[hull[-2]]), (s[hull[-1]], h[hull[-1]])
            if (s1 - s0) * (h[index] - h0) - (h1 - h0) * (s[index] - s0) >= 0:
                hull.pop()
            else:
                break
        hull.append(index)
    return np.interp(s, s[hull], h[hull])


def road_field(measured, valid_cells, roads, raster, piece_length=100.0, sigma=SMOOTHING_SIGMA_M, reach=3.0, agreement=0.3):
    """Smooth each carriageway from its own NMT cells, then blend roads that agree in height.

    Roads at different levels that touch in plan (ramps into cuttings, slip roads beside embankments) keep
    their own surfaces: where fields disagree the nearest centre line wins, so the step stays a sharp edge.
    """
    pieces = []
    for road in roads:
        line = road["line"]
        count = max(1, math.ceil(line.length / piece_length))
        for index in range(count):
            part = shapely.ops.substring(line, index * line.length / count, (index + 1) * line.length / count)
            if part.length > 0.01:
                pieces.append((part, road["width"] / 2))
    margin = int(math.ceil(4 * sigma + reach + 2))

    def piece_window(part, half_width):
        minx, minz, maxx, maxz = part.bounds
        c0 = max(int((minx - raster.x0) / raster.cell) - margin - int(half_width), 0)
        r0 = max(int((minz - raster.z0) / raster.cell) - margin - int(half_width), 0)
        c1 = min(int((maxx - raster.x0) / raster.cell) + margin + int(half_width) + 1, raster.columns)
        r1 = min(int((maxz - raster.z0) / raster.cell) + margin + int(half_width) + 1, raster.rows)
        window = (slice(r0, r1), slice(c0, c1))
        transform = raster.transform * Affine.translation(c0, r0)
        shape = (r1 - r0, c1 - c0)
        own = rasterize([part.buffer(half_width, quad_segs=4)], out_shape=shape, transform=transform, fill=0, dtype=np.uint8).astype(bool)
        centre = rasterize([part.buffer(0.5 * raster.cell)], out_shape=shape, transform=transform, fill=0, all_touched=True, dtype=np.uint8)
        distance = distance_transform_edt(centre == 0) * raster.cell
        cells = own & valid_cells[window]
        values = measured[window]
        first = normalized_smoothing(values, cells, sigma)
        cells &= ~(np.abs(values - first) > OUTLIER_M)
        field = normalized_smoothing(values, cells, sigma)
        weight = np.clip((half_width + reach - distance) / reach, 0, 1)
        weight[~np.isfinite(field)] = 0
        return window, np.nan_to_num(field), weight, distance

    nearest_distance = np.full(measured.shape, np.inf, dtype=np.float32)
    nearest_value = np.full(measured.shape, np.nan, dtype=np.float32)
    for part, half_width in pieces:
        window, field, weight, distance = piece_window(part, half_width)
        closer = (weight > 0) & (distance < nearest_distance[window])
        nearest_distance[window][closer] = distance[closer]
        nearest_value[window][closer] = field[closer]
    total = np.zeros(measured.shape, dtype=np.float32)
    weights = np.zeros(measured.shape, dtype=np.float32)
    for part, half_width in pieces:
        window, field, weight, _ = piece_window(part, half_width)
        agree = np.exp(-np.square((field - np.nan_to_num(nearest_value[window])) / agreement))
        total[window] += weight * agree * field
        weights[window] += weight * agree
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(weights > 1e-4, total / weights, np.nan).astype(np.float32), float(weights.size)


def grade_limit(highway):
    return 0.15 if highway == "service" else 0.08 if highway in ("residential", "living_street", "unclassified") else MAXIMUM_GRADE


def bridge_profile(s, ground, start_height, end_height, minimum=None, grade=MAXIMUM_GRADE):
    """Deck heights along a bridge spanning surveyed dips and meeting per-sample minimum heights.

    Returns the profile and how much each end must be raised to reach the minimum within the grade limit.
    """
    length = float(s[-1])
    ground = np.where(np.isfinite(ground), ground, np.interp(s, [0, length], [start_height, end_height]))
    minimum = np.full_like(s, np.nan) if minimum is None else minimum
    constrained = np.isfinite(minimum)
    raise_start = raise_end = 0.0
    if constrained.any():
        raise_start = max(0.0, float(np.max(minimum[constrained] - grade * s[constrained])) - start_height)
        raise_end = max(0.0, float(np.max(minimum[constrained] - grade * (length - s[constrained]))) - end_height)
    start, end = start_height + raise_start, end_height + raise_end
    target = np.fmax(ground, minimum)
    points_s = np.r_[0.0, s, length]
    points_h = np.r_[start, np.minimum(target, np.minimum(start + grade * s, end + grade * (length - s))), end]
    profile = upper_hull(points_s, points_h)[1:-1]
    if len(profile) > 5:
        profile = gaussian_filter1d(profile, 6.0 / max(s[1] - s[0], 1e-6), mode="nearest")
    profile += (start - profile[0]) + ((end - profile[-1]) - (start - profile[0])) * s / max(length, 1e-6)
    return profile, raise_start, raise_end


def clockwise_from_above(vertices, triangles):
    """Godot front faces wind clockwise seen from +Y; collision ignores back faces."""
    a, b, c = vertices[triangles[:, 0]], vertices[triangles[:, 1]], vertices[triangles[:, 2]]
    cross_y = (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]) - (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
    flipped = triangles.copy()
    flipped[cross_y > 0] = flipped[cross_y > 0][:, [0, 2, 1]]
    return flipped


def grid_mesh(polygon, cell=MESH_CELL):
    """Triangulate a polygon on a shared world grid: whole cells inside, clipped cells along the boundary."""
    if polygon.is_empty:
        return np.zeros((0, 2)), np.zeros((0, 3), dtype=np.int64)
    minx, minz, maxx, maxz = polygon.bounds
    x0, z0 = math.floor(minx / cell) * cell, math.floor(minz / cell) * cell
    columns, rows = math.ceil((maxx - x0) / cell) + 1, math.ceil((maxz - z0) / cell) + 1
    touched = rasterize([polygon], out_shape=(rows, columns), transform=Affine(cell, 0, x0, 0, cell, z0),
                        all_touched=True, fill=0, default_value=1, dtype=np.uint8)
    row, column = np.nonzero(touched)
    boxes = shapely.box(x0 + column * cell, z0 + row * cell, x0 + (column + 1) * cell, z0 + (row + 1) * cell)
    shapely.prepare(polygon)
    inside = shapely.contains_properly(polygon, boxes)
    corners = [(row[inside], column[inside]), (row[inside], column[inside] + 1),
               (row[inside] + 1, column[inside] + 1), (row[inside] + 1, column[inside])]
    full = np.stack([np.column_stack([x0 + c * cell, z0 + r * cell]) for r, c in corners], axis=1)
    pieces = shapely.intersection(boxes[~inside], polygon)
    pieces = pieces[~shapely.is_empty(pieces)]
    triangles = shapely.get_parts(shapely.get_parts(shapely.constrained_delaunay_triangles(pieces)))
    triangles = triangles[shapely.get_type_id(triangles) == 3]
    triangles = triangles[shapely.area(triangles) > 1e-6]
    clipped = shapely.get_coordinates(shapely.get_exterior_ring(triangles)).reshape(-1, 4, 2)[:, :3]
    corners_xy = np.concatenate([full[:, [0, 1, 2]], full[:, [0, 2, 3]], clipped]).reshape(-1, 2)
    keys = np.round(corners_xy * 1000).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    vertices = unique.astype(np.float64) / 1000
    triangles = inverse.reshape(-1, 3)
    triangles = triangles[(triangles[:, 0] != triangles[:, 1]) & (triangles[:, 1] != triangles[:, 2]) & (triangles[:, 0] != triangles[:, 2])]
    return vertices, clockwise_from_above(vertices, triangles)


def vertex_normals(positions, triangles):
    a, b, c = positions[triangles[:, 0]], positions[triangles[:, 1]], positions[triangles[:, 2]]
    faces = np.cross(b - a, c - a)
    faces[faces[:, 1] < 0] *= -1
    normals = np.zeros_like(positions)
    for corner in range(3):
        np.add.at(normals, triangles[:, corner], faces)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.where(length > 1e-9, normals / np.maximum(length, 1e-9), [0, 1, 0])
    return normals


def remove_small_holes(geometry, minimum_area):
    polygons = []
    for polygon in shapely.get_parts(geometry):
        holes = [ring for ring in polygon.interiors if shapely.Polygon(ring).area >= minimum_area]
        polygons.append(shapely.Polygon(polygon.exterior, holes))
    return shapely.union_all(polygons) if polygons else geometry


class Raster:
    """1 m grid in game coordinates (x east, z south); cell centres at origin + (index + 0.5)."""

    def __init__(self, bounds, cell=RASTER_CELL, margin=20.0):
        minx, minz, maxx, maxz = bounds
        self.cell = cell
        self.x0 = math.floor((minx - margin) / cell) * cell
        self.z0 = math.floor((minz - margin) / cell) * cell
        self.columns = math.ceil((maxx + margin - self.x0) / cell)
        self.rows = math.ceil((maxz + margin - self.z0) / cell)
        self.transform = Affine(cell, 0, self.x0, 0, cell, self.z0)

    def mask(self, geometry):
        if geometry.is_empty:
            return np.zeros((self.rows, self.columns), dtype=bool)
        return rasterize([geometry], out_shape=(self.rows, self.columns), transform=self.transform, fill=0,
                         default_value=1, dtype=np.uint8).astype(bool)

    def centres(self, rows, columns):
        return self.x0 + (columns + 0.5) * self.cell, self.z0 + (rows + 0.5) * self.cell

    def sample(self, values, x, z, order=1):
        coordinates = np.vstack([(np.asarray(z) - self.z0) / self.cell - 0.5, (np.asarray(x) - self.x0) / self.cell - 0.5])
        return map_coordinates(values, coordinates, order=order, mode="nearest")


class NmtSampler:
    def __init__(self, root, stack, local_crs, bounds, source_crs):
        self.to_source = Transformer.from_crs(local_crs, source_crs, always_xy=True)
        minx, minz, maxx, maxz = bounds
        east, north = self.to_source.transform([minx, maxx, minx, maxx], [-minz, -minz, -maxz, -maxz])
        box = (math.floor(min(east)) - 5, math.floor(min(north)) - 5, math.ceil(max(east)) + 5, math.ceil(max(north)) + 5)
        rasters = []
        self.sources = []
        for filename in sorted((root / "data").glob("*.asc")):
            raster = stack.enter_context(rasterio.open(filename))
            if raster.crs and raster.crs != rasterio.crs.CRS.from_string(source_crs):
                raise ValueError(f"CRS mismatch: {filename.name}: {raster.crs}")
            if not (raster.bounds.right < box[0] or raster.bounds.left > box[2] or raster.bounds.top < box[1] or raster.bounds.bottom > box[3]):
                rasters.append(raster)
                self.sources.append(filename.name)
        if not rasters:
            raise ValueError("No NMT tile intersects the road network.")
        mosaic, self.affine = merge(rasters, bounds=box, res=1.0, nodata=-9999, resampling=Resampling.bilinear, dtype="float32")
        self.heights = np.where(mosaic[0] == -9999, np.nan, mosaic[0]).astype(np.float32)

    def sample(self, x, z):
        east, north = self.to_source.transform(np.asarray(x, dtype=np.float64), -np.asarray(z, dtype=np.float64))
        columns = (east - self.affine.c) / self.affine.a - 0.5
        rows = (north - self.affine.f) / self.affine.e - 0.5
        return map_coordinates(self.heights, np.vstack([rows, columns]), order=1, mode="constant", cval=np.nan)


def collect_deck_returns(root, decks, local_crs, source_crs):
    """Non-ground LiDAR returns over mapped bridge decks (local x, z, height), cached per deck geometry."""
    region = decks.buffer(1.0)
    key = hashlib.sha1(region.wkb + str(sorted(path.name for path in source_files(root / "data"))).encode()).hexdigest()
    cache = root / ".cache/bridge_deck_returns.npz"
    if cache.exists():
        stored = np.load(cache)
        if str(stored["key"]) == key:
            return stored["points"]
    to_source = Transformer.from_crs(local_crs, source_crs, always_xy=True)
    to_local = Transformer.from_crs(source_crs, local_crs, always_xy=True)
    coordinates = shapely.get_coordinates(region)
    source_region = shapely.set_coordinates(region, np.column_stack(to_source.transform(coordinates[:, 0], -coordinates[:, 1])))
    shapely.prepare(source_region)
    minx, miny, maxx, maxy = source_region.bounds
    records = []
    for filename in source_files(root / "data"):
        with laspy.open(filename) as reader:
            header = reader.header
            if header.maxs[0] < minx or header.mins[0] > maxx or header.maxs[1] < miny or header.mins[1] > maxy:
                continue
            for chunk in reader.chunk_iterator(2000000):
                x, y = np.asarray(chunk.x), np.asarray(chunk.y)
                keep = (x >= minx) & (x <= maxx) & (y >= miny) & (y <= maxy) & np.isin(np.asarray(chunk.classification), DECK_CLASSES)
                keep &= ~np.asarray(chunk.withheld, dtype=bool)
                if not keep.any():
                    continue
                inside = shapely.contains_xy(source_region, x[keep], y[keep])
                east, north = to_local.transform(x[keep][inside], y[keep][inside])
                records.append(np.column_stack([east, -np.asarray(north), np.asarray(chunk.z)[keep][inside]]))
        print(f"{filename.name}: bridge deck returns scanned", flush=True)
    points = np.concatenate(records) if records else np.zeros((0, 3))
    np.savez_compressed(cache, key=key, points=points)
    return points


def surveyed_deck(returns, road, s, spacing, density=4.0, spread=0.3):
    """Median deck height per station where the survey shows a dense, flat surface across the carriageway."""
    heights = np.full(len(s), np.nan)
    if not len(returns):
        return heights
    core = road["buffer"].buffer(-min(1.0, road["width"] / 4))
    inside = shapely.contains_xy(core, returns[:, 0], returns[:, 1])
    if not inside.any():
        return heights
    along = shapely.line_locate_point(road["line"], shapely.points(returns[inside, :2]))
    z = returns[inside, 2]
    station = np.clip(np.round(along / spacing).astype(int), 0, len(s) - 1)
    minimum_count = density * spacing * max(core.area / max(road["line"].length, 1e-6), 1.0)
    for index in np.unique(station):
        values = z[station == index]
        if len(values) < minimum_count:
            continue
        # Lamp posts and catenary wires are sparse; a deck is the dense lowest layer.
        low = values[values <= np.percentile(values, 50)]
        if np.subtract(*np.percentile(low, [75, 25])) <= spread:
            heights[index] = float(np.median(low))
    return heights


def project_factory(terrain):
    lon0, lat0 = terrain["origin_lon"], terrain["origin_lat"]
    scale = 111320.0 * math.cos(math.radians(lat0))
    return lambda lon, lat: np.column_stack([(np.asarray(lon) - lon0) * scale, -(np.asarray(lat) - lat0) * 111320.0])


def collect_roads(osm, project, corridor):
    """Clip drivable OSM ways to the corridor and merge same-attribute ways into continuous carriageways."""
    groups = defaultdict(list)
    shapely.prepare(corridor)
    included = []
    for element in osm["elements"]:
        tags = element.get("tags", {})
        profile = road_profile(tags)
        geometry = element.get("geometry", [])
        if profile is None or len(geometry) < 2:
            continue
        line = LineString(project([p["lon"] for p in geometry], [p["lat"] for p in geometry]))
        if not corridor.intersects(line):
            continue
        clipped = line.intersection(corridor)
        parts = [part for part in shapely.get_parts(clipped) if part.geom_type == "LineString" and part.length > 0.5]
        if not parts:
            continue
        included.append(int(element["id"]))
        key = (profile["highway"], tags.get("name", ""), profile["width"], profile["lanes"], profile["oneway"],
               profile["flags"], profile["bridge"], profile["layer"], tags.get("bridge", ""), profile["tunnel"])
        groups[key].extend(parts)
    roads = []
    for key, parts in groups.items():
        merged = linemerge(MultiLineString(parts)) if len(parts) > 1 else parts[0]
        for line in shapely.get_parts(merged):
            roads.append({"line": line, "highway": key[0], "name": key[1], "width": key[2], "lanes": key[3], "oneway": key[4],
                          "flags": key[5], "bridge": key[6], "layer": key[7], "bridge_kind": key[8], "tunnel": key[9]})
    return roads, sorted(included)


def extend_under_crossings(road, crossings, step=1.0, limit=30.0, margin=2.0):
    """Tunnel ways often stop inside the road above; extend them until the bore clears every crossing road."""
    coordinates = np.asarray(road["line"].coords, dtype=float)
    for at_start in (True, False):
        point, inner = (coordinates[0], coordinates[1]) if at_start else (coordinates[-1], coordinates[-2])
        outward = (point - inner) / max(np.linalg.norm(point - inner), 1e-9)
        probe = shapely.box(-road["width"] / 2, -0.1, road["width"] / 2, 0.1)
        distance = 0.0
        while distance < limit:
            centre = point + outward * distance
            footprint = shapely.affinity.rotate(shapely.affinity.translate(probe, *centre), math.degrees(math.atan2(outward[1], outward[0])) + 90, origin=tuple(centre))
            if not crossings.intersects(footprint):
                break
            distance += step
        if distance > 0:
            extended = point + outward * min(distance + margin, limit)
            coordinates = np.vstack([extended, coordinates]) if at_start else np.vstack([coordinates, extended])
    road["line"] = LineString(coordinates)


def route_gap_band(route_lines, roads, spacing=2.0):
    """Paved band along the bus route only where OSM maps no parallel carriageway close by.

    Where the GTFS shape runs a few metres beside a mapped road it is drawn offset, not a second road.
    """
    lines = np.array([road["line"] for road in roads], dtype=object)
    half = np.array([road["width"] / 2 for road in roads])
    tree = shapely.STRtree(lines)
    pieces = []
    for route in route_lines:
        s = np.linspace(0, route.length, max(2, math.ceil(route.length / spacing) + 1))
        points = shapely.line_interpolate_point(route, s)
        middle = shapely.line_interpolate_point(route, (s[:-1] + s[1:]) / 2)
        segment = np.diff(shapely.get_coordinates(points), axis=0)
        segment /= np.maximum(np.linalg.norm(segment, axis=1, keepdims=True), 1e-9)
        covered = np.zeros(len(middle), dtype=bool)
        point_index, road_index = tree.query(middle, predicate="dwithin", distance=float(half.max()) + 4.0)
        along = shapely.line_locate_point(lines[road_index], middle[point_index])
        direction = shapely.get_coordinates(shapely.line_interpolate_point(lines[road_index], along + 1.0)) - \
            shapely.get_coordinates(shapely.line_interpolate_point(lines[road_index], np.maximum(along - 1.0, 0)))
        direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-9)
        parallel = np.abs(np.sum(direction * segment[point_index], axis=1)) > 0.85
        close = shapely.distance(lines[road_index], middle[point_index]) <= half[road_index] + 4.0
        covered[point_index[parallel & close]] = True
        for index in np.flatnonzero(~covered):
            pieces.append(LineString([points[index], points[index + 1]]).buffer(ROUTE_HALF_WIDTH, quad_segs=6))
    return shapely.union_all(pieces) if pieces else shapely.Polygon()


def build_surfaces(roads, route_lines):
    for road in roads:
        road["is_deck"] = road["bridge"] and road["layer"] > 0
        road["is_tunnel"] = road["tunnel"] and not road["is_deck"]
        road["layered"] = road["is_deck"] or road["is_tunnel"]
    plain = [road for road in roads if not road["layered"]]
    for road in roads:
        if road["is_tunnel"]:
            ends = shapely.MultiPoint([road["line"].coords[0], road["line"].coords[-1]]).buffer(1.0)
            bore = road["line"].buffer(road["width"] / 2, cap_style="flat")
            above = [other["line"].buffer(other["width"] / 2) for other in plain
                     if other["line"].intersects(bore) and not other["line"].intersects(ends)]
            if above:
                extend_under_crossings(road, shapely.union_all(above))
    for road in roads:
        road["buffer"] = road["line"].buffer(road["width"] / 2, quad_segs=6,
                                             cap_style="flat" if road["layered"] else "round", join_style="round")
    bridges = [road for road in roads if road["is_deck"]]
    tunnels = [road for road in roads if road["is_tunnel"]]
    layered_roads = bridges + tunnels
    ground_roads = [road for road in roads if not road["layered"]]
    deck = shapely.union_all([road["buffer"] for road in bridges]) if bridges else shapely.Polygon()
    tunnel = shapely.union_all([road["buffer"] for road in tunnels]) if tunnels else shapely.Polygon()
    layered = shapely.union_all([deck, tunnel])
    if layered_roads:
        # Where the bus route runs alongside a structure but off its mapped outline, it is on the structure.
        structure_lines = [road["line"] for road in layered_roads]
        tree = shapely.STRtree(structure_lines)
        ground_union = shapely.union_all([road["buffer"] for road in ground_roads])
        widened = {"bridge": [], "tunnel": []}
        for route in route_lines:
            s = np.linspace(0, route.length, max(2, math.ceil(route.length / 2.0) + 1))
            points = shapely.line_interpolate_point(route, s)
            middle = shapely.line_interpolate_point(route, (s[:-1] + s[1:]) / 2)
            (_, nearest), distance = tree.query_nearest(middle, all_matches=False, return_distance=True)
            along = shapely.line_locate_point(np.array(structure_lines, dtype=object)[nearest], middle)
            lines = np.array(structure_lines, dtype=object)[nearest]
            direction = shapely.get_coordinates(shapely.line_interpolate_point(lines, along + 1.0)) - \
                shapely.get_coordinates(shapely.line_interpolate_point(lines, np.maximum(along - 1.0, 0)))
            segment = np.diff(shapely.get_coordinates(points), axis=0)
            cosine = np.abs(np.sum(direction * segment, axis=1)) / np.maximum(
                np.linalg.norm(direction, axis=1) * np.linalg.norm(segment, axis=1), 1e-9)
            for index in np.flatnonzero((distance <= 8.0) & (cosine > 0.85)):
                piece = LineString([points[index], points[index + 1]]).buffer(ROUTE_HALF_WIDTH, cap_style="flat")
                kind = "bridge" if layered_roads[nearest[index]]["is_deck"] else "tunnel"
                widened[kind].append(piece)
        for kind, pieces in widened.items():
            if not pieces:
                continue
            extra = shapely.union_all(pieces).buffer(1.0, quad_segs=4).buffer(-1.0, quad_segs=4)
            extra = extra.difference(ground_union).difference(layered)
            if kind == "bridge":
                deck = shapely.union_all([deck, extra])
            else:
                tunnel = shapely.union_all([tunnel, extra])
        layered = shapely.union_all([deck, tunnel])
    ends = shapely.MultiPoint([point for road in layered_roads for point in (road["line"].coords[0], road["line"].coords[-1])])
    end_zone = ends.buffer(1.0) if layered_roads else shapely.Polygon()
    underpasses, connected = [], []
    for road in ground_roads:
        # Ground roads that cross a deck or tunnel without joining it keep their full surface on their own level.
        passes = bool(layered_roads) and road["buffer"].intersects(layered) and not road["line"].intersects(end_zone)
        (underpasses if passes else connected).append(road)
    route_band = route_gap_band(route_lines, roads)
    base = shapely.union_all([road["buffer"] for road in connected] + [route_band.difference(layered.buffer(0.5))])
    base = base.buffer(CLOSING_M, quad_segs=6).buffer(-CLOSING_M, quad_segs=6)
    ground = shapely.union_all([base.difference(layered)] + [road["buffer"] for road in underpasses])
    ground = remove_small_holes(shapely.make_valid(ground).buffer(0), 25.0).simplify(0.05)
    deck = deck.simplify(0.05)
    tunnel = tunnel.simplify(0.05)
    verge = ground.buffer(VERGE_MAX_M, quad_segs=4).difference(ground).difference(layered.buffer(0.3))
    carriageway_only = shapely.union_all([road["buffer"] for road in roads])
    return {"ground": ground, "deck": deck, "tunnel": tunnel, "verge": verge, "bridges": bridges, "tunnels": tunnels,
            "underpasses": underpasses, "carriageways": carriageway_only}


def road_attributes(points, roads, owner_weight, raster):
    """Per-vertex marking data: signed lateral offset and distance along the owning carriageway."""
    tree = shapely.STRtree([road["buffer"] for road in roads])
    geometries = shapely.points(points)
    point_index, road_index = tree.query(geometries, predicate="within")
    lines = np.array([road["line"] for road in roads], dtype=object)
    owner = np.full(len(points), -1)
    owner[point_index] = road_index
    missing = owner < 0
    if missing.any():
        owner[missing] = tree.query_nearest(geometries[missing], all_matches=False)[1]
    along = shapely.line_locate_point(lines[owner], geometries)
    ahead = shapely.get_coordinates(shapely.line_interpolate_point(lines[owner], along + 0.25))
    behind = shapely.get_coordinates(shapely.line_interpolate_point(lines[owner], np.maximum(along - 0.25, 0)))
    tangent = ahead - behind
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    centre = shapely.get_coordinates(shapely.line_interpolate_point(lines[owner], along))
    offset = points - centre
    # Positive lateral is to the right of the way direction (x east, z south).
    lateral = offset[:, 0] * -tangent[:, 1] + offset[:, 1] * tangent[:, 0]
    attributes = np.zeros((len(points), 6), dtype=np.float32)
    attributes[:, 0] = lateral
    attributes[:, 1] = along
    attributes[:, 2] = [roads[index]["width"] / 2 for index in owner]
    attributes[:, 3] = [roads[index]["lanes"] for index in owner]
    attributes[:, 4] = [roads[index]["flags"] for index in owner]
    attributes[:, 5] = np.clip(raster.sample(owner_weight, points[:, 0], points[:, 1]), 0, 1)
    return attributes


def generate(root, terrain, route, osm, source_crs, corridor_m, preview):
    project = project_factory(terrain)
    local_crs = local_projection(terrain)
    route_lines = {}
    for direction in route["directions"]:
        coordinates = np.asarray(direction["points"])
        route_lines[direction["shape_id"]] = LineString(project(coordinates[:, 0], coordinates[:, 1]))
    terrain_box = shapely.box(terrain["grid_x"] + 10, terrain["grid_z"] + 10,
                              terrain["grid_x"] + (terrain["columns"] - 1) * terrain["spacing_m"] - 10,
                              terrain["grid_z"] + (terrain["rows"] - 1) * terrain["spacing_m"] - 10)
    corridor = shapely.union_all([line.buffer(corridor_m) for line in route_lines.values()]).intersection(terrain_box)
    roads, included = collect_roads(osm, project, corridor)
    surfaces = build_surfaces(roads, list(route_lines.values()))
    ground, deck, verge = surfaces["ground"], surfaces["deck"], surfaces["verge"]
    route_union = shapely.union_all(list(route_lines.values()))
    coverage = route_union.intersection(surfaces["carriageways"]).length / route_union.length
    print(f"Roads: {len(roads)} carriageways from {len(included)} OSM ways; ground {ground.area:,.0f} m2, "
          f"decks {deck.area:,.0f} m2 ({len(surfaces['bridges'])}), route on OSM carriageway {coverage:.1%}", flush=True)

    raster = Raster(shapely.union_all([ground, deck, verge]).bounds)
    ground_mask = raster.mask(ground)
    needed = binary_dilation(ground_mask, iterations=int(VERGE_MAX_M + 4 * SMOOTHING_SIGMA_M))
    rows, columns = np.nonzero(needed)
    with ExitStack() as stack:
        nmt = NmtSampler(root, stack, local_crs, raster_bounds(raster), source_crs)
        cx, cz = raster.centres(rows, columns)
        measured = np.full((raster.rows, raster.columns), np.nan, dtype=np.float32)
        measured[rows, columns] = nmt.sample(cx, cz)
        valid = ground_mask & np.isfinite(measured)
        first = normalized_smoothing(measured, valid, SMOOTHING_SIGMA_M)
        outliers = valid & (np.abs(measured - first) > OUTLIER_M)
        valid &= ~outliers
        # Global smoothing only fills route-only areas with no mapped carriageway nearby.
        fallback = normalized_smoothing(measured, valid, SMOOTHING_SIGMA_M)
        # Bare-earth heights inside tunnel footprints are the underpass floor, not the roads crossing above it.
        bore = raster.mask(surfaces["tunnel"].buffer(1.5)) if not surfaces["tunnel"].is_empty else np.zeros_like(ground_mask)
        if surfaces["underpasses"]:
            bore &= raster.mask(shapely.union_all([road["buffer"] for road in surfaces["underpasses"]]))
        else:
            bore[:] = False
        per_road, _ = road_field(measured, ground_mask & np.isfinite(measured) & ~bore, [road for road in roads if not road["layered"]], raster)
        field = np.where(np.isfinite(per_road), per_road, fallback).astype(np.float32)
        if bore.any():
            known = np.isfinite(field) & ~bore
            _, (near_rows, near_columns) = distance_transform_edt(~known, return_indices=True)
            filled = normalized_smoothing(field[near_rows, near_columns], np.ones_like(known), 4.0)
            field = np.where(bore & ground_mask, filled, field).astype(np.float32)
        residual = np.abs(measured - field)[valid & np.isfinite(field)]

        chains = []

        def approach_height(line, at_start, fallback):
            """Bare-earth height at a structure end, fitted along the approach so crossing levels do not leak in."""
            coordinates = np.asarray(line.coords)
            point, inner = (coordinates[0], coordinates[1]) if at_start else (coordinates[-1], coordinates[-2])
            outward = (point - inner) / max(np.linalg.norm(point - inner), 1e-9)
            distance = np.arange(3.0, 15.1, 1.0)
            heights = nmt.sample(point[0] + outward[0] * distance, point[1] + outward[1] * distance)
            usable = np.isfinite(heights)
            if usable.sum() < 5:
                return fallback
            slope, intercept = np.polyfit(distance[usable], heights[usable], 1)
            return float(intercept) if np.isfinite(intercept) else fallback
        deck_returns = collect_deck_returns(root, deck, local_crs, source_crs) if surfaces["bridges"] else np.zeros((0, 3))
        for road in surfaces["bridges"]:
            line = road["line"]
            spacing = line.length / max(2, math.ceil(line.length))
            s = np.arange(0, line.length + spacing * 0.5, spacing)
            centre = shapely.get_coordinates(shapely.line_interpolate_point(line, s))
            under = nmt.sample(centre[:, 0], centre[:, 1])
            start, end = (raster.sample(field, [c[0]], [c[1]])[0] for c in (centre[0], centre[-1]))
            start = start if np.isfinite(start) else under[0]
            end = end if np.isfinite(end) else under[-1]
            envelope = upper_hull(np.r_[0.0, s, s[-1]], np.r_[start, np.where(np.isfinite(under), under, start), end])[1:-1]
            spanned = envelope - under > BRIDGE_VOID_M
            surveyed = surveyed_deck(deck_returns, road, s, spacing)
            minimum = np.full_like(s, np.nan)
            evidence = spanned & np.isfinite(surveyed)
            if spanned.any() and evidence.sum() >= 0.5 * spanned.sum():
                # The 2018 survey saw this deck: follow it across the dip.
                minimum[spanned] = np.interp(s[spanned], s[evidence], surveyed[evidence])
                source = "lidar_deck"
            elif spanned.any() and road["bridge_kind"] == "viaduct":
                # Built after the survey and the map keeps no railways: assume the lowest part of the dip is tracks.
                track_level = np.nanpercentile(under[spanned], 10)
                tracks = spanned & (under < track_level + 1.0)
                minimum[tracks] = under[tracks] + BRIDGE_RAIL_CLEARANCE_M
                source = "engineered_clearance"
            else:
                source = "surveyed_envelope"
            below = [other for other in surfaces["underpasses"] if other["buffer"].intersects(road["buffer"])]
            if below:
                crossing = shapely.contains_xy(shapely.union_all([other["buffer"] for other in below]).buffer(2.0), centre[:, 0], centre[:, 1])
                road_floor = raster.sample(np.nan_to_num(field, nan=-1e3), centre[crossing, 0], centre[crossing, 1]) + BRIDGE_ROAD_CLEARANCE_M
                minimum[crossing] = np.fmax(minimum[crossing], road_floor)
            profile, raise_start, raise_end = bridge_profile(s, under, start, end, minimum, grade_limit(road["highway"]))
            chains.append({"road": road, "s": s, "profile": profile, "raise": (float(raise_start), float(raise_end)), "source": source,
                           "kind": "bridge", "cover": None,
                           "max_grade": float(np.max(np.abs(np.diff(profile)) / spacing)) if len(s) > 1 else 0.0,
                           "minimum_clearance": float(np.nanmin(profile - under))})
        for road in surfaces["tunnels"]:
            line = road["line"]
            spacing = line.length / max(2, math.ceil(line.length))
            s = np.arange(0, line.length + spacing * 0.5, spacing)
            centre = shapely.get_coordinates(shapely.line_interpolate_point(line, s))
            above = nmt.sample(centre[:, 0], centre[:, 1])
            start, end = (raster.sample(field, [c[0]], [c[1]])[0] for c in (centre[0], centre[-1]))
            start = approach_height(line, True, start if np.isfinite(start) else above[0] - TUNNEL_COVER_M)
            end = approach_height(line, False, end if np.isfinite(end) else above[-1] - TUNNEL_COVER_M)
            linear = start + (end - start) * s / max(line.length, 1e-6)
            # Only the covered part constrains depth; portals already sit in their cuttings.
            covered = above - linear > 2.0
            ceiling_limit = np.where(covered, -(above - TUNNEL_COVER_M), np.nan)
            # Negated, the deck fitter gives the shallowest floor that stays a full bore below the surface above.
            lowest, lower_start, lower_end = bridge_profile(s, -linear, -start, -end, ceiling_limit, grade_limit(road["highway"]))
            profile = -lowest
            chains.append({"road": road, "s": s, "profile": profile, "raise": (-float(lower_start), -float(lower_end)),
                           "source": "tunnel_below_surface", "kind": "tunnel", "cover": np.fmax(above, profile + TUNNEL_COVER_M),
                           "max_grade": float(np.max(np.abs(np.diff(profile)) / spacing)) if len(s) > 1 else 0.0,
                           "minimum_clearance": float(np.nanmin(above - profile))})
    # Twin carriageways of one structure share a deck (or bore) level, so no step appears between them.
    combined = []
    for chain in chains:
        line = chain["road"]["line"]
        centre = shapely.get_coordinates(shapely.line_interpolate_point(line, chain["s"]))
        profile = chain["profile"].copy()
        for other in chains:
            if (other is chain or other["kind"] != chain["kind"] or not chain["road"]["name"]
                    or other["road"]["name"] != chain["road"]["name"] or other["road"]["highway"] != chain["road"]["highway"]):
                continue
            reach = (chain["road"]["width"] + other["road"]["width"]) / 2 + 4.0
            if line.distance(other["road"]["line"]) > reach:
                continue
            near = shapely.distance(other["road"]["line"], shapely.points(centre)) < reach
            if near.any():
                other_s = shapely.line_locate_point(other["road"]["line"], shapely.points(centre[near]))
                other_h = np.interp(other_s, other["s"], other["profile"])
                profile[near] = np.maximum(profile[near], other_h) if chain["kind"] == "bridge" else np.minimum(profile[near], other_h)
        combined.append(profile)
    for chain, profile in zip(chains, combined):
        shift = profile - chain["profile"]
        chain["raise"] = (chain["raise"][0] + float(shift[0]), chain["raise"][1] + float(shift[-1]))
        chain["profile"] = profile
        chain["max_grade"] = float(np.max(np.abs(np.diff(profile)) / np.diff(chain["s"]))) if len(profile) > 1 else 0.0
    # Approach ramps: when a deck must start higher (or a tunnel lower) than the surveyed road, reshape the joining roads.
    ramp_up = np.zeros_like(field)
    ramp_down = np.zeros_like(field)
    for chain in chains:
        line = chain["road"]["line"]
        for end_index, raise_by in enumerate(chain["raise"]):
            if abs(raise_by) <= 0.05:
                continue
            point = line.coords[0] if end_index == 0 else line.coords[-1]
            grade = grade_limit(chain["road"]["highway"])
            reach = int(abs(raise_by) / grade / raster.cell) + 2
            centre_row, centre_column = int((point[1] - raster.z0) / raster.cell), int((point[0] - raster.x0) / raster.cell)
            rows = slice(max(centre_row - reach, 0), min(centre_row + reach, raster.rows))
            columns = slice(max(centre_column - reach, 0), min(centre_column + reach, raster.columns))
            # Everything at the approach level near the structure end moves with it; other levels stay put.
            level = raster.sample(field, [point[0]], [point[1]])[0]
            window_field = field[rows, columns]
            approach = ground_mask[rows, columns] & np.isfinite(window_field) & (np.abs(window_field - level) < 1.5)
            if not np.isfinite(level) or not approach.any():
                continue
            gz, gx = np.mgrid[rows, columns]
            gx, gz = raster.centres(gz, gx)
            # Ramps run along the structure axis, so contours stay square to the carriageway.
            ahead = np.array(line.coords[1] if end_index == 0 else line.coords[-2]) - np.array(point)
            outward = -ahead / max(np.linalg.norm(ahead), 1e-9)
            distance = np.maximum((gx - point[0]) * outward[0] + (gz - point[1]) * outward[1], 0)
            lateral = np.abs((gx - point[0]) * -outward[1] + (gz - point[1]) * outward[0])
            approach &= lateral <= chain["road"]["width"] / 2 + 6.0
            cone = raise_by * np.clip(1 - distance * grade / abs(raise_by), 0, 1) * approach
            if raise_by > 0:
                ramp_up[rows, columns] = np.maximum(ramp_up[rows, columns], cone)
            else:
                ramp_down[rows, columns] = np.minimum(ramp_down[rows, columns], cone)
    if ramp_up.any() or ramp_down.any():
        field = field + np.nan_to_num(gaussian_filter(ramp_up + ramp_down, 1.5))
    field += SURFACE_LIFT
    if not np.isfinite(raster.sample(field, *shapely.get_coordinates(ground.representative_point()).T)).all():
        raise ValueError("Elevation field does not cover the road network.")

    owner_count = rasterize([(road["buffer"], 1) for road in roads], out_shape=(raster.rows, raster.columns), transform=raster.transform,
                            fill=0, merge_alg=MergeAlg.add, dtype=np.uint8)
    junction_distance = distance_transform_edt(owner_count < 2) * raster.cell
    owner_weight = (np.clip((junction_distance - 1.0) / 3.0, 0, 1) * (owner_count == 1)).astype(np.float32)
    owner_weight = gaussian_filter(owner_weight, 0.7)

    chain_lines = [chain["road"]["line"] for chain in chains]
    chain_tree = shapely.STRtree(chain_lines) if chains else None

    def deck_heights(points, chosen=None):
        geometries = shapely.points(points)
        nearest = chain_tree.query_nearest(geometries, all_matches=False)[1] if chosen is None else np.asarray(chosen)
        heights = np.empty(len(points))
        for index in np.unique(nearest):
            chain = chains[index]
            selected = nearest == index
            line = chain_lines[index]
            s = shapely.line_locate_point(line, geometries[selected])
            centre = shapely.get_coordinates(shapely.line_interpolate_point(line, s))
            base = np.interp(s, chain["s"], chain["profile"]) + SURFACE_LIFT
            offset = points[selected] - centre
            for end_s, weight in ((0.0, np.clip(1 - s / 10, 0, 1)), (line.length, np.clip(1 - (line.length - s) / 10, 0, 1))):
                # Blend into the joining road's crossfall so the seam has no step.
                end_point = np.array(shapely.get_coordinates(line.interpolate(end_s))[0])
                seam = raster.sample(field, end_point[0] + offset[:, 0], end_point[1] + offset[:, 1])
                end_height = np.interp(end_s, chain["s"], chain["profile"]) + SURFACE_LIFT
                base += weight * np.where(np.isfinite(seam), seam - end_height, 0)
            heights[selected] = base
        return heights

    ground_vertices, ground_triangles = grid_mesh(ground)
    layered = shapely.union_all([deck, surfaces["tunnel"]])
    if chains and not layered.is_empty:
        # Ground at a joining level next to a structure end (staggered twin approaches, widened route) meets it smoothly.
        end_zones = shapely.union_all([shapely.Point(chain["road"]["line"].coords[index]).buffer(LAYER_BLEND_M + chain["road"]["width"])
                                       for chain in chains for index in (0, -1)])
        layered_mask = raster.mask(layered.intersection(end_zones))
        crossing = raster.mask(shapely.union_all([road["buffer"] for road in surfaces["underpasses"]])) if surfaces["underpasses"] else np.zeros_like(ground_mask)
        rows, columns = np.nonzero(layered_mask)
        layered_field = np.full_like(field, np.nan)
        layered_field[rows, columns] = deck_heights(np.column_stack(raster.centres(rows, columns)))
        distance, (near_rows, near_columns) = distance_transform_edt(~layered_mask, return_indices=True)
        distance *= raster.cell
        target = layered_field[near_rows, near_columns]
        difference = target - field
        blend = (1 - np.clip(distance / LAYER_BLEND_M, 0, 1)) ** 2 * (np.abs(difference) < 3.0) * ground_mask * ~crossing
        field = np.where(blend > 0, field + blend * np.nan_to_num(difference), field)
    deck_vertices, deck_triangles = grid_mesh(deck)
    tunnel_vertices, tunnel_triangles = grid_mesh(surfaces["tunnel"])
    verge_vertices, verge_triangles = grid_mesh(verge)
    ground_y = raster.sample(field, ground_vertices[:, 0], ground_vertices[:, 1])
    deck_y = deck_heights(deck_vertices) if len(deck_vertices) else np.zeros(0)
    tunnel_y = deck_heights(tunnel_vertices) if len(tunnel_vertices) else np.zeros(0)
    if not (np.isfinite(ground_y).all() and np.isfinite(deck_y).all() and np.isfinite(tunnel_y).all()):
        raise ValueError("Road vertices outside the elevation field.")
    road_distance = distance_transform_edt(~ground_mask) * raster.cell
    verge_edge = raster.sample(np.nan_to_num(field, nan=np.nanmean(field)), verge_vertices[:, 0], verge_vertices[:, 1])
    verge_distance = np.maximum(raster.sample(road_distance, verge_vertices[:, 0], verge_vertices[:, 1]) - 0.5, 0)
    # Verge vertices shared with the road mesh take the road height exactly, so the seam has no crack.
    road_keys = {tuple(key) for key in np.round(ground_vertices * 1000).astype(np.int64)}
    on_boundary = np.array([tuple(key) in road_keys for key in np.round(verge_vertices * 1000).astype(np.int64)], dtype=bool)
    if on_boundary.any():
        verge_distance[on_boundary] = 0
        verge_edge[on_boundary] = raster.sample(field, verge_vertices[on_boundary, 0], verge_vertices[on_boundary, 1])

    surfaces_out = {}
    for name, vertices, triangles, heights in (("road", ground_vertices, ground_triangles, ground_y),
                                                ("deck", deck_vertices, deck_triangles, deck_y),
                                                ("tunnel", tunnel_vertices, tunnel_triangles, tunnel_y)):
        positions = np.column_stack([vertices[:, 0], heights, vertices[:, 1]])
        data = np.zeros((len(vertices), ROAD_STRIDE), dtype=np.float32)
        data[:, 0:3] = positions
        data[:, 3:6] = vertex_normals(positions, triangles) if len(triangles) else [0, 1, 0]
        if len(vertices):
            data[:, 6:12] = road_attributes(vertices, roads, owner_weight, raster)
        surfaces_out[name] = (data, triangles)
    verge_data = np.column_stack([verge_vertices[:, 0], verge_vertices[:, 1], verge_edge, verge_distance]).astype(np.float32)
    surfaces_out["verge"] = (verge_data, verge_triangles)

    # Terrain is carved down to tunnel floors so portals open; this cover restores the surface above the bore.
    cover_polygon = shapely.union_all([chain["road"]["line"].buffer(chain["road"]["width"] / 2 + 5.0, cap_style="flat")
                                       for chain in chains if chain["kind"] == "tunnel"]) if chains else shapely.Polygon()
    cover_vertices, cover_triangles = grid_mesh(cover_polygon)
    if len(cover_vertices):
        ceiling = deck_heights(cover_vertices) + TUNNEL_HEIGHT_M + 0.3
        cover_y = np.fmax(nmt.sample(cover_vertices[:, 0], cover_vertices[:, 1]) - 0.15, ceiling)
        surfaces_out["cover"] = (np.column_stack([cover_vertices[:, 0], cover_y, cover_vertices[:, 1]]).astype(np.float32), cover_triangles)

    def edge_lines(polygon, open_to, spacing=2.0):
        """Structure outline parts not shared with another drivable surface, with heights on the structure."""
        result = []
        if polygon.is_empty:
            return result
        sides = polygon.boundary.difference(open_to.buffer(0.6))
        for part in shapely.get_parts(shapely.line_merge(sides) if sides.geom_type == "MultiLineString" else sides):
            if part.geom_type != "LineString" or part.length < 2:
                continue
            xz = shapely.get_coordinates(shapely.segmentize(part, spacing))
            result.append(np.column_stack([xz[:, 0], deck_heights(xz), xz[:, 1]]).round(3).tolist())
        return result

    tunnel = surfaces["tunnel"]
    parapets = edge_lines(deck, shapely.union_all([ground, tunnel]))
    tunnel_walls = edge_lines(tunnel, shapely.union_all([ground, deck]))
    deck_outlines = []
    for polygon in shapely.get_parts(deck):
        if polygon.is_empty:
            continue
        for ring in [polygon.exterior, *polygon.interiors]:
            xz = shapely.get_coordinates(shapely.segmentize(ring, 2.0))
            deck_outlines.append(np.column_stack([xz[:, 0], deck_heights(xz), xz[:, 1]]).round(3).tolist())
    portals = []
    for chain in chains:
        if chain["kind"] != "tunnel":
            continue
        line = chain["road"]["line"]
        for at_start in (True, False):
            coordinates = np.asarray(line.coords)
            point, inner = (coordinates[0], coordinates[1]) if at_start else (coordinates[-1], coordinates[-2])
            across = np.array([-(point - inner)[1], (point - inner)[0]]) / max(np.linalg.norm(point - inner), 1e-9)
            half = chain["road"]["width"] / 2 + 0.3
            floor = float(deck_heights(point[None, :])[0])
            top = max(float(nmt.sample([point[0]], [point[1]])[0]) if np.isfinite(nmt.sample([point[0]], [point[1]])[0]) else 0.0,
                      floor + TUNNEL_HEIGHT_M + 1.0)
            a, b = point + across * half, point - across * half
            portals.append([round(float(a[0]), 3), round(float(a[1]), 3), round(float(b[0]), 3), round(float(b[1]), 3),
                            round(floor, 3), round(top, 3)])

    building_clips, clip_zone = {}, deck.buffer(1.0)
    for element in osm["elements"]:
        tags = element.get("tags", {})
        geometry = element.get("geometry", [])
        if "building" not in tags or len(geometry) < 4 or deck.is_empty:
            continue
        footprint = shapely.make_valid(shapely.Polygon(project([p["lon"] for p in geometry], [p["lat"] for p in geometry])))
        if not footprint.intersects(clip_zone):
            continue
        remaining = footprint.difference(clip_zone)
        building_clips[str(element["id"])] = [np.asarray(part.exterior.coords)[:-1].round(3).tolist()
                                              for part in shapely.get_parts(remaining) if part.geom_type == "Polygon" and part.area > 10]

    route_samples, quality = {}, {"route_on_osm_carriageway": coverage, "nmt_outlier_fraction": float(outliers.sum() / max(1, (ground_mask).sum())),
                                  "nmt_smoothing_residual_p95_m": float(np.percentile(residual, 95)), "bridges": []}
    surface_union = shapely.union_all([ground, layered])
    shapely.prepare(surface_union)
    shapely.prepare(layered)
    for shape_id, line in route_lines.items():
        s = np.arange(0, line.length, 2.0)
        points = shapely.get_coordinates(shapely.line_interpolate_point(line, s))
        tangent = shapely.get_coordinates(shapely.line_interpolate_point(line, np.minimum(s + 1, line.length))) - \
            shapely.get_coordinates(shapely.line_interpolate_point(line, np.maximum(s - 1, 0)))
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        sample_points = points
        # Where the GTFS shape strays off the mapped carriageway, read heights from the nearest paved point.
        outside = ~shapely.contains_xy(surface_union, points[:, 0], points[:, 1])
        points = points.copy()
        if outside.any():
            nearest = shapely.get_coordinates(shapely.shortest_line(shapely.points(points[outside]), surface_union)).reshape(-1, 2, 2)[:, 1]
            inward = nearest - points[outside]
            inward /= np.maximum(np.linalg.norm(inward, axis=1, keepdims=True), 1e-9)
            points[outside] = nearest + inward * 0.1
        heights = raster.sample(field, points[:, 0], points[:, 1])
        if chains:
            on_deck = shapely.contains_xy(layered, points[:, 0], points[:, 1])
            geometries = shapely.points(points)
            # Candidate structures the route runs along: close to the centre line and parallel to it.
            reach = max(chain["road"]["width"] / 2 for chain in chains) + 3.0
            point_index, chain_index = chain_tree.query(geometries, predicate="dwithin", distance=reach)
            lines = np.array(chain_lines, dtype=object)[chain_index]
            along = shapely.line_locate_point(lines, geometries[point_index])
            direction = shapely.get_coordinates(shapely.line_interpolate_point(lines, along + 1.0)) - \
                shapely.get_coordinates(shapely.line_interpolate_point(lines, np.maximum(along - 1.0, 0)))
            direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-9)
            half = np.array([chains[index]["road"]["width"] / 2 for index in chain_index])
            keep = (shapely.distance(lines, geometries[point_index]) <= half + 3.0) & \
                (np.abs(np.sum(direction * tangent[point_index], axis=1)) > 0.85) & on_deck[point_index]
            point_index, chain_index = point_index[keep], chain_index[keep]
            candidate_y = deck_heights(points[point_index], chain_index) if len(point_index) else np.zeros(0)
            candidates = defaultdict(list)
            for index, value in zip(point_index, candidate_y):
                candidates[int(index)].append(float(value))
            other_y = np.full(len(points), np.nan)
            if on_deck.any():
                other_y[on_deck] = deck_heights(points[on_deck])
            previous = heights[0] if np.isfinite(heights[0]) else other_y[0]
            for index in range(len(points)):
                options = candidates.get(index) or [value for value in (heights[index], other_y[index]) if np.isfinite(value)]
                if options:
                    heights[index] = min(options, key=lambda value: abs(value - previous))
                previous = heights[index]
        right = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        edges = np.full((len(points), 2), 20.0)
        for side, column in ((-1, 0), (1, 1)):
            for offset in np.arange(0.25, 20.01, 0.25):
                probe = sample_points + right * side * offset
                outside = ~shapely.contains_xy(surface_union, probe[:, 0], probe[:, 1]) & (edges[:, column] >= 20.0)
                edges[outside, column] = offset
        grade = np.abs(np.diff(heights)) / 2.0
        heights = median_filter(heights, size=5, mode="nearest")
        route_samples[shape_id] = np.column_stack([sample_points[:, 0], heights, sample_points[:, 1], edges]).round(3).tolist()
        quality[f"route_{shape_id}_max_grade"] = float(grade.max())
        quality[f"route_{shape_id}_grade_spikes"] = [[round(float(s[i]), 1), round(float(points[i, 0]), 1), round(float(points[i, 1]), 1),
                                                        round(float(heights[i]), 2), round(float(heights[i + 1]), 2)]
                                                       for i in np.flatnonzero(grade > 0.15)[:12]]
        quality[f"route_{shape_id}_p99_grade_change_per_m"] = float(np.percentile(np.abs(np.diff(grade)) / 2.0, 99))
    for chain in chains:
        quality["bridges"].append({"name": chain["road"]["name"], "kind": chain["kind"] + ":" + chain["road"]["bridge_kind"], "highway": chain["road"]["highway"],
                                   "length_m": round(chain["road"]["line"].length, 1), "width_m": chain["road"]["width"], "source": chain["source"],
                                   "raise_m": [round(value, 2) for value in chain["raise"]],
                                   "max_grade": round(chain["max_grade"], 4), "minimum_clearance_m": round(chain["minimum_clearance"], 2)})
    if preview:
        write_preview(root, raster, ground, layered, verge, owner_weight, field)
    return {"surfaces": surfaces_out, "route_samples": route_samples, "parapets": parapets, "deck_outlines": deck_outlines,
            "tunnel_walls": tunnel_walls, "portals": portals,
            "building_clips": building_clips, "included_way_ids": included, "quality": quality,
            "nmt_sources": nmt.sources, "counts": {"carriageways": len(roads), "bridges": len(surfaces["bridges"]), "tunnels": len(surfaces["tunnels"])}}


def raster_bounds(raster):
    return (raster.x0, raster.z0, raster.x0 + raster.columns * raster.cell, raster.z0 + raster.rows * raster.cell)


def write_preview(root, raster, ground, deck, verge, weight, field):
    step = 2
    image = np.zeros((3, raster.rows // step, raster.columns // step), dtype=np.uint8)
    for geometry, colour in ((verge, (150, 140, 110)), (ground, (70, 72, 76)), (deck, (60, 110, 200))):
        mask = raster.mask(geometry)[::step, ::step][:image.shape[1], :image.shape[2]]
        for band in range(3):
            image[band][mask] = colour[band]
    marks = (weight[::step, ::step] > 0.5)[:image.shape[1], :image.shape[2]] & (image[0] == 70)
    image[0][marks], image[1][marks], image[2][marks] = 90, 92, 96
    write_png(root / ".cache/road_network_preview.png", image)


def tile_and_write(root, staging, result):
    blob = bytearray()
    tiles = defaultdict(dict)
    for name, (data, triangles) in result["surfaces"].items():
        if not len(triangles):
            continue
        x = data[:, 0]
        z = data[:, 1] if name == "verge" else data[:, 2]
        centroid_x = x[triangles].mean(axis=1)
        centroid_z = z[triangles].mean(axis=1)
        keys = np.column_stack([np.floor(centroid_x / TILE_M), np.floor(centroid_z / TILE_M)]).astype(int)
        for key in np.unique(keys, axis=0):
            selected = triangles[(keys == key).all(axis=1)]
            used, local = np.unique(selected, return_inverse=True)
            vertex_offset = len(blob)
            blob += data[used].astype("<f4").tobytes()
            index_offset = len(blob)
            blob += local.reshape(-1, 3).astype("<i4").tobytes()
            tiles[(int(key[0]), int(key[1]))][name] = {"vertex_offset": vertex_offset, "vertex_count": int(len(used)),
                                                       "index_offset": index_offset, "index_count": int(local.size)}
    (staging / "network.bin").write_bytes(bytes(blob))
    return [{"tile": list(key), **{"surfaces": value}} for key, value in sorted(tiles.items())]


def main():
    parser = argparse.ArgumentParser(description="Generate the OSM/NMT road network around route 108.")
    parser.add_argument("--source-crs", default="EPSG:2180")
    parser.add_argument("--corridor", type=float, default=CORRIDOR_M)
    parser.add_argument("--preview", action="store_true", help="Write .cache/road_network_preview.png")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    terrain = json.loads((root / "data/terrain_108.json").read_text(encoding="utf-8-sig"))
    route = json.loads((root / "data/route_108.json").read_text(encoding="utf-8-sig"))
    osm = json.loads((root / "data/map_108.json").read_text(encoding="utf-8-sig"))
    result = generate(root, terrain, route, osm, args.source_crs, args.corridor, args.preview)
    print(json.dumps(result["quality"], indent=2), flush=True)
    for chain in result["quality"]["bridges"]:
        if max(chain["raise_m"]) > 0.05 and chain["max_grade"] > grade_limit(chain["highway"]) + 0.01:
            raise ValueError(f"Bridge deck grade too steep: {chain}")
    staging = root / ".cache/roads-staging"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)
    tiles = tile_and_write(root, staging, result)
    counts = {name: {"vertices": int(len(data)), "triangles": int(len(triangles))} for name, (data, triangles) in result["surfaces"].items()}
    metadata = {"schema_version": 2, "generated_at": datetime.now(timezone.utc).isoformat(),
                "origin_lon": terrain["origin_lon"], "origin_lat": terrain["origin_lat"],
                "terrain_grid": {key: terrain[key] for key in ["grid_x", "grid_z", "columns", "rows", "spacing_m", "generated_at"]},
                "source_crs_override": args.source_crs, "corridor_m": args.corridor, "nmt_sources": result["nmt_sources"],
                "binary": "res://data/roads_108/network.bin", "tile_m": TILE_M,
                "strides": {"road": ROAD_STRIDE, "deck": ROAD_STRIDE, "tunnel": ROAD_STRIDE, "verge": VERGE_STRIDE, "cover": 3},
                "road_layout": "x y z nx ny nz lateral along half_width lanes flags junction_weight (float32); indices int32",
                "verge_layout": "x z edge_height distance_from_road (float32); runtime blends to terrain",
                "cover_layout": "x y z (float32); restores the surface over carved tunnel bores",
                "tunnel_height_m": TUNNEL_HEIGHT_M,
                "verge_max_m": VERGE_MAX_M, "surface_lift_m": SURFACE_LIFT, "marking_flags": {"oneway": 1, "edge_lines": EDGE_LINES,
                                                                                               "centre_line": CENTRE_LINE, "solid_centre": SOLID_CENTRE},
                "counts": {**counts, **result["counts"]}, "quality": result["quality"], "tiles": tiles,
                "route_samples": result["route_samples"], "route_sample_layout": "x y z left_edge right_edge every 2 m",
                "parapets": result["parapets"], "deck_outlines": result["deck_outlines"],
                "tunnel_walls": result["tunnel_walls"], "portals": result["portals"], "portal_layout": "x1 z1 x2 z2 floor_y top_y",
                "building_clips": result["building_clips"], "included_way_ids": result["included_way_ids"],
                "attribution": "(c) OpenStreetMap contributors / ODbL; GUGiK Geoportal NMT 2018.",
                "limitations": "Carriageway widths from OSM lanes/width tags or class defaults; heights from the 2018 bare-earth NMT, "
                               "so roads rebuilt after 2018 and bridge decks use fitted profiles with assumed clearance."}
    (staging / "roads_108.json").write_text(json.dumps(metadata), encoding="utf-8")
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
    size = (destination / "network.bin").stat().st_size / 1048576
    print(f"ROAD NETWORK: {len(tiles)} tiles, {json.dumps(counts)}, {size:.1f} MiB")


if __name__ == "__main__":
    main()
