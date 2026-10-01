import sys
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from shapely.geometry import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from import_lidar import ground_at, roof_geometry, source_files
from import_landmark import build_landmark, detailed_roof, load_selections, publish_landmark, read_registry, resolve_selection


class LidarGeometryTests(unittest.TestCase):
    def setUp(self):
        self.footprint = Polygon([(-6, -10), (6, -10), (6, 10), (-6, 10)])
        east, south = np.meshgrid(np.linspace(-5.5, 5.5, 12), np.linspace(-9.5, 9.5, 20))
        self.samples = np.column_stack([east.ravel(), south.ravel()])

    def test_flat_roof(self):
        wall, triangles, kind, residual = roof_geometry(self.footprint, self.samples, np.full(len(self.samples), 12.0))
        self.assertEqual(kind, "measured_flat")
        self.assertIsNone(residual)
        self.assertTrue(all(vertex[1] == 12 for vertex in wall))
        self.assertAlmostEqual(sum(Polygon([(vertex[0], vertex[2]) for vertex in triangle]).area for triangle in triangles), self.footprint.area)

    def test_gable_fit_and_footprint(self):
        wall, triangles, kind, residual = roof_geometry(self.footprint, self.samples, 16 - 0.6 * np.abs(self.samples[:, 0]))
        self.assertEqual(kind, "fitted_gable")
        self.assertLess(residual, 0.01)
        self.assertEqual(len(wall), 6)
        self.assertTrue(all(self.footprint.covers(Polygon([(vertex[0], vertex[2]) for vertex in triangle])) for triangle in triangles))
        self.assertAlmostEqual(sum(Polygon([(vertex[0], vertex[2]) for vertex in triangle]).area for triangle in triangles), self.footprint.area)

    def test_irregular_footprint_is_not_forced_to_gable(self):
        footprint = Polygon([(-6, -10), (6, -10), (6, 0), (0, 0), (0, 10), (-6, 10)])
        _, triangles, kind, _ = roof_geometry(footprint, self.samples, 16 - 0.6 * np.abs(self.samples[:, 0]))
        self.assertEqual(kind, "measured_flat")
        self.assertTrue(all(footprint.covers(Polygon([(vertex[0], vertex[2]) for vertex in triangle])) for triangle in triangles))

    def test_terrain_sampling_matches_triangle_split(self):
        terrain = {"grid_x": 0, "grid_z": 0, "spacing_m": 1, "columns": 2, "rows": 2}
        heights = np.array([[0, 2], [4, 10]])
        actual = ground_at(np.array([0.25, 0.75]), np.array([0.25, 0.75]), terrain, heights)
        np.testing.assert_allclose(actual, [1.5, 6.5])

    def test_copc_duplicate_is_not_counted_twice(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            for filename in ["tile.laz", "tile.copc.laz", "other.copc.laz"]:
                (folder / filename).touch()
            self.assertEqual([filename.name for filename in source_files(folder)], ["other.copc.laz", "tile.laz"])

    def test_native_landmark_keeps_source_heights_and_removes_only_duplicate_xy(self):
        samples = np.column_stack([self.samples, 16 - 0.6 * np.abs(self.samples[:, 0])])
        samples = np.concatenate([samples, [[samples[0, 0], samples[0, 1], samples[0, 2] - 1]]])
        vertices, indices, _, stats = detailed_roof(self.footprint, samples)
        self.assertEqual(stats["unique_source_xy"], len(self.samples))
        self.assertEqual(stats["duplicate_xy_removed"], 1)
        self.assertGreater(stats["roof_area_coverage"], 0.99)
        for east, south, elevation in samples[:-1]:
            match = vertices[(vertices[:, 0] == east) & (vertices[:, 2] == south)]
            self.assertEqual(len(match), 1)
            self.assertEqual(match[0, 1], elevation)
        triangles = vertices[indices][:, :, [0, 2]]
        self.assertTrue(all(self.footprint.buffer(0.0001).covers(Polygon(triangle)) for triangle in triangles))

    def test_native_landmark_rejects_large_source_gaps(self):
        sparse = self.samples[np.abs(self.samples[:, 0]) > 3]
        samples = np.column_stack([sparse, np.full(len(sparse), 12)])
        with self.assertRaises(ValueError):
            detailed_roof(self.footprint, samples)


class LandmarkWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "data").mkdir()
        (self.root / ".cache").mkdir()
        self.station = {"osm_id": "1", "name": "station", "roof_vertices": [[1, 2, 3]]}
        self.forum = {"osm_id": "2", "name": "forum", "roof_texture": {"file": "res://data/landmark_2.png"}}
        self.path = self.root / "data/landmarks_108.json"
        self.path.write_text(json.dumps({"schema_version": 1, "landmarks": [self.station]}), encoding="utf-8")
        self.configuration = {"schema_version": 1, "source_crs": "EPSG:2180", "landmarks": [
            {"key": "station", "northing": 721345.37, "easting": 476906.75},
            {"key": "forum", "northing": 720639.46, "easting": 476850.34},
        ]}
        self.config_path = self.root / "data/landmark_selections.json"

    def write_configuration(self):
        self.config_path.write_text(json.dumps(self.configuration), encoding="utf-8")

    def test_named_selection_and_all_default(self):
        self.write_configuration()
        self.assertEqual([entry["key"] for entry in load_selections(self.config_path)], ["station", "forum"])
        selected = load_selections(self.config_path, ["forum"])
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["source_crs"], "EPSG:2180")
        self.assertEqual(selected[0]["northing"], 720639.46)

    def test_unknown_duplicate_and_unsafe_keys_rejected(self):
        self.write_configuration()
        with self.assertRaises(ValueError):
            load_selections(self.config_path, ["missing"])
        for invalid in ["station", "../outside"]:
            self.configuration["landmarks"][1]["key"] = invalid
            self.write_configuration()
            with self.assertRaises(ValueError):
                load_selections(self.config_path)

    def test_incorrect_building_identity_rejected(self):
        selection = {**self.configuration["landmarks"][1], "source_crs": "EPSG:2180", "expected_osm_id": "2"}
        with patch("import_landmark.select_building", return_value=({"id": 99}, None)):
            with self.assertRaises(ValueError):
                resolve_selection(self.root, selection)

    def test_publish_preserves_other_landmark_and_is_idempotent(self):
        staging = self.root / ".cache/texture.png"
        for content in [b"first image", b"updated image"]:
            staging.write_bytes(content)
            publish_landmark(self.root, self.forum, staging)
            registry = read_registry(self.root)
            self.assertEqual(registry["landmarks"], [self.station, self.forum])
            self.assertEqual((self.root / "data/landmark_2.png").read_bytes(), content)

    def test_publish_failure_restores_texture_and_registry(self):
        staging = self.root / ".cache/texture.png"
        staging.write_bytes(b"new")
        image = self.root / "data/landmark_2.png"
        image.write_bytes(b"original")
        original = self.path.read_bytes()
        replace = Path.replace
        def fail_registry(source, destination):
            if source.name == "landmarks_108.tmp.json":
                raise OSError("simulated registry write failure")
            return replace(source, destination)
        with patch.object(Path, "replace", fail_registry):
            with self.assertRaises(OSError):
                publish_landmark(self.root, self.forum, staging)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertEqual(image.read_bytes(), b"original")

    def test_unchanged_selection_skips_point_extraction(self):
        image = self.root / "data/landmark_2.png"
        image.write_bytes(b"image")
        self.forum.update({"input_fingerprint": "same", "texture_sha256": hashlib.sha256(b"image").hexdigest()})
        self.path.write_text(json.dumps({"schema_version": 1, "landmarks": [self.station, self.forum]}), encoding="utf-8")
        selection = {**self.configuration["landmarks"][1], "source_crs": "EPSG:2180"}
        original = self.path.read_bytes()
        with patch("import_landmark.resolve_selection", return_value=({"id": 2}, None)), \
                patch("import_landmark.input_fingerprint", return_value="same"), \
                patch("import_landmark.extract_points") as extract:
            build_landmark(self.root, selection)
            extract.assert_not_called()
        self.assertEqual(self.path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()