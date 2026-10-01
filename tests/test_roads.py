import sys
import unittest
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from import_roads import clean_polyline, limit_longitudinal_grade, remove_short_backtracks, road_cross_sections, rounded_polyline, sample_road_elevation, vertex_normals


class RoadGeometryTests(unittest.TestCase):
    def test_duplicates_removed(self):
        points = clean_polyline([[0, 0], [0, 0], [1, 0]])
        np.testing.assert_array_equal(points, [[0, 0], [1, 0]])

    def test_corner_is_rounded_with_bounded_deviation(self):
        source = np.array([[0, 0], [10, 0], [10, 10]], dtype=float)
        curve = rounded_polyline(source, spacing=0.25, maximum_trim=4)
        self.assertGreater(len(curve), 60)
        self.assertLessEqual(max(LineString(curve).distance(Point(point)) for point in source), 2.01)
        directions = np.diff(curve, axis=0)
        angles = np.degrees(np.arccos(np.clip(np.sum(directions[:-1] * directions[1:], axis=1) /
                                               (np.linalg.norm(directions[:-1], axis=1) * np.linalg.norm(directions[1:], axis=1)), -1, 1)))
        self.assertLess(angles.max(), 8)

    def test_short_backtrack_removed_without_dropping_main_corner(self):
        points = np.array([[0, 0], [10, 0], [9, 1], [20, 0]])
        cleaned = remove_short_backtracks(points)
        self.assertEqual(len(cleaned), 3)
        self.assertTrue(np.all(np.diff(cleaned[:, 0]) > 0))
        self.assertLessEqual(max(LineString(cleaned).distance(Point(point)) for point in points), 1.1)

    def test_cross_sections_share_edges_at_curves(self):
        curve = rounded_polyline([[0, 0], [10, 0], [10, 10]], spacing=0.5)
        sections, tangents, offsets = road_cross_sections(curve, half_width=4.5, lateral_spacing=0.5)
        self.assertEqual(sections.shape, (len(curve), 19, 2))
        self.assertAlmostEqual(offsets[0], -4.5)
        self.assertAlmostEqual(offsets[-1], 4.5)
        self.assertTrue(np.allclose(np.linalg.norm(tangents, axis=1), 1))
        self.assertTrue(np.all(np.linalg.norm(sections[:, -1] - sections[:, 0], axis=1) > 8.99))

    def test_invalid_paths_rejected(self):
        for points in [[], [[0, 0]], [[0, 0], [0, 0]], [[0, 0], [np.nan, 1]]]:
            with self.assertRaises(ValueError):
                rounded_polyline(points)

    def test_lidar_elevation_uses_dense_ground_and_falls_back_across_gap(self):
        terrain = {"grid_x": 0, "grid_z": 0, "spacing_m": 10, "columns": 2, "rows": 2}
        heights = np.full((2, 2), 5.0)
        sections = np.zeros((2, 5, 2))
        sections[0, :, 0] = np.linspace(0.5, 2.5, 5)
        sections[0, :, 1] = 1
        sections[1, :, 0] = np.linspace(7.5, 9.5, 5)
        sections[1, :, 1] = 9
        east, south = np.meshgrid(np.arange(0.25, 3.0, 0.25), np.arange(0.25, 2.0, 0.25))
        ground = np.column_stack([east.ravel(), south.ravel(), np.full(east.size, 5.1)])
        elevations, support, _ = sample_road_elevation(sections, ground, terrain, heights)
        self.assertTrue(support[0].all())
        self.assertFalse(support[1].any())
        self.assertTrue((elevations[0] > elevations[1]).all())
        self.assertTrue(((elevations >= 5.18) & (elevations <= 5.28)).all())
        self.assertLessEqual(np.abs(elevations[1] - elevations[0]).max(), 0.060001)

    def test_surface_normals_face_up(self):
        positions = np.zeros((3, 3, 3), dtype=np.float32)
        for row in range(3):
            for column in range(3):
                positions[row, column] = [column, row * 0.1, row]
        normals = vertex_normals(positions)
        np.testing.assert_allclose(np.linalg.norm(normals, axis=2), 1, atol=1e-6)
        self.assertTrue((normals[:, :, 1] > 0.99).all())

    def test_grade_limiter_removes_spike_and_preserves_allowed_slope(self):
        elevations = np.column_stack([np.arange(9) * 0.04, np.arange(9) * 0.04])
        elevations[4] += 2
        limited = limit_longitudinal_grade(elevations, spacing=0.5, maximum_grade=0.12)
        self.assertLessEqual(np.abs(np.diff(limited, axis=0)).max(), 0.060001)
        np.testing.assert_allclose(limited[[0, 8]], elevations[[0, 8]], atol=0.011)
        smooth = np.column_stack([np.arange(9) * 0.04, np.arange(9) * 0.04])
        np.testing.assert_allclose(limit_longitudinal_grade(smooth, 0.5, 0.12), smooth)


if __name__ == "__main__":
    unittest.main()