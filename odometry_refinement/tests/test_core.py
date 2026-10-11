"""Geometry, metric, and causal-feature tests. They do not read the recordings."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

from odom_refinement.data.transforms import course_deg, rotate_clockwise_about, se2_from_chord, wgs84_to_utm33, wrap_deg
from odom_refinement.evaluation.trajectory_metrics import ate_summary, horizontal_errors, relative_displacement_error
from odom_refinement.features.odometry import OnlineOdometryFeatures, assert_feature_names_are_causal, extract_odometry_features
from odom_refinement.reliability.models import chronological_split


class FrameTests(unittest.TestCase):
    def test_utm_roundtrip_scale(self):
        lat = np.array([63.23967013096054])
        lon = np.array([9.50187033102811])
        east, north = wgs84_to_utm33(lat, lon)
        self.assertGreater(east[0], 200000)
        self.assertLess(east[0], 300000)
        self.assertGreater(north[0], 7.0e6)
        self.assertLess(north[0], 7.1e6)

    def test_clockwise_rotation_sends_north_to_east(self):
        rotated = rotate_clockwise_about(np.array([[0.0, 1.0]]), np.array([0.0, 0.0]), 90.0)
        np.testing.assert_allclose(rotated[0], [1.0, 0.0], atol=1e-9)

    def test_initial_chord_uses_only_the_opening_segment(self):
        source = np.array([[0.0, 0.0], [0.0, 40.0], [100.0, 40.0]])
        target = np.array([[5.0, 1.0], [5.0, 41.0], [-50.0, 41.0]])
        result = se2_from_chord(source, target, 30.0)
        self.assertTrue(result["valid"])
        self.assertEqual(result["end_index"], 1)
        np.testing.assert_allclose(result["aligned_xy"][1], target[1], atol=1e-6)


class MetricTests(unittest.TestCase):
    def test_identical_trajectory_has_zero_ate_and_rpe(self):
        time_s = np.arange(0, 5, 0.1)
        xy = np.column_stack([time_s * 10.0, np.zeros_like(time_s)])
        valid = np.ones(len(time_s), dtype=bool)
        errors = horizontal_errors(xy, xy, valid)
        self.assertAlmostEqual(ate_summary(errors)["rmse"], 0.0)
        rpe = relative_displacement_error(xy, xy, time_s, valid, 1.0, 0.05)
        self.assertAlmostEqual(rpe["translation_m"]["rmse"], 0.0)

    def test_constant_offset_survives_in_ate_but_not_in_rpe(self):
        time_s = np.arange(0, 5, 0.1)
        ref = np.column_stack([time_s * 8.0, time_s * 2.0])
        est = ref + np.array([12.0, -3.0])
        valid = np.ones(len(time_s), dtype=bool)
        errors = horizontal_errors(est, ref, valid)
        self.assertAlmostEqual(ate_summary(errors)["median"], np.hypot(12.0, 3.0), places=6)
        rpe = relative_displacement_error(est, ref, time_s, valid, 1.0, 0.05)
        self.assertLess(rpe["translation_m"]["max"], 1e-6)


class FeatureTests(unittest.TestCase):
    def test_names_reject_absolute_position(self):
        assert_feature_names_are_causal(("speed_m_s", "course_rate_deg_s"))
        with self.assertRaises(ValueError):
            assert_feature_names_are_causal(("easting_m",))

    def test_online_matches_batch_and_ignores_the_future(self):
        time_s = np.arange(0, 4, 0.1)
        xy = np.column_stack([time_s * 5.0, np.sin(time_s)])
        z = np.zeros_like(time_s)
        batch = extract_odometry_features(time_s, xy, z)
        online = OnlineOdometryFeatures()
        for index in range(len(time_s)):
            latest = online.push(time_s[index], xy[index, 0], xy[index, 1], z[index])
            np.testing.assert_allclose(latest["features"], batch["features"][index], equal_nan=True)
        future = xy.copy()
        future[-1, 0] += 50.0
        changed = extract_odometry_features(time_s, future, z)
        np.testing.assert_allclose(changed["features"][10], batch["features"][10], equal_nan=True)
        self.assertFalse(np.allclose(changed["features"][-1], batch["features"][-1], equal_nan=True))

    def test_course_is_zero_when_travelling_north(self):
        course = course_deg(np.array([0.0, 0.0, 0.0]), np.array([0.0, 1.0, 2.0]))
        self.assertTrue(np.isnan(course[0]))
        np.testing.assert_allclose(course[1:], [0.0, 0.0])


class SplitTests(unittest.TestCase):
    def test_gap_separates_blocks(self):
        time_s = np.arange(0, 100, 0.1)
        split = chronological_split(time_s, train_frac=0.5, val_frac=0.2, gap_s=5.0)
        self.assertIn("train", split)
        self.assertIn("gap", split)
        self.assertIn("test", split)
        train_end = time_s[split == "train"][-1]
        val_start = time_s[split == "val"][0]
        self.assertGreaterEqual(val_start - train_end, 5.0)
        self.assertTrue(np.all(wrap_deg(np.array([190.0, -190.0])) == np.array([-170.0, 170.0])))


if __name__ == "__main__":
    unittest.main()
