#!/usr/bin/env python3

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from generate_global_trajectory import (
    lateral_speed_limits,
    load_velocity_limits,
    velocity_profile,
)


class VelocityLimitsTest(unittest.TestCase):
    @staticmethod
    def _closed_test_path(count: int = 240) -> tuple[np.ndarray, np.ndarray]:
        angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        points = np.column_stack((8.0 * np.cos(angle), 5.0 * np.sin(angle)))
        kappa = 0.03 + 0.42 * (0.5 + 0.5 * np.sin(2.0 * angle))
        return points, kappa

    def test_constant_table_matches_removed_scalar_profile(self) -> None:
        points, kappa = self._closed_test_path()
        limits = np.array(
            [
                [0.0, 3.7, 2.0, 5.5],
                [9.0, 3.7, 2.0, 5.5],
            ]
        )
        actual_v, actual_ax, actual_lap = velocity_profile(points, kappa, 6.5, 2.0, limits)

        seg = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        expected_v = np.clip(np.sqrt(5.5 / np.maximum(np.abs(kappa), 1e-4)), 2.0, 6.5)
        for _ in range(8):
            for i in range(len(expected_v)):
                j = (i + 1) % len(expected_v)
                possible = math.sqrt(expected_v[i] ** 2 + 2.0 * 3.7 * seg[i])
                expected_v[j] = min(expected_v[j], possible)
            for i in range(len(expected_v) - 1, -1, -1):
                j = (i - 1) % len(expected_v)
                possible = math.sqrt(expected_v[i] ** 2 + 2.0 * 2.0 * seg[j])
                expected_v[j] = min(expected_v[j], possible)
        expected_ax = (np.roll(expected_v, -1) ** 2 - expected_v ** 2) / (2.0 * seg)
        expected_lap = float(np.sum(seg / expected_v))

        np.testing.assert_allclose(actual_v, expected_v, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(actual_ax, expected_ax, rtol=0.0, atol=1e-12)
        self.assertAlmostEqual(actual_lap, expected_lap, places=12)

    def test_variable_table_changes_each_speed_zone_and_obeys_constraints(self) -> None:
        count = 320
        angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        points = np.column_stack((10.0 * np.cos(angle), 6.0 * np.sin(angle)))
        kappa = np.full(count, 0.03)
        kappa[55:105] = 0.22
        kappa[190:250] = 0.40
        variable = np.array(
            [
                [0.0, 4.5, 5.0, 8.0],
                [2.0, 4.0, 4.5, 7.0],
                [4.0, 3.0, 3.5, 6.0],
                [6.0, 1.5, 2.0, 4.5],
                [8.0, 0.8, 1.5, 3.5],
            ]
        )
        constant = np.array(
            [
                [0.0, 4.5, 5.0, 8.0],
                [8.0, 4.5, 5.0, 8.0],
            ]
        )

        velocity, _, _ = velocity_profile(points, kappa, 7.0, 1.0, variable)
        constant_velocity, _, _ = velocity_profile(points, kappa, 7.0, 1.0, constant)
        seg = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        next_velocity = np.roll(velocity, -1)
        accel_limit = np.interp(velocity, variable[:, 0], variable[:, 1])
        decel_limit = np.interp(next_velocity, variable[:, 0], variable[:, 2])
        lateral_limit = np.interp(velocity, variable[:, 0], variable[:, 3])

        lateral_excess = velocity**2 * np.abs(kappa) - lateral_limit
        accel_excess = next_velocity**2 - velocity**2 - 2.0 * accel_limit * seg
        decel_excess = velocity**2 - next_velocity**2 - 2.0 * decel_limit * seg

        self.assertGreater(np.count_nonzero((velocity >= 2.0) & (velocity < 4.0)), 0)
        self.assertGreater(np.count_nonzero((velocity >= 4.0) & (velocity < 6.0)), 0)
        self.assertGreater(np.count_nonzero(velocity >= 6.0), 0)
        self.assertGreater(float(np.ptp(accel_limit)), 1.0)
        self.assertGreater(float(np.max(np.abs(velocity - constant_velocity))), 1.0)
        self.assertLessEqual(float(np.max(lateral_excess)), 1e-10)
        self.assertLessEqual(float(np.max(accel_excess)), 1e-10)
        self.assertLessEqual(float(np.max(decel_excess)), 1e-10)

    def test_speed_dependent_lateral_limit_solves_implicit_equation(self) -> None:
        # ay_max(v) = 8-v and kappa=0.5 -> 0.5*v^2 = 8-v.
        limits = np.array(
            [
                [0.0, 3.0, 3.0, 8.0],
                [6.0, 3.0, 3.0, 2.0],
            ]
        )
        expected = -1.0 + math.sqrt(17.0)
        actual = lateral_speed_limits(np.array([0.5]), 6.0, limits)[0]
        self.assertAlmostEqual(float(actual), expected, places=12)

    def test_loader_rejects_table_that_does_not_cover_max_speed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.csv"
            path.write_text(
                "# speed,accel,decel,lateral\n0,3,4,5\n4,2,3,4\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "Extend the table"):
                load_velocity_limits(path, max_speed=6.5)

if __name__ == "__main__":
    unittest.main()
