import json
import math
from pathlib import Path
import random
import unittest

import numpy as np

from cmaes_tuning.lidar_beam_contract import (
    BackendBeamContract,
    endpoint_xy,
    inspect_authoritative_sources,
    metadata_angles_rad,
    old_analytic_angles_rad,
    wrapped_angle_delta_rad,
)


FIXTURE = Path(__file__).parent / "fixtures" / "validation039_beam_contract.json"


def reference_cpp_indices(contract: BackendBeamContract, yaw_rad: float) -> np.ndarray:
    """Independent literal translation of the source loop used as a test oracle."""

    theta = contract.theta_discretization * (
        yaw_rad - contract.scan_fov_rad / 2.0
    ) / (2.0 * math.pi)
    theta = math.fmod(theta, contract.theta_discretization)
    while theta < 0.0:
        theta += contract.theta_discretization
    output = []
    increment = contract.theta_discretization * (
        contract.scan_fov_rad / (contract.scan_beams - 1)
    ) / (2.0 * math.pi)
    for _ in range(contract.scan_beams):
        output.append(int(theta))
        theta += increment
        while theta >= contract.theta_discretization:
            theta -= contract.theta_discretization
    return np.asarray(output, dtype=np.int64)


class LidarBeamContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        cls.sources = inspect_authoritative_sources()
        cls.contract = BackendBeamContract(
            cls.fixture["scan_fov_rad"],
            cls.fixture["scan_beams"],
            int(cls.sources["theta_discretization"]),
        )

    def test_backend_source_contract(self):
        self.assertEqual(self.sources["theta_discretization"], 2000)
        self.assertIn("num_beams - 1", self.sources["backend_increment_source"])
        self.assertIn("theta_dis_ - 1", self.sources["backend_table_source"])
        self.assertIn("static_cast<int>", self.sources["backend_conversion_source"])
        self.assertIn("scan_fov / scan_beams", self.sources["publisher_increment_source"])

    def test_published_laserscan_index_contract(self):
        values = metadata_angles_rad(
            self.fixture["angle_min_rad"],
            self.fixture["angle_increment_rad"],
            self.fixture["scan_beams"],
        )
        indices = [0, self.fixture["scan_beams"] // 2, self.fixture["scan_beams"] - 1]
        indices.extend(random.Random(39039).sample(range(self.fixture["scan_beams"]), 16))
        for index in indices:
            expected = (
                self.fixture["angle_min_rad"]
                + index * self.fixture["angle_increment_rad"]
            )
            self.assertEqual(values[index], expected)

    def test_full_backend_scan_matches_source_loop(self):
        for yaw in (-3.0, -0.25, 0.0, 1.2, math.pi - 1.0e-6):
            np.testing.assert_array_equal(
                self.contract.table_indices(yaw), reference_cpp_indices(self.contract, yaw)
            )

    def test_recorded_validation039_core(self):
        displacements = []
        for row in self.fixture["core"]:
            yaw = row["yaw_rad"]
            beam = row["beam_index"]
            backend = self.contract.physical_angles_rad(yaw)[beam]
            metadata = metadata_angles_rad(
                self.fixture["angle_min_rad"],
                self.fixture["angle_increment_rad"],
                self.fixture["scan_beams"],
                yaw_rad=yaw,
            )[beam]
            endpoints = endpoint_xy([backend, metadata], [row["range_m"], row["range_m"]])
            displacements.append(float(np.linalg.norm(endpoints[0] - endpoints[1])))
        self.assertGreaterEqual(min(displacements), 0.013)
        self.assertLessEqual(max(displacements), 0.036)

    def test_random_deterministic_geometry_round_trip(self):
        generator = random.Random(39039)
        for case in range(128):
            yaw = generator.uniform(-math.pi, math.pi)
            index = generator.randrange(self.contract.scan_beams)
            distance = generator.uniform(0.1, 30.0)
            origin = (generator.uniform(-20.0, 20.0), generator.uniform(-20.0, 20.0))
            table_index = int(self.contract.table_indices(yaw)[index])
            self.assertIn(index, self.contract.scan_indices_for_backend_bin(yaw, table_index))
            angle = self.contract.physical_angles_rad(yaw)[index]
            point = endpoint_xy(
                [angle], [distance], origin_x_m=origin[0], origin_y_m=origin[1]
            )[0]
            recovered = math.atan2(point[1] - origin[1], point[0] - origin[0])
            self.assertLess(abs(float(wrapped_angle_delta_rad(recovered, angle))), 2.0e-14)

    def test_old_analytic_fov_n_minus_1_is_not_a_fallback(self):
        row = self.fixture["core"][0]
        backend = self.contract.physical_angles_rad(row["yaw_rad"])
        old = old_analytic_angles_rad(
            self.fixture["scan_fov_rad"],
            self.fixture["scan_beams"],
            yaw_rad=row["yaw_rad"],
        )
        error = np.abs(wrapped_angle_delta_rad(old, backend))
        self.assertGreater(float(np.max(error)), math.radians(0.1))
        beam_error = abs(float(error[row["beam_index"]]))
        self.assertGreater(row["range_m"] * math.sin(beam_error), 0.01)


if __name__ == "__main__":
    unittest.main()
