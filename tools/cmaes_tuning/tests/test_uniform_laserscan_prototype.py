import math
import random
import unittest

import numpy as np

from cmaes_tuning.lidar_beam_contract import BackendBeamContract
from cmaes_tuning.uniform_laserscan_prototype import (
    LaserScanContractMode,
    LEGACY_SCAN_MODE,
    PROTOTYPE_UNIFORM_SCAN_MODE,
    UniformLaserScanContract,
    float32,
    oriented_rectangle_ranges,
    physical_angles_for_mode,
)


class UniformLaserScanPrototypeTest(unittest.TestCase):
    def test_all_beams_equal_published_metadata(self):
        for count in (2, 3, 7, 17, 360, 1080, 1440, 4096):
            contract = UniformLaserScanContract.from_fov(4.7, count)
            np.testing.assert_array_equal(
                contract.physical_relative_angles_rad(),
                contract.published_relative_angles_rad(),
            )
            self.assertEqual(contract.physical_relative_angles_rad()[0], contract.angle_min_rad)
            self.assertEqual(
                contract.physical_relative_angles_rad()[-1],
                contract.angle_min_rad + (count - 1) * contract.angle_increment_rad,
            )

    def test_ros_float32_endpoint_precision_is_bounded(self):
        contract = UniformLaserScanContract.from_fov(4.7, 1080)
        computed_last = contract.physical_relative_angles_rad()[-1]
        self.assertEqual(contract.angle_max_rad, float32(computed_last))
        self.assertLessEqual(abs(contract.angle_max_rad - computed_last), np.finfo(np.float32).eps)

    def test_fov_validation_and_off_by_one(self):
        for invalid in (0.0, -1.0, math.inf, math.nan):
            with self.assertRaises(ValueError):
                UniformLaserScanContract.from_fov(invalid, 1080)
        with self.assertRaises(ValueError):
            UniformLaserScanContract.from_fov(4.7, 1)
        contract = UniformLaserScanContract.from_fov(2.0 * math.pi, 2001)
        self.assertEqual(len(contract.physical_relative_angles_rad()), 2001)

    def test_yaw_wrap_does_not_change_contract(self):
        contract = UniformLaserScanContract.from_fov(4.7, 1080)
        relative = contract.physical_relative_angles_rad()
        for yaw in (-9.0 * math.pi, -math.pi, 0.0, math.pi, 11.0 * math.pi):
            np.testing.assert_allclose(
                contract.physical_world_angles_rad(yaw) - yaw,
                relative,
                rtol=0.0,
                atol=8.0 * np.finfo(float).eps * max(1.0, abs(yaw)),
            )

    def test_contract_mode_is_explicit_and_legacy_is_unchanged(self):
        with self.assertRaises(ValueError):
            physical_angles_for_mode("UNSPECIFIED", 4.7, 1080, 0.3)
        expected = BackendBeamContract(4.7, 1080).physical_angles_rad(0.3)
        actual = physical_angles_for_mode(
            LEGACY_SCAN_MODE, 4.7, 1080, 0.3
        )
        np.testing.assert_array_equal(actual, expected)
        prototype = physical_angles_for_mode(
            PROTOTYPE_UNIFORM_SCAN_MODE, 4.7, 1080, 0.3
        )
        self.assertIs(LEGACY_SCAN_MODE, LaserScanContractMode.LEGACY_QUANTIZED_2000)
        self.assertIs(
            PROTOTYPE_UNIFORM_SCAN_MODE,
            LaserScanContractMode.PROTOTYPE_UNIFORM_INCLUSIVE,
        )
        self.assertGreater(float(np.max(np.abs(prototype - actual))), 1.0e-3)

    def test_random_rectangle_scans_are_deterministic(self):
        generator = random.Random(39039)
        for _ in range(128):
            count = generator.choice((17, 180, 360, 1080, 1440))
            contract = UniformLaserScanContract.from_fov(generator.uniform(1.0, 5.5), count)
            yaw = generator.uniform(-math.pi, math.pi)
            directions = contract.directions(yaw)
            bearing = generator.uniform(-2.5, 2.5) + yaw
            distance = generator.uniform(0.5, 20.0)
            centre = np.asarray([distance * math.cos(bearing), distance * math.sin(bearing)])
            kwargs = dict(
                origin_xy=np.zeros(2),
                directions=directions,
                centre_xy=centre,
                rectangle_yaw_rad=generator.uniform(-math.pi, math.pi),
                width_m=generator.uniform(0.1, 1.5),
                height_m=generator.uniform(0.1, 2.0),
            )
            first = oriented_rectangle_ranges(**kwargs)
            second = oriented_rectangle_ranges(**kwargs)
            np.testing.assert_array_equal(first, second)


if __name__ == "__main__":
    unittest.main()
