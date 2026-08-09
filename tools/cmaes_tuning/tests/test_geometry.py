from pathlib import Path
import math
import unittest

from cmaes_tuning.geometry import (
    OccupancyDistanceField,
    footprint_obstacle_clearance,
)


ROOT = Path(__file__).resolve().parents[3]


class GeometryTest(unittest.TestCase):
    def test_rectangle_clearance_and_overlap(self):
        obstacle = {
            "shape": "rect",
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "width": 0.5,
            "height": 0.5,
        }
        clearance, overlap = footprint_obstacle_clearance(0.0, 0.0, 0.0, 0.56, 0.287, obstacle)
        self.assertFalse(overlap)
        self.assertAlmostEqual(clearance, 1.0 - 0.25 - 0.28, places=6)
        clearance, overlap = footprint_obstacle_clearance(0.6, 0.0, 0.0, 0.56, 0.287, obstacle)
        self.assertTrue(overlap)
        self.assertEqual(clearance, 0.0)

    def test_clean_map_footprint_clearance_is_finite(self):
        field = OccupancyDistanceField(
            ROOT / "src" / "monte_carlo_localization" / "maps" / "ifac_track.yaml"
        )
        clearance, overlap = field.footprint_clearance(
            -11.792322542814967,
            0.10846627546883192,
            0.007960411704868452,
            0.56,
            0.287,
            0.0125,
        )
        self.assertFalse(overlap)
        self.assertTrue(math.isfinite(clearance))
        self.assertGreater(clearance, 0.0)


if __name__ == "__main__":
    unittest.main()
