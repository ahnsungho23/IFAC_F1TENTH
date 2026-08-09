from pathlib import Path
import tempfile
import unittest

import yaml

from cmaes_tuning.parameter_space import ParameterSpace


ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / "tools" / "cmaes_tuning"


class ParameterSpaceTest(unittest.TestCase):
    def setUp(self):
        self.space = ParameterSpace.load(TOOL / "config" / "parameter_space.yaml")
        self.baseline = ROOT / "src" / "local_planning" / "config" / "local_planning.yaml"

    def test_baseline_matches_operational_yaml(self):
        self.space.validate_baseline(self.baseline)

    def test_round_trip_and_ordering(self):
        baseline = self.space.baseline_physical()
        decoded = self.space.decode(self.space.encode(baseline))
        for name, value in baseline.items():
            self.assertAlmostEqual(decoded[name], value)
        patch = self.space.planner_patch(decoded)
        self.assertGreater(patch["pre_apex_distances_m"][0], patch["pre_apex_distances_m"][1])
        self.assertGreater(patch["pre_apex_distances_m"][1], patch["pre_apex_distances_m"][2])
        self.assertLess(patch["post_apex_distances_m"][0], patch["post_apex_distances_m"][1])
        self.assertLess(patch["post_apex_distances_m"][1], patch["post_apex_distances_m"][2])
        self.assertLess(patch["transition_distance_scales"][0], patch["transition_distance_scales"][1])
        self.assertLess(patch["transition_distance_scales"][1], patch["transition_distance_scales"][2])

    def test_only_whitelist_is_patched(self):
        with tempfile.TemporaryDirectory() as temporary:
            candidate = Path(temporary) / "candidate.yaml"
            self.space.write_candidate_yaml(self.baseline, candidate, [0.5] * self.space.dimension)
            original = yaml.safe_load(self.baseline.read_text())
            changed = yaml.safe_load(candidate.read_text())
            original_params = original["local_planner_node"]["ros__parameters"]
            changed_params = changed["local_planner_node"]["ros__parameters"]
            allowed = {
                "safety_margin_m",
                "obstacle_longitudinal_padding_m",
                "pre_apex_distances_m",
                "post_apex_distances_m",
                "transition_distance_scales",
                "outside_line_transition_scale",
                "minimum_target_offset_m",
                "wall_safety_margin_m",
            }
            self.assertEqual(
                {key for key in original_params if original_params[key] != changed_params[key]},
                allowed,
            )


if __name__ == "__main__":
    unittest.main()
