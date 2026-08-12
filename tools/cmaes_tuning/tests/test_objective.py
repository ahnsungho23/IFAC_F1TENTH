from pathlib import Path
import copy
import unittest

import yaml

from cmaes_tuning.objective import candidate_fitness


ROOT = Path(__file__).resolve().parents[3]
CONFIG = yaml.safe_load(
    (ROOT / "tools" / "cmaes_tuning" / "config" / "tuning_config.yaml").read_text()
)


def episode(
    *, collision=False, off_track=False, invalid_suffix=False, safe_stop=False,
    p0_fallback=False, planner_failure=False, worst=False,
):
    value = 1.0 if worst else 0.0
    metrics = {
        "completed": not (collision or off_track or planner_failure),
        "episode_timeout_s": 1.0,
        "completion_time_s": value,
        "progress_fraction": 0.0 if worst else 1.0,
        "minimum_obstacle_clearance_m": 0.0 if worst else 1.0,
        "minimum_wall_clearance_m": 0.0 if worst else 1.0,
        "steering_total_variation_per_s": 100.0 if worst else 0.0,
        "planned_curvature_rate_rms_radpm2": 100.0 if worst else 0.0,
    }
    return {
        "valid": True,
        "failure": {
            "collision": collision,
            "off_track": off_track,
            "invalid_suffix": invalid_suffix,
            "safe_stop": safe_stop,
            "p0_fallback": p0_fallback,
            "planner_failure": planner_failure,
        },
        "metrics": metrics,
    }


class ObjectiveTest(unittest.TestCase):
    def test_any_one_hard_failure_dominates_worst_zero_failure(self):
        worst_safe = candidate_fitness([episode(worst=True)], CONFIG)
        best_failure = candidate_fitness([episode(collision=True, worst=False)], CONFIG)
        self.assertLess(worst_safe["fitness"], best_failure["fitness"])

    def test_episode_failure_is_or_not_double_count(self):
        single = candidate_fitness([episode(collision=True)], CONFIG)
        overlapping = candidate_fitness(
            [episode(collision=True, off_track=True, planner_failure=True)], CONFIG
        )
        self.assertEqual(single["failure_count"], 1)
        self.assertEqual(overlapping["failure_count"], 1)

    def test_invalid_episode_is_rejected(self):
        invalid = copy.deepcopy(episode())
        invalid["valid"] = False
        with self.assertRaises(ValueError):
            candidate_fitness([invalid], CONFIG)

    def test_every_p3_fail_closed_signal_is_a_hard_penalty(self):
        for name in ("invalid_suffix", "safe_stop", "p0_fallback"):
            with self.subTest(name=name):
                result = candidate_fitness([episode(**{name: True})], CONFIG)
                self.assertEqual(result["failure_count"], 1)


if __name__ == "__main__":
    unittest.main()
