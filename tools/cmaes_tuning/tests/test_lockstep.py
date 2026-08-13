import unittest
from pathlib import Path

import yaml

from cmaes_tuning.lockstep import common_random_number_schedule, summarize_lockstep_runs


class LockstepHelpersTest(unittest.TestCase):
    def test_production_yaml_defaults_keep_lockstep_disabled(self):
        workspace = Path(__file__).resolve().parents[3]
        paths = (
            workspace / "src/obstacle_detector/config/obstacle_detector.yaml",
            workspace / "src/local_planning/config/local_planning.yaml",
            workspace / "src/state_machine/config/state_machine.yaml",
        )
        for path in paths:
            payload = yaml.safe_load(path.read_text(encoding="utf-8"))
            parameters = next(iter(payload.values()))["ros__parameters"]
            self.assertIs(parameters["lockstep_mode"], False, str(path))

    def test_common_random_numbers_repeat_pairs_for_every_candidate(self):
        jobs = common_random_number_schedule(["a", "b"], ["s1"], [101, 202])
        self.assertEqual(
            [(job["scenario_id"], job["simulator_seed"]) for job in jobs[:2]],
            [(job["scenario_id"], job["simulator_seed"]) for job in jobs[2:]],
        )

    def test_summary_accepts_identical_runs(self):
        result = {
            "valid": True,
            "physics_state_sequence_hash": "a",
            "scan_hash_sequence_hash": "b",
            "static_geometry_sequence_hash": "c",
            "avoid_waypoints_hash_sequence_hash": "d",
            "controller_command_sequence_hash": "e",
            "trajectory_hash": "f",
            "collision": False,
            "confirmation_scan_index": 12,
            "commitment_scan_index": 20,
            "selected_side": "left",
            "entry_transition_scale": 1.0,
            "exit_transition_scale": 1.0,
            "committed_target_d": 0.5,
            "fitness": 0.25,
        }
        summary = summarize_lockstep_runs([dict(result) for _ in range(10)])
        self.assertTrue(summary["accepted"])
        self.assertEqual(summary["fitness_statistics"]["std"], 0.0)

    def test_summary_rejects_a_path_or_fitness_divergence(self):
        common = {
            "valid": True,
            "physics_state_sequence_hash": "physics",
            "scan_hash_sequence_hash": "scan",
            "static_geometry_sequence_hash": "static",
            "avoid_waypoints_hash_sequence_hash": "path-a",
            "controller_command_sequence_hash": "command",
            "trajectory_hash": "trajectory",
            "collision": False,
            "confirmation_scan_index": 12,
            "commitment_scan_index": 20,
            "selected_side": "left",
            "entry_transition_scale": 1.0,
            "exit_transition_scale": 1.0,
            "committed_target_d": 0.5,
            "fitness": 0.25,
        }
        divergent = dict(common)
        divergent["avoid_waypoints_hash_sequence_hash"] = "path-b"
        divergent["fitness"] = 0.3
        summary = summarize_lockstep_runs([common, divergent])
        self.assertFalse(summary["accepted"])
        self.assertFalse(
            summary["exact_agreement"]["avoid_waypoints_hash_sequence_hash"])
        self.assertGreater(summary["fitness_statistics"]["std"], 0.0)

    def test_summary_rejects_a_safety_outcome_flip(self):
        common = {
            "valid": True,
            "physics_state_sequence_hash": "physics",
            "scan_hash_sequence_hash": "scan",
            "static_geometry_sequence_hash": "static",
            "avoid_waypoints_hash_sequence_hash": "path",
            "controller_command_sequence_hash": "command",
            "trajectory_hash": "trajectory",
            "collision": False,
            "off_track": False,
            "planner_failure": False,
            "confirmation_scan_index": 12,
            "commitment_scan_index": 20,
            "selected_side": "left",
            "entry_transition_scale": 1.0,
            "exit_transition_scale": 1.0,
            "committed_target_d": 0.5,
            "fitness": 0.25,
        }
        unsafe = dict(common)
        unsafe["collision"] = True
        summary = summarize_lockstep_runs([common, unsafe])
        self.assertFalse(summary["accepted"])
        self.assertEqual(summary["safety_outcome_flip_count"], 1)


if __name__ == "__main__":
    unittest.main()
