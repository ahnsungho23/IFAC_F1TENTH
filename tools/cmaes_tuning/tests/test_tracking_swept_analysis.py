import math
from types import SimpleNamespace
import unittest

from cmaes_tuning.parallel_diagnostic import (
    compare_deterministic_results,
    select_worker_count,
)
from cmaes_tuning.tracking_swept_analysis import (
    interpolate_angle,
    percentile_summary,
    project_to_path,
)


def waypoint(x, y, d=0.0, curvature=0.0, speed=4.0):
    return SimpleNamespace(
        x_m=x, y_m=y, psi_rad=0.0, kappa_radpm=curvature,
        vx_mps=speed, s_m=x, d_m=d)


class TrackingSweptAnalysisTest(unittest.TestCase):
    def test_path_projection_has_signed_normal_error(self):
        projection = project_to_path(1.0, 0.25, [waypoint(0.0, 0.0), waypoint(2.0, 0.0)])
        self.assertIsNotNone(projection)
        self.assertAlmostEqual(projection.x, 1.0)
        self.assertAlmostEqual(projection.y, 0.0)
        self.assertAlmostEqual(projection.lateral_error, 0.25)
        self.assertAlmostEqual(projection.heading, 0.0)

    def test_angle_interpolation_uses_short_arc(self):
        value = interpolate_angle(math.radians(179.0), math.radians(-179.0), 0.5)
        self.assertAlmostEqual(abs(value), math.pi, places=12)

    def test_percentile_summary_is_deterministic(self):
        summary = percentile_summary(range(1, 101))
        self.assertEqual(summary["count"], 100)
        self.assertEqual(summary["max"], 100.0)
        self.assertAlmostEqual(summary["p95"], 95.05)
        self.assertAlmostEqual(summary["p99"], 99.01)

    def test_parallel_gate_requires_exact_metrics(self):
        lockstep = {
            "physics_state_sequence_hash": "a",
            "scan_hash_sequence_hash": "b",
            "static_geometry_sequence_hash": "c",
            "avoid_waypoints_hash_sequence_hash": "d",
            "controller_command_sequence_hash": "e",
            "trajectory_hash": "f",
            "committed_target_d": 0.5,
            "selected_side": "left",
            "entry_transition_scale": 1.0,
            "exit_transition_scale": 0.5,
            "effective_entry_transition_scale": 0.8,
            "effective_exit_transition_scale": 0.5,
            "fitness": 12.0,
            "collision": True,
            "off_track": False,
            "planner_failure": False,
            "scenario_success": False,
        }
        episode = {
            "classification": "safety_failure",
            "failure": {"collision": True},
            "metrics": {"completion_time_s": 1.0},
            "performance_cost": {"total": 3.0},
        }
        agreement = compare_deterministic_results(lockstep, episode, lockstep, episode)
        self.assertTrue(agreement["accepted"])
        changed = {**episode, "metrics": {"completion_time_s": 1.0 + 1.0e-15}}
        disagreement = compare_deterministic_results(lockstep, episode, lockstep, changed)
        self.assertFalse(disagreement["accepted"])
        self.assertEqual(disagreement["mismatch_count"], 1)

    def test_worker_selection_stops_at_throughput_plateau(self):
        def stage(workers, rate):
            return {
                "workers": workers, "episodes_per_min": rate,
                "deterministic_mismatch_count": 0,
                "infrastructure_failure_count": 0,
                "process_contamination_count": 0,
                "dds_collision_count": 0,
                "swap_increase": False,
            }

        selected, reason = select_worker_count([
            stage(1, 2.0), stage(2, 3.8), stage(4, 6.8), stage(8, 6.9)])
        self.assertEqual(selected, 4)
        self.assertIn("workers=8", reason)


if __name__ == "__main__":
    unittest.main()
