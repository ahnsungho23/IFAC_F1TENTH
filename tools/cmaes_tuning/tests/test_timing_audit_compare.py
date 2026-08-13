import unittest

from cmaes_tuning.timing_audit_compare import compare, statistics


def row(scenario, repetition, *, collision=False, delay=50.0, ego_s=12.0, target=-0.3):
    return {
        "candidate": "baseline",
        "scenario_id": scenario,
        "repetition": repetition,
        "classification": "safety_failure" if collision else "success",
        "collision": collision,
        "off_track": False,
        "planner_failure": False,
        "collision_agreement": True,
        "fitness": 100.0 if collision else 1.0 + repetition * 0.01,
        "infrastructure_attempt_count": 1,
        "noise": {
            "planner_log": {"initial_commit_target_d_m": target},
            "end_to_end_timing": {
                "complete_chain": True,
                "avoidance_start_ego_s": ego_s,
                "initial_committed_target_d_m": target,
                "latencies_ms": {
                    "t3_minus_t2": delay,
                    "t4_minus_t3": 2.0,
                    "t5_minus_t4": 10.0,
                    "t8_minus_t6": 1.0,
                    "t9_minus_t8": 5.0,
                    "t9_minus_t1": delay + 50.0,
                },
            },
        },
    }


class TimingAuditCompareTest(unittest.TestCase):
    def test_statistics_uses_sample_std_and_interpolated_p95(self):
        result = statistics([1.0, 2.0, 3.0])
        self.assertEqual(result["count"], 3)
        self.assertAlmostEqual(result["mean"], 2.0)
        self.assertAlmostEqual(result["std"], 1.0)
        self.assertAlmostEqual(result["p95"], 2.9)

    def test_comparison_reports_flip_and_within_scenario_dispersion(self):
        ten = [
            row("a", 0, collision=False, delay=80.0, ego_s=10.0),
            row("a", 1, collision=True, delay=60.0, ego_s=10.4),
            row("b", 0, collision=False, delay=70.0, ego_s=20.0),
            row("b", 1, collision=False, delay=50.0, ego_s=20.2),
        ]
        hundred = [
            row("a", 0, delay=8.0, ego_s=10.0),
            row("a", 1, delay=6.0, ego_s=10.1),
            row("b", 0, delay=7.0, ego_s=20.0),
            row("b", 1, delay=5.0, ego_s=20.1),
        ]
        report = compare(ten, hundred)
        self.assertEqual(report["ten_hz"]["safety_flip_count"], 1)
        self.assertEqual(report["hundred_hz"]["safety_flip_count"], 0)
        self.assertEqual(report["ten_hz"]["binary_safety_outcome_flip_count"], 1)
        self.assertEqual(report["hundred_hz"]["collision_flip_count"], 0)
        self.assertEqual(report["changes"]["safety_flip_count"], -1)
        self.assertAlmostEqual(
            report["changes"]["t3_minus_t2_mean_ratio_100_to_10"], 0.1
        )
        self.assertLess(
            report["changes"][
                "avoidance_start_ego_s_pooled_std_ratio_100_to_10"
            ],
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
