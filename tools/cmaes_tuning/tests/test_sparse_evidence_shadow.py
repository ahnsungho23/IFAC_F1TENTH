import unittest

import numpy as np

from cmaes_tuning.sparse_evidence_shadow import (
    Fragment,
    ShadowIdentityState,
    compatible_with_identity,
    extract_production_fragments,
    has_timestamp_persistence,
)


def fragment(points, beams=(10,), outcome="TEST"):
    return Fragment(
        tuple(beams), np.asarray(points, dtype=float), np.ones(len(beams)),
        (0,), "TEST", outcome,
    )


class SparseEvidenceShadowTests(unittest.TestCase):
    def extraction(self, ranges, points):
        return extract_production_fragments(
            np.asarray(ranges, dtype=float), np.asarray(points, dtype=float),
            angle_increment_rad=0.004, range_min_m=0.0, maximum_range_m=14.0,
            lambda_deg=10.0, cluster_sigma_m=0.03,
            minimum_two_point_distance_m=0.01, merge_enabled=True,
            merge_min_fragment_points=2, merge_distance_m=0.12,
            minimum_cluster_points=5, maximum_obstacle_diagonal_m=0.8,
        )

    def test_single_point_preserves_exact_early_rejection_reason(self):
        result = self.extraction([1.0, 30.0], [[1.0, 0.0], [30.0, 0.0]])
        self.assertEqual(len(result.accepted_clusters), 0)
        self.assertEqual(result.rejected_candidates[0].point_count, 1)
        self.assertEqual(
            result.rejected_candidates[0].production_outcome,
            "REJECTED_TEMPORARY_FRAGMENT_MIN_POINTS",
        )

    def test_four_point_candidate_reaches_final_min_cluster_rejection(self):
        ranges = [1.0, 1.0, 1.0, 1.0]
        points = [[1.0, 0.01 * index] for index in range(4)]
        result = self.extraction(ranges, points)
        self.assertEqual(len(result.accepted_clusters), 0)
        self.assertEqual(result.rejected_candidates[0].point_count, 4)
        self.assertEqual(
            result.rejected_candidates[0].production_outcome,
            "REJECTED_FINAL_MIN_CLUSTER_POINTS",
        )

    def test_existing_identity_compatibility_reuses_production_geometry_gates(self):
        existing = fragment([[1.0, 0.0], [1.0, 0.05]], beams=(1, 2))
        near = fragment([[1.02, 0.08]])
        far = fragment([[2.0, 2.0]])
        self.assertTrue(compatible_with_identity(near, [existing], distance_m=0.12, maximum_diagonal_m=0.8))
        self.assertFalse(compatible_with_identity(far, [existing], distance_m=0.12, maximum_diagonal_m=0.8))

    def test_persistence_uses_fresh_timestamps_not_scan_count(self):
        current = fragment([[1.02, 0.02]])
        prior = fragment([[1.0, 0.0]])
        self.assertTrue(has_timestamp_persistence(
            current, 20_000_000, [(10_000_000, prior)], window_ns=20_000_000,
            distance_m=0.12, maximum_diagonal_m=0.8,
        ))
        self.assertFalse(has_timestamp_persistence(
            current, 40_000_001, [(10_000_000, prior)], window_ns=20_000_000,
            distance_m=0.12, maximum_diagonal_m=0.8,
        ))

    def test_dynamic_transition_immediately_invalidates_shadow_history(self):
        state = ShadowIdentityState(7, [])
        state.observe(10, fragment([[1.0, 0.0]]))
        self.assertEqual(len(state.evidence), 1)
        state.update_motion(20, "DYNAMIC")
        self.assertFalse(state.active)
        self.assertEqual(state.evidence, [])
        state.observe(30, fragment([[1.0, 0.0]]))
        self.assertEqual(state.evidence, [])


if __name__ == "__main__":
    unittest.main()
