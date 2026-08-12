from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest

from cmaes_tuning.bag_reader import BagData, MessageRecord
from cmaes_tuning.noise_diagnostics import (
    chain_timing_events,
    _detector_semantics_diagnostics,
    _localization_diagnostics,
    _planner_timing,
    _ranges_sha256,
    _scan_identity_diagnostics,
    _tf_consistency_diagnostics,
    planner_log_diagnostics,
)


def stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def quaternion(yaw=0.0):
    import math
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def odom(ns, x, yaw=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns), frame_id="map"),
        child_frame_id="ego_racecar/base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0.0),
                orientation=quaternion(yaw),
            )
        ),
    )


class LocalizationDiagnosticsTest(unittest.TestCase):
    def test_monotonic_timing_chain_matches_forwarded_command(self):
        events = [
            {"event": "T0_STATIC_OBS", "steady_time_ns": 1_000_000_000},
            {"event": "T1_AVOID_WAYPOINTS", "steady_time_ns": 1_010_000_000},
            {"event": "T2_STATE_CONFIRMATION", "steady_time_ns": 1_020_000_000},
            {"event": "T3_STATE_TRANSITION", "steady_time_ns": 1_080_000_000},
            {"event": "T4_LOCAL_WAYPOINTS", "steady_time_ns": 1_084_000_000},
            {"event": "T5_CONTROL_CONSUME", "steady_time_ns": 1_090_000_000},
            {
                "event": "T6_DRIVE_AUTONOMOUS",
                "steady_time_ns": 1_092_000_000,
                "drive_stamp_ns": 600,
            },
            {
                "event": "T7_DRIVE",
                "steady_time_ns": 1_093_000_000,
                "input_drive_stamp_ns": 600,
                "drive_stamp_ns": 700,
            },
            {
                "event": "T8_SIM_RECEIVE",
                "steady_time_ns": 1_094_000_000,
                "drive_stamp_ns": 700,
                "drive_receive_sequence": 8,
            },
            {
                "event": "T9_PHYSICS_APPLY",
                "steady_time_ns": 1_100_000_000,
                "drive_receive_sequence": 8,
            },
            {
                "event": "T9_PHYSICS_APPLY",
                "steady_time_ns": 1_101_000_000,
                "drive_receive_sequence": 9,
            },
        ]
        result = chain_timing_events(events)
        self.assertTrue(result["complete_chain"])
        self.assertTrue(result["monotonic_order_valid"])
        self.assertAlmostEqual(result["latencies_ms"]["t3_minus_t2"], 60.0)
        self.assertAlmostEqual(result["latencies_ms"]["t9_minus_t1"], 90.0)
        self.assertEqual(result["events"]["T9"]["drive_receive_sequence"], 8)

    def test_scan_identity_proves_duplicate_backend_publication(self):
        scans = []
        identities = []
        for index, ns in enumerate((1_000_000_000, 1_004_000_000, 1_008_000_000)):
            scan = SimpleNamespace(
                header=SimpleNamespace(stamp=stamp(ns)), ranges=[1.0, 2.0]
            )
            scans.append(MessageRecord(timestamp_ns=ns + 10, message=scan))
            payload = {
                "schema": "f1tenth_scan_identity/1",
                "publish_sequence": index,
                "reset_index": 0,
                "backend_scan_index": 4,
                "backend_generation_timestamp_ns": 999_000_000,
                "publish_timestamp_ns": ns,
                "ranges_sha256": _ranges_sha256(scan),
                "fresh_backend_scan": index == 0,
                "publication_mode": "legacy_republish",
                "simulator_seed": 12345,
                "scan_noise_std": 0.01,
            }
            identities.append(
                MessageRecord(
                    timestamp_ns=ns + 20,
                    message=SimpleNamespace(data=__import__("json").dumps(payload)),
                )
            )
        result, by_header = _scan_identity_diagnostics(
            scans, identities, 1_008_000_000
        )
        self.assertEqual(result["unique_backend_scan_count"], 1)
        self.assertEqual(result["maximum_duplicate_publication_count"], 3)
        self.assertEqual(result["consecutive_duplicate_with_new_timestamp_count"], 2)
        self.assertEqual(result["ranges_hash_mismatch_count"], 0)
        self.assertEqual(result["unique_backend_scans_before_commitment"], 1)

        static_records = []
        for index, ns in enumerate((1_000_000_000, 1_004_000_000, 1_008_000_000)):
            static_records.append(
                MessageRecord(
                    timestamp_ns=ns + 30,
                    message=SimpleNamespace(
                        header=SimpleNamespace(stamp=stamp(ns)),
                        obstacles=[] if index < 2 else [SimpleNamespace(id=7)],
                    ),
                )
            )
        detector = _detector_semantics_diagnostics(
            static_records, by_header, 1_008_000_000
        )
        self.assertTrue(
            detector["existence_confirmation_duplicate_contribution"]
        )
        self.assertTrue(detector["envelope_stability_duplicate_contribution"])

    def test_exact_bridge_has_zero_error_and_no_discontinuity(self):
        ground_truth = [
            {"timestamp_ns": ns, "header_timestamp_ns": ns, "x": x, "y": 0.0, "yaw": yaw}
            for ns, x, yaw in ((1_000_000_000, 0.0, 0.0), (1_010_000_000, 0.04, 0.01))
        ]
        records = [
            MessageRecord(timestamp_ns=ns + 100, message=odom(ns, x, yaw))
            for ns, x, yaw in ((1_000_000_000, 0.0, 0.0), (1_010_000_000, 0.04, 0.01))
        ]
        result = _localization_diagnostics(ground_truth, records)
        self.assertEqual(result["position_error_m"]["max"], 0.0)
        self.assertEqual(result["yaw_error_rad"]["max"], 0.0)
        self.assertEqual(result["exact_source_timestamp_match_fraction"], 1.0)
        self.assertFalse(result["bridge_introduced_discontinuity"])

    def test_planner_timing_separates_detector_and_path_latency(self):
        detection_ns = 2_000_000_000
        scan_ns = detection_ns - 3_000_000
        path_ns = detection_ns + 25_000_000
        sensor_stamp = 1_500_000_000
        obstacle = SimpleNamespace(
            id=7,
            has_cartesian=True,
            is_static=False,
            is_visible=True,
            x_center=2.1,
            y_center=1.0,
            radius=0.35,
            s_center=5.0,
            d_center=0.1,
            s_start=4.7,
            s_end=5.3,
            d_right=-0.2,
            d_left=0.3,
        )
        scan = SimpleNamespace(header=SimpleNamespace(stamp=stamp(sensor_stamp)))
        static = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(sensor_stamp)),
            obstacles=[obstacle],
        )
        path = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(sensor_stamp + 24_000_000)),
            wpnts=[
                SimpleNamespace(d_m=0.0),
                SimpleNamespace(d_m=0.4),
            ],
        )
        bag = BagData(
            uri=SimpleNamespace(),
            topic_types={"/scan": "scan", "/static_obs": "obstacles"},
            records={
                "/scan": [MessageRecord(scan_ns, scan)],
                "/static_obs": [MessageRecord(detection_ns, static)],
                "/avoid_waypoints": [MessageRecord(path_ns, path)],
            },
        )
        ground_truth = [
            {
                "header_timestamp_ns": sensor_stamp,
                "timestamp_ns": sensor_stamp,
                "x": 1.0,
                "y": 1.0,
                "yaw": 0.0,
            }
        ]
        result = _planner_timing(
            bag,
            {"obstacle": {"x": 2.0, "y": 1.0}},
            ground_truth,
        )
        self.assertAlmostEqual(result["first_detection_scan_to_static_obs_ms"], 3.0)
        self.assertAlmostEqual(result["detection_to_path_ms"], 25.0)
        self.assertAlmostEqual(
            result["first_detection"]["center_error_to_manifest_m"], 0.1
        )
        self.assertAlmostEqual(result["first_path"]["maximum_abs_d_m"], 0.4)

    def test_tf_edge_matches_pose_and_detects_no_duplicate(self):
        ns = 2_000_000_000
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(ns), frame_id="map"),
            child_frame_id="ego_racecar/base_link",
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=0.0),
                rotation=quaternion(0.2),
            ),
        )
        tf_record = MessageRecord(
            timestamp_ns=ns + 200,
            message=SimpleNamespace(transforms=[transform]),
        )
        pose_record = MessageRecord(timestamp_ns=ns + 100, message=odom(ns, 1.0, 0.2))
        result = _tf_consistency_diagnostics([tf_record], [pose_record])
        self.assertEqual(result["duplicate_edge_sample_count"], 0)
        self.assertEqual(result["position_error_m"]["max"], 0.0)
        self.assertEqual(result["yaw_error_rad"]["max"], 0.0)

    def test_planner_log_extracts_commit_branch_and_replan(self):
        with tempfile.TemporaryDirectory(prefix="planner_log_", dir="/tmp") as directory:
            path = Path(directory) / "local_planning.log"
            path.write_text(
                "[node] [INFO] [123.004000005] [planner]: Committed left d-offset "
                "spline around static obstacle 0 (target d=0.22).\n"
                "Hard commitment collision; replanning immediately: obstacle_id=0 waypoint[1]\n"
                "Replaced commitment with left d-offset spline around static obstacle 0 "
                "(target d=0.57).\n"
                "Static safe-stop latched: no collision-free stop prefix\n",
                encoding="utf-8",
            )
            result = planner_log_diagnostics(path)
            self.assertEqual(result["initial_commit_side"], "left")
            self.assertAlmostEqual(result["initial_commit_target_d_m"], 0.22)
            self.assertEqual(result["initial_commit_timestamp_ns"], 123_004_000_005)
            self.assertEqual(result["replacement_count"], 1)
            self.assertEqual(result["hard_collision_obstacle_ids"], [0])
            self.assertEqual(result["safe_stop_latch_count"], 1)


if __name__ == "__main__":
    unittest.main()
