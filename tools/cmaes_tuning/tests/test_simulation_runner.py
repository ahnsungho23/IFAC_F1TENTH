import os
from pathlib import Path
import tempfile
import time
import unittest

from cmaes_tuning.simulation_runner import EpisodeRunner, ProcessSupervisor


class FakeResult:
    def __init__(self, stdout: str, returncode: int = 0):
        self.stdout = stdout
        self.stderr = ""
        self.returncode = returncode


class FakeSupervisor:
    def __init__(self, topic_info: str, node_list: str):
        self.topic_info = topic_info
        self.node_list = node_list

    def run(self, command, timeout):
        del timeout
        if command[1:3] == ["topic", "info"]:
            return FakeResult(self.topic_info)
        if command[1:3] == ["node", "list"]:
            return FakeResult(self.node_list)
        raise AssertionError(command)


class ProcessSupervisorTest(unittest.TestCase):
    def test_stop_all_removes_children_after_launch_parent_exits(self):
        with tempfile.TemporaryDirectory(prefix="cma_group_cleanup_", dir="/tmp") as temp:
            root = Path(temp)
            environment = dict(os.environ)
            environment["CMA_WORKSPACE_ROOT"] = str(root)
            supervisor = ProcessSupervisor(root, environment, "true")
            script = (
                "import signal,subprocess,sys,time;"
                "subprocess.Popen(['python3','-c',"
                "'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN);"
                " time.sleep(60)']);"
                "signal.signal(signal.SIGINT, lambda *_: sys.exit(0));"
                "time.sleep(60)"
            )
            managed = [
                supervisor.start(f"group_{index}", ["python3", "-c", script])
                for index in range(3)
            ]
            try:
                time.sleep(0.25)
                failures = supervisor.stop_all(0.25, 0.25)
                self.assertEqual([], failures)
                for process in managed:
                    with self.assertRaises(ProcessLookupError):
                        os.killpg(process.process.pid, 0)
            finally:
                supervisor.stop_all(0.1, 0.1)


class LocalizationGraphAuditTest(unittest.TestCase):
    def test_accepts_endpoint_identity_when_node_list_discovery_omits_bridge(self):
        config = {
            "experiment": {"localization_mode": "ground_truth"},
            "runner": {"topic_probe_timeout_sec": 1.0},
        }
        runner = EpisodeRunner(config, "/tmp")
        topic_info = """Type: nav_msgs/msg/Odometry

Publisher count: 1

Node name: gt_localization_bridge
Node namespace: /

Subscription count: 1

Node name: frenet_odom_node
Node namespace: /
"""
        supervisor = FakeSupervisor(topic_info, "/bridge\n/frenet_odom_node\n")
        status = {}
        self.assertEqual([], runner._audit_localization_graph(supervisor, status))
        self.assertEqual(
            ["gt_localization_bridge"],
            status["localization_graph_audit"]["publisher_node_names"],
        )

    def test_rejects_wrong_provider_for_ground_truth_mode(self):
        config = {
            "experiment": {"localization_mode": "ground_truth"},
            "runner": {"topic_probe_timeout_sec": 1.0},
        }
        runner = EpisodeRunner(config, "/tmp")
        topic_info = """Publisher count: 1
Node name: particle_filter
Subscription count: 1
Node name: frenet_odom_node
"""
        failures = runner._audit_localization_graph(
            FakeSupervisor(topic_info, "/particle_filter\n"), {}
        )
        self.assertIn("gt_localization_bridge_missing", failures)
        self.assertIn("particle_filter_present_in_ground_truth_mode", failures)


if __name__ == "__main__":
    unittest.main()
