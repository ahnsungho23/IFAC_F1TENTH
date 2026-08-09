from pathlib import Path
import json
import tempfile
import unittest

import yaml

from cmaes_tuning.scenario_generator import generate_fixed_scenarios
from cmaes_tuning.schemas import sha256_file


ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = ROOT / "tools" / "cmaes_tuning" / "config" / "tuning_config.yaml"


class ScenarioGeneratorTest(unittest.TestCase):
    def test_fixed_manifests_are_replayable(self):
        config = yaml.safe_load(CONFIG_PATH.read_text())
        with tempfile.TemporaryDirectory() as temporary:
            config["paths"]["output_root"] = temporary
            first = generate_fixed_scenarios(config, ROOT)
            first_payloads = [json.loads(path.read_text()) for path in first]
            first_hashes = [sha256_file(path) for path in first]
            second = generate_fixed_scenarios(config, ROOT)
            second_payloads = [json.loads(path.read_text()) for path in second]
            second_hashes = [sha256_file(path) for path in second]
            self.assertEqual(first_hashes, second_hashes)
            self.assertEqual(first_payloads, second_payloads)
            self.assertEqual([item["scenario_id"] for item in first_payloads], [
                "train_000", "train_001", "train_002"
            ])
            for payload in first_payloads:
                self.assertEqual(payload["schema"], "cmaes_scenario/1")
                self.assertEqual(payload["obstacle"]["shape"], "rect")
                self.assertEqual(sha256_file(payload["baked_map_image"]), payload["baked_map_image_sha256"])


if __name__ == "__main__":
    unittest.main()
