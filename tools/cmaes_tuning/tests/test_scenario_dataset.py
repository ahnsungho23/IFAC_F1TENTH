from pathlib import Path
import tempfile
import unittest

import yaml

from cmaes_tuning.scenario_audit import audit_scenario_dataset
from cmaes_tuning.scenario_generator import generate_stratified_scenario_dataset


ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = ROOT / "tools" / "cmaes_tuning" / "config" / "tuning_config.yaml"


class ScenarioDatasetTest(unittest.TestCase):
    def test_stratified_train_validation_dataset_is_valid_and_disjoint(self):
        config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary:
            config["paths"]["output_root"] = temporary
            datasets = generate_stratified_scenario_dataset(config, ROOT)
            report = audit_scenario_dataset(datasets, config)
            self.assertTrue(report["all_valid"])
            self.assertEqual(len(datasets["training"]), 25)
            self.assertEqual(len(datasets["validation"]), 40)
            self.assertEqual(report["training_validation_duplicate_count"], 0)
            self.assertEqual(
                report["training_validation_baked_hash_duplicate_count"], 0
            )
            self.assertEqual(
                set(report["distribution"]["training"]["category"]),
                set(config["dataset"]["categories"]),
            )
            self.assertEqual(
                set(report["distribution"]["validation"]["lateral"]),
                set(config["dataset"]["lateral_bands"]),
            )


if __name__ == "__main__":
    unittest.main()
