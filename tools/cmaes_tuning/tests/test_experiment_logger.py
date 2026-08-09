import csv
from pathlib import Path
import tempfile
import unittest

from cmaes_tuning.experiment_logger import CSV_FIELDS, append_result_csv


class ExperimentLoggerTest(unittest.TestCase):
    def test_candidate_scenario_row_is_upserted(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "results.csv"
            first = {name: "" for name in CSV_FIELDS}
            first.update(
                {
                    "candidate_id": "candidate_000",
                    "scenario_id": "train_000",
                    "fitness": 4.0,
                }
            )
            second = dict(first)
            second["fitness"] = 2.0
            append_result_csv(path, first)
            append_result_csv(path, second)

            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["fitness"], "2.0")


if __name__ == "__main__":
    unittest.main()
