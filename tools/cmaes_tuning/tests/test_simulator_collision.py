import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from PIL import Image, ImageDraw
import yaml

from cmaes_tuning.geometry import footprint_obstacle_clearance
from cmaes_tuning.map_baker import MapModel
from cmaes_tuning.schemas import ObstacleSpec
from cmaes_tuning.simulator_collision import (
    SimulatorRasterCollisionModel,
    repair_collision_yaw_reset,
)


FIXTURE = Path(__file__).parent / "fixtures" / "train_000_collision_mismatch.json"


def model_parameters(*, noise_std=0.0, noise_sigma=0.0, ttc=0.0):
    return {
        "vehicle_length_m": 0.56,
        "vehicle_width_m": 0.287,
        "occupied_gray_threshold": 128,
        "lidar_offset_x_m": 0.275,
        "scan_beams": 1080,
        "scan_fov_rad": 4.7,
        "ttc_threshold_sec": ttc,
        "scan_noise_std_m": noise_std,
        "scan_noise_guard_sigma": noise_sigma,
    }


def write_map(directory, *, origin=(-1.0, -1.0, 0.0), resolution=0.01, wall=True):
    directory = Path(directory)
    image = Image.new("L", (200, 200), 255)
    if wall:
        draw = ImageDraw.Draw(image)
        # Pixel column 120 is the half-open simulator cell x=[0.20, 0.21).
        draw.rectangle((120, 0, 120, 199), fill=0)
    image_path = directory / "map.png"
    yaml_path = directory / "map.yaml"
    image.save(image_path)
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": image_path.name,
                "resolution": resolution,
                "origin": list(origin),
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return yaml_path


class SimulatorCollisionTest(unittest.TestCase):
    def test_collision_frame_yaw_and_speed_are_extrapolated(self):
        samples = [
            {
                "timestamp_ns": 1,
                "x": 0.00,
                "y": 0.0,
                "yaw": 0.10,
                "speed": 1.0,
                "yaw_rate": 2.0,
            },
            {
                "timestamp_ns": 2,
                "x": 0.01,
                "y": 0.0,
                "yaw": 0.12,
                "speed": 0.9,
                "yaw_rate": 2.0,
            },
            {
                "timestamp_ns": 3,
                "x": 0.02,
                "y": 0.0,
                "yaw": 0.0,
                "speed": 0.0,
                "yaw_rate": 0.0,
            },
        ]
        repaired, diagnostic = repair_collision_yaw_reset(
            samples,
            {
                "yaw_reset_tolerance_rad": 1.0e-6,
                "minimum_pre_reset_speed_mps": 0.4,
                "maximum_reset_step_m": 0.15,
                "physics_timestep_sec": 0.01,
            },
        )
        self.assertTrue(diagnostic["detected"])
        self.assertAlmostEqual(repaired[-1]["yaw"], 0.14)
        self.assertAlmostEqual(repaired[-1]["collision_speed"], 0.8)

    def test_clear_separation_is_not_collision(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary), model_parameters()
            )
            self.assertFalse(model.evaluate_pose(-0.20, 0.0, 0.0, 0.0)["collision"])

    def test_tangent_and_near_boundary_convention(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary), model_parameters()
            )
            self.assertTrue(model.evaluate_pose(-0.08, 0.0, 0.0, 0.0)["collision"])
            self.assertFalse(model.evaluate_pose(-0.081, 0.0, 0.0, 0.0)["collision"])

    def test_slight_overlap_is_collision(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary), model_parameters()
            )
            self.assertTrue(model.evaluate_pose(-0.079, 0.0, 0.0, 0.0)["collision"])

    def test_rotated_vehicle_uses_oriented_footprint(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary), model_parameters()
            )
            tangent_center_x = 0.20 - 0.5 * 0.287
            result = model.evaluate_pose(tangent_center_x, 0.0, math.pi / 2.0, 0.0)
            self.assertTrue(result["collision"])

    def test_rotated_obstacle_baking_is_seen_in_raster(self):
        with tempfile.TemporaryDirectory() as temporary:
            clean_yaml = write_map(temporary, wall=False)
            map_model = MapModel(clean_yaml)
            obstacle = ObstacleSpec(
                shape="rect",
                x=0.0,
                y=0.0,
                s=0.0,
                d=0.0,
                yaw=math.pi / 4.0,
                width=0.40,
                height=0.20,
            )
            baked = map_model.write_baked(Path(temporary) / "baked", "rotated", [obstacle])
            model = SimulatorRasterCollisionModel(
                baked["yaml"], model_parameters()
            )
            self.assertTrue(model.evaluate_pose(0.0, 0.0, -math.pi / 6.0, 0.0)["collision"])

    def test_raster_map_boundary_is_collision(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary, wall=False), model_parameters()
            )
            result = model.evaluate_pose(-0.90, 0.0, 0.0, 0.0)
            self.assertTrue(result["collision"])
            self.assertEqual(result["cause"], "map_boundary")

    def test_recorded_scan_reapplies_exact_simulator_ttc(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = SimulatorRasterCollisionModel(
                write_map(temporary, wall=False), model_parameters(ttc=0.005)
            )
            beam = model.scan_beams // 2
            ranges = [30.0] * model.scan_beams
            ranges[beam] = float(model.side_distances[beam] + 0.002)
            scans = [
                SimpleNamespace(
                    timestamp_ns=2,
                    message=SimpleNamespace(ranges=ranges),
                )
            ]
            samples = [
                {
                    "timestamp_ns": 1,
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "speed": 1.0,
                    "yaw_rate": 0.0,
                }
            ]
            result = model.evaluate_recorded_scans(scans, samples)
            self.assertTrue(result["collision"])
            self.assertEqual(result["cause"], "recorded_lidar_ttc")
            self.assertEqual(result["pose_diagnostics"]["beam_index"], beam)

    def test_minimal_recorded_yaw_reset_mismatch(self):
        fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary:
            map_config = fixture["map"]
            clean_yaml = write_map(
                temporary,
                wall=False,
                origin=tuple(map_config["origin"]),
                resolution=float(map_config["resolution"]),
            )
            map_model = MapModel(clean_yaml)
            obstacle = ObstacleSpec(**fixture["obstacle"])
            baked = map_model.write_baked(Path(temporary) / "baked", "mismatch", [obstacle])
            published = fixture["published_collision_frame"]
            recorded_clearance, recorded_overlap = footprint_obstacle_clearance(
                published["x"],
                published["y"],
                published["yaw"],
                fixture["vehicle"]["length"],
                fixture["vehicle"]["width"],
                fixture["obstacle"],
            )
            self.assertFalse(recorded_overlap)
            self.assertAlmostEqual(recorded_clearance, 0.020329260275138594)

            repaired, diagnostic = repair_collision_yaw_reset(
                [fixture["pre_collision_frame"], published],
                {
                    "yaw_reset_tolerance_rad": 1.0e-6,
                    "minimum_pre_reset_speed_mps": 0.4,
                    "maximum_reset_step_m": 0.15,
                },
            )
            self.assertTrue(diagnostic["detected"])
            model = SimulatorRasterCollisionModel(
                baked["yaml"], model_parameters(noise_std=0.01, noise_sigma=3.0, ttc=0.005)
            )
            collision_frame = repaired[-1]
            result = model.evaluate_pose(
                collision_frame["x"],
                collision_frame["y"],
                collision_frame["yaw"],
                collision_frame["collision_speed"],
            )
            self.assertTrue(result["collision"])


if __name__ == "__main__":
    unittest.main()
