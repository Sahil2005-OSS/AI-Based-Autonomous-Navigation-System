"""
Unit tests for scenario configuration loader and schema validation.
"""

import json
import tempfile
import unittest
from pathlib import Path

from autonomous_nav.config_loader import load_config, validate_scenario_config


class TestConfigLoader(unittest.TestCase):
    """Test configuration loading and schema validation rules."""

    def setUp(self) -> None:
        self.valid_data = {
            "scenario_name": "test_arena",
            "world": {
                "width": 500,
                "height": 400,
                "grid_resolution": 10
            },
            "robot": {
                "start_position": [20.0, 20.0],
                "target_position": [450.0, 350.0]
            },
            "obstacles": [
                {
                    "id": 1,
                    "type": "circle",
                    "position": [100.0, 100.0],
                    "radius": 20.0
                }
            ]
        }

    def test_valid_config_passes_validation(self) -> None:
        """A well-formed configuration should validate without errors."""
        validate_scenario_config(self.valid_data)

    def test_missing_world_raises_value_error(self) -> None:
        """Config without 'world' block must be rejected."""
        invalid_data = {"scenario_name": "bad_arena"}
        with self.assertRaises(ValueError) as ctx:
            validate_scenario_config(invalid_data)
        self.assertIn("missing required 'world'", str(ctx.exception))

    def test_non_positive_world_dimensions_rejected(self) -> None:
        """Non-positive dimensions must raise ValueError."""
        data = dict(self.valid_data)
        data["world"] = {"width": -500, "height": 400, "grid_resolution": 10}
        with self.assertRaises(ValueError):
            validate_scenario_config(data)

        data["world"] = {"width": 500, "height": 0, "grid_resolution": 10}
        with self.assertRaises(ValueError):
            validate_scenario_config(data)

        data["world"] = {"width": 500, "height": 400, "grid_resolution": -1}
        with self.assertRaises(ValueError):
            validate_scenario_config(data)

    def test_invalid_robot_positions_rejected(self) -> None:
        """Malformed robot start or target coordinates must be rejected."""
        data = dict(self.valid_data)
        data["robot"] = {"start_position": [10.0]}  # Only 1 coordinate
        with self.assertRaises(ValueError):
            validate_scenario_config(data)

        data["robot"] = {"start_position": "not-a-list"}
        with self.assertRaises(ValueError):
            validate_scenario_config(data)

    def test_missing_file_raises_filenotfound(self) -> None:
        """Non-existent path must raise FileNotFoundError."""
        with self.assertRaises(FileNotFoundError):
            load_config("non_existent_file_xyz.json")

    def test_corrupted_json_raises_value_error(self) -> None:
        """Syntax-corrupted JSON file must raise ValueError."""
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".json") as f:
            f.write("{ bad json: ")
            tmp_path = f.name

        try:
            with self.assertRaises(ValueError):
                load_config(tmp_path)
        finally:
            Path(tmp_path).unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
