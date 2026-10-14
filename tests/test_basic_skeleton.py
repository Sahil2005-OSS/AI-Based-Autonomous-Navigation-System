"""
Initial unit tests for project skeleton and configuration validation.
"""

import unittest
from pathlib import Path

from autonomous_nav.config_loader import load_config
from autonomous_nav.world import World


class TestProjectSkeleton(unittest.TestCase):
    """Validate initial configuration loading and world instantiation."""

    def setUp(self) -> None:
        self.config_path = (
            Path(__file__).resolve().parents[1] / "config" / "default_scenario.json"
        )

    def test_default_config_exists_and_loads(self) -> None:
        """Verify default_scenario.json exists and loads valid dictionary."""
        self.assertTrue(self.config_path.is_file(), "default_scenario.json should exist")
        data = load_config(self.config_path)
        self.assertIsInstance(data, dict)
        self.assertIn("world", data)
        self.assertIn("robot", data)
        self.assertIn("obstacles", data)

    def test_world_instantiation_from_config(self) -> None:
        """Verify World object can be initialized from loaded config."""
        data = load_config(self.config_path)
        world = World.from_config(data)
        self.assertEqual(world.width, 800.0)
        self.assertEqual(world.height, 600.0)
        self.assertEqual(len(world.obstacles), 3)


if __name__ == "__main__":
    unittest.main()
