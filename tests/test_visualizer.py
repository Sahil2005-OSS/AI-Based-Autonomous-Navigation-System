"""
Headless unit tests for Pygame Visualizer (src/autonomous_nav/visualizer.py).

All tests run in headless mode using SDL's dummy video driver to prevent opening
physical display windows during automated test runs.
"""

import os
# Ensure dummy video driver is active before Pygame initializes in tests
os.environ["SDL_VIDEODRIVER"] = "dummy"

import unittest
import numpy as np

from autonomous_nav.world import World, Obstacle
from autonomous_nav.planner import AStarPlanner, PathResult
from autonomous_nav.visualizer import SimulationVisualizer


class TestVisualizerHeadless(unittest.TestCase):
    """Test visualizer instantiation, state stepping, and safety in headless environment."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0)  # 10x10
        self.world.add_obstacle_cell(5, 5)

        planner = AStarPlanner(self.world, connectivity=8)
        self.path_result = planner.plan((0, 0), (9, 9))
        self.assertTrue(self.path_result.success)

    def test_import_does_not_open_window(self) -> None:
        """Module import should not initialize active Pygame display."""
        # SimulationVisualizer class exists and display is not active by default
        import autonomous_nav.visualizer as vis_mod
        self.assertTrue(hasattr(vis_mod, "SimulationVisualizer"))

    def test_headless_initialization_and_cleanup(self) -> None:
        """Visualizer can be initialized and closed safely in headless mode."""
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=True,
        )
        self.assertIsNotNone(sim.screen)
        self.assertEqual(sim.current_step_index, 0)
        self.assertEqual(sim.current_position, (0, 0))

        # Render one frame headlessly
        sim.render_frame()

        # Clean shutdown
        sim.cleanup()
        self.assertIsNone(sim.screen)

    def test_waypoints_consumed_in_order(self) -> None:
        """Path waypoints must be traversed sequentially."""
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=False,
        )

        expected_path = self.path_result.path
        self.assertEqual(len(sim.path), len(expected_path))

        for idx, expected_pt in enumerate(expected_path):
            self.assertEqual(sim.current_position, expected_pt)
            if idx < len(expected_path) - 1:
                advanced = sim.step_forward()
                self.assertTrue(advanced)
            else:
                self.assertTrue(sim.is_at_goal)

    def test_advancing_at_goal_no_index_error(self) -> None:
        """Calling step_forward at goal must return False without raising IndexError."""
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=False,
        )

        # Fast forward to end of path
        while not sim.is_at_goal:
            sim.step_forward()

        self.assertTrue(sim.is_at_goal)
        self.assertEqual(sim.current_position, (9, 9))

        # Further advances must return False safely
        for _ in range(5):
            self.assertFalse(sim.step_forward())
            self.assertEqual(sim.current_position, (9, 9))

    def test_reset_returns_to_start(self) -> None:
        """Reset must restore robot marker to waypoint 0."""
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=False,
        )

        # Advance 3 steps
        sim.step_forward()
        sim.step_forward()
        sim.step_forward()
        self.assertEqual(sim.current_step_index, 3)

        sim.reset()
        self.assertEqual(sim.current_step_index, 0)
        self.assertEqual(sim.current_position, (0, 0))
        self.assertFalse(sim.auto_play)

    def test_unsuccessful_path_handled_gracefully(self) -> None:
        """An empty/failed PathResult must not cause crashes."""
        failed_result = PathResult(
            success=False,
            path=[],
            cost=float("inf"),
            expanded_nodes=42,
            message="No path",
        )
        sim = SimulationVisualizer(
            world=self.world,
            path_result=failed_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=True,
        )

        self.assertEqual(sim.path, [])
        self.assertEqual(sim.current_position, (0, 0))
        self.assertFalse(sim.is_at_goal)
        self.assertFalse(sim.step_forward())

        # Render headless frame with failed result
        sim.render_frame()
        sim.cleanup()

    def test_visualization_does_not_mutate_occupancy_grid(self) -> None:
        """Rendering must not alter world grid values."""
        grid_copy = self.world.grid.copy()

        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=True,
        )

        for _ in range(5):
            sim.step_forward()
            sim.render_frame()

        sim.cleanup()
        np.testing.assert_array_equal(self.world.grid, grid_copy)

    def test_visualization_with_smoothed_path(self) -> None:
        """Visualizer with smoothed path renders headlessly without error."""
        from autonomous_nav.path_smoother import PathSmoother
        smoother = PathSmoother(self.world)
        smooth_res = smoother.smooth_result(self.path_result)
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            smoothed_path=smooth_res.smoothed_path,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=True,
        )
        sim.render_frame()
        sim.cleanup()

    def test_continuous_motion_stepping_and_telemetry(self) -> None:
        """Continuous motion mode advances robot pose and renders telemetry cleanly."""
        sim = SimulationVisualizer(
            world=self.world,
            path_result=self.path_result,
            start_grid=(0, 0),
            goal_grid=(9, 9),
            init_display=True,
            continuous_motion=True,
            max_linear_speed=20.0,
        )
        init_x = sim.robot_state.x
        init_y = sim.robot_state.y

        # Advance 5 simulation steps
        for _ in range(5):
            sim.step_motion(dt=0.05)
            sim.render_frame()

        self.assertGreater(sim.motion_elapsed_time, 0.0)
        # Position should have advanced forward along the path
        self.assertTrue(sim.robot_state.x > init_x or sim.robot_state.y > init_y)

        # Test pause functionality
        sim.toggle_motion_pause()
        self.assertTrue(sim.motion_paused)
        paused_x = sim.robot_state.x
        sim.step_motion(dt=0.05)
        self.assertEqual(sim.robot_state.x, paused_x)

        # Test reset functionality
        sim.reset()
        self.assertEqual(sim.robot_state.x, init_x)
        self.assertEqual(sim.robot_state.y, init_y)
        self.assertEqual(sim.motion_elapsed_time, 0.0)
        self.assertFalse(sim.motion_paused)

        sim.cleanup()


if __name__ == "__main__":
    unittest.main()
