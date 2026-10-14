"""
Unit tests for Collision-Safe Path Smoother (src/autonomous_nav/path_smoother.py).
"""

import math
from pathlib import Path
import unittest
import numpy as np

from autonomous_nav.path_smoother import (
    PathSmoother,
    SmoothedPathResult,
    get_traversed_cells,
    has_line_of_sight,
    smooth_path,
)
from autonomous_nav.world import World, Obstacle
from autonomous_nav.planner import AStarPlanner, PathResult
from autonomous_nav.config_loader import load_config


class TestTraversedCellsAndLineOfSight(unittest.TestCase):
    """Test low-level cell traversal and line-of-sight checks."""

    def setUp(self) -> None:
        self.world = World(width=50.0, height=50.0, grid_resolution=10.0)  # 5x5 grid

    def test_horizontal_line_cells(self) -> None:
        """Horizontal line visits exactly the intermediate cells."""
        cells = get_traversed_cells(0, 2, 4, 2)
        expected = [(0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]
        self.assertEqual(cells, expected)

    def test_vertical_line_cells(self) -> None:
        """Vertical line visits exactly the vertical cells."""
        cells = get_traversed_cells(1, 0, 1, 3)
        expected = [(1, 0), (1, 1), (1, 2), (1, 3)]
        self.assertEqual(cells, expected)

    def test_diagonal_traversed_cells(self) -> None:
        """Diagonal segment (0, 0) -> (2, 2) visits diagonal cells."""
        cells = get_traversed_cells(0, 0, 2, 2)
        self.assertEqual(cells, [(0, 0), (1, 1), (2, 2)])

    def test_line_of_sight_blocked_by_obstacle(self) -> None:
        """Line of sight is False when direct segment passes through an obstacle."""
        self.world.add_obstacle_cell(2, 0)
        self.assertFalse(has_line_of_sight(self.world, (0, 0), (4, 0)))
        # Unblocked parallel segment
        self.assertTrue(has_line_of_sight(self.world, (0, 1), (4, 1)))

    def test_line_of_sight_blocked_by_corner(self) -> None:
        """Line of sight is False when diagonal cuts between two blocked corner obstacles."""
        self.world.add_obstacle_cell(1, 0)
        self.world.add_obstacle_cell(0, 1)
        self.assertFalse(has_line_of_sight(self.world, (0, 0), (1, 1)))


class TestPathSmootherFunctionality(unittest.TestCase):
    """Test path smoothing rules, shortcutting, and edge cases."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0)  # 10x10 grid
        self.smoother = PathSmoother(self.world)

    def test_straight_path_redundant_waypoints_reduced(self) -> None:
        """A straight path with intermediate points should collapse to [start, goal]."""
        straight_path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]
        res = self.smoother.smooth_path(straight_path)

        self.assertTrue(res.success)
        self.assertEqual(res.smoothed_path, [(0, 0), (5, 0)])
        self.assertEqual(res.original_waypoints, 6)
        self.assertEqual(res.smoothed_waypoints, 2)
        self.assertEqual(res.waypoints_removed, 4)
        self.assertAlmostEqual(res.smoothed_cost, 5.0)

    def test_open_space_detour_removed(self) -> None:
        """An unnecessary triangular detour in open space should shortcut directly."""
        detour_path = [(0, 0), (1, 1), (2, 2), (3, 1), (4, 0)]
        res = self.smoother.smooth_path(detour_path)

        self.assertTrue(res.success)
        self.assertEqual(res.smoothed_path, [(0, 0), (4, 0)])
        self.assertEqual(res.smoothed_waypoints, 2)
        self.assertAlmostEqual(res.smoothed_cost, 4.0)

    def test_path_around_obstacle_retains_safe_detour(self) -> None:
        """A path navigating around an obstacle must not shortcut through it."""
        # Wall at x=2, y in [0, 1, 2]
        self.world.add_obstacle_cell(2, 0)
        self.world.add_obstacle_cell(2, 1)
        self.world.add_obstacle_cell(2, 2)

        # Path goes around the wall through y=3
        original_path = [(0, 0), (1, 1), (1, 3), (2, 3), (3, 3), (3, 1), (4, 0)]
        res = self.smoother.smooth_path(original_path)

        self.assertTrue(res.success)
        # Verify that start and goal are retained
        self.assertEqual(res.smoothed_path[0], (0, 0))
        self.assertEqual(res.smoothed_path[-1], (4, 0))

        # Crucial check: Direct shortcut (0, 0) -> (4, 0) MUST NOT be taken
        self.assertNotIn([(0, 0), (4, 0)], [res.smoothed_path])

        # Verify every segment in smoothed path is 100% collision-free
        for i in range(len(res.smoothed_path) - 1):
            p1, p2 = res.smoothed_path[i], res.smoothed_path[i + 1]
            self.assertTrue(
                has_line_of_sight(self.world, p1, p2),
                f"Segment {p1} -> {p2} must be collision free"
            )

    def test_preservation_of_start_and_goal(self) -> None:
        """Smoothed path start and goal must exactly equal original start and goal."""
        path = [(1, 2), (2, 3), (3, 4), (4, 5)]
        res = self.smoother.smooth_path(path)
        self.assertEqual(res.smoothed_path[0], (1, 2))
        self.assertEqual(res.smoothed_path[-1], (4, 5))

    def test_waypoint_ordering_preserved(self) -> None:
        """Waypoint monotonic order from start to goal must be maintained."""
        path = [(0, 0), (2, 0), (4, 2), (6, 2), (8, 4)]
        res = self.smoother.smooth_path(path)
        x_coords = [p[0] for p in res.smoothed_path]
        self.assertEqual(x_coords, sorted(x_coords))

    def test_empty_and_one_point_paths(self) -> None:
        """Empty and single-point paths are handled gracefully."""
        empty_res = self.smoother.smooth_path([])
        self.assertFalse(empty_res.success)
        self.assertEqual(empty_res.smoothed_path, [])

        single_res = self.smoother.smooth_path([(3, 3)])
        self.assertTrue(single_res.success)
        self.assertEqual(single_res.smoothed_path, [(3, 3)])
        self.assertEqual(single_res.smoothed_waypoints, 1)

    def test_unsuccessful_astar_result(self) -> None:
        """Unsuccessful PathResult produces an unsuccessful SmoothedPathResult."""
        failed_res = PathResult(
            success=False,
            path=[],
            cost=float("inf"),
            expanded_nodes=10,
            message="No path",
        )
        smoothed = self.smoother.smooth_result(failed_res)
        self.assertFalse(smoothed.success)
        self.assertEqual(smoothed.smoothed_path, [])

    def test_invalid_and_occupied_coordinates_rejected(self) -> None:
        """Invalid coordinate types, out-of-bounds, or occupied points raise ValueError."""
        with self.assertRaises(ValueError):
            self.smoother.smooth_path(["invalid", (1, 1)])

        with self.assertRaises(ValueError):
            self.smoother.smooth_path([(0, 0), (105, 50)])

        self.world.add_obstacle_cell(5, 5)
        with self.assertRaises(ValueError):
            self.smoother.smooth_path([(0, 0), (5, 5), (9, 9)])

    def test_occupancy_grid_unmutated_after_smoothing(self) -> None:
        """Smoothing must leave the world grid completely unmodified."""
        self.world.add_obstacle_cell(3, 3)
        self.world.add_obstacle_cell(4, 4)
        grid_copy = self.world.grid.copy()

        path = [(0, 0), (1, 1), (2, 2), (2, 5), (5, 5)]
        self.smoother.smooth_path(path)

        np.testing.assert_array_equal(self.world.grid, grid_copy)


class TestDefaultScenarioSmoothing(unittest.TestCase):
    """Test smoothing on the loaded default project scenario."""

    def test_default_scenario_path_smoothing(self) -> None:
        """A* path in default scenario must smooth with reduced waypoints and 100% collision safety."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_scenario.json"
        self.assertTrue(cfg_path.is_file())

        cfg = load_config(cfg_path)
        world = World.from_config(cfg)
        planner = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True)

        # Start (5, 5), Goal (75, 55)
        astar_res = planner.plan((5, 5), (75, 55))
        self.assertTrue(astar_res.success)
        orig_count = len(astar_res.path)

        smoother = PathSmoother(world)
        smooth_res = smoother.smooth_result(astar_res)

        self.assertTrue(smooth_res.success)
        self.assertEqual(smooth_res.smoothed_path[0], (5, 5))
        self.assertEqual(smooth_res.smoothed_path[-1], (75, 55))

        # Significant waypoint reduction
        self.assertLess(smooth_res.smoothed_waypoints, orig_count)
        self.assertGreater(smooth_res.waypoints_removed, 0)

        # Every segment in smoothed path must be collision-free
        for i in range(len(smooth_res.smoothed_path) - 1):
            p1 = smooth_res.smoothed_path[i]
            p2 = smooth_res.smoothed_path[i + 1]
            self.assertTrue(
                has_line_of_sight(world, p1, p2),
                f"Smoothed segment {p1} -> {p2} failed line-of-sight safety check!"
            )


class TestConservativeCornerCrossingSafety(unittest.TestCase):
    """
    Regression tests for conservative grid-corner collision checking (Task 05.1).
    Ensures diagonal line-of-sight is rejected if EITHER adjacent orthogonal cell
    sharing an exact grid-vertex crossing is occupied or out of bounds.
    """

    def setUp(self) -> None:
        # Small 3x3 grid world: (0, 0) to (2, 2)
        self.world = World(width=30.0, height=30.0, grid_resolution=10.0)

    def test_diagonal_crossing_both_adjacent_free_accepted(self) -> None:
        """Case 1: Diagonal crossing where both adjacent orthogonal cells are free."""
        # Diagonal from (0, 0) to (1, 1). Vertex crossed is (1.0, 1.0).
        # Adjacent orthogonal cells (1, 0) and (0, 1) are both free.
        self.assertTrue(
            has_line_of_sight(self.world, (0, 0), (1, 1)),
            "Diagonal traversal must be accepted when both adjacent cells are free."
        )

    def test_diagonal_crossing_first_adjacent_occupied_rejected(self) -> None:
        """Case 2: Diagonal crossing where the first adjacent cell (gx+sx, gy) is occupied and second is free."""
        # Segment from (0, 0) to (1, 1): adj_x = (1, 0), adj_y = (0, 1).
        self.world.add_obstacle_cell(1, 0)  # First adjacent cell occupied
        self.assertFalse(
            has_line_of_sight(self.world, (0, 0), (1, 1)),
            "Diagonal crossing must be rejected if the first adjacent cell is occupied."
        )

    def test_diagonal_crossing_second_adjacent_occupied_rejected(self) -> None:
        """Case 3: Diagonal crossing where the second adjacent cell (gx, gy+sy) is occupied and first is free."""
        # Segment from (0, 0) to (1, 1): adj_x = (1, 0), adj_y = (0, 1).
        self.world.add_obstacle_cell(0, 1)  # Second adjacent cell occupied
        self.assertFalse(
            has_line_of_sight(self.world, (0, 0), (1, 1)),
            "Diagonal crossing must be rejected if the second adjacent cell is occupied."
        )

    def test_diagonal_crossing_both_adjacent_occupied_rejected(self) -> None:
        """Case 4: Diagonal crossing where both adjacent cells are occupied."""
        # Segment from (0, 0) to (1, 1): adj_x = (1, 0), adj_y = (0, 1).
        self.world.add_obstacle_cell(1, 0)
        self.world.add_obstacle_cell(0, 1)
        self.assertFalse(
            has_line_of_sight(self.world, (0, 0), (1, 1)),
            "Diagonal crossing must be rejected when both adjacent cells are occupied."
        )

    def test_safe_route_around_blocked_corner_retained_after_smoothing(self) -> None:
        """Case 5: A safe route around a blocked corner remains valid and intact after smoothing."""
        # Place obstacle cell at (1, 1).
        self.world.add_obstacle_cell(1, 1)
        # Safe L-shaped detour route from (0, 1) to (1, 0) stepping through (0, 0):
        route = [(0, 1), (0, 0), (1, 0)]

        # Direct shortcut (0, 1) -> (1, 0) crosses vertex (1.0, 1.0) where (1, 1) is occupied.
        # It must be rejected.
        self.assertFalse(
            has_line_of_sight(self.world, (0, 1), (1, 0)),
            "Direct diagonal shortcut through the blocked corner must be rejected."
        )

        smoother = PathSmoother(self.world)
        smooth_res = smoother.smooth_path(route)

        self.assertTrue(smooth_res.success)
        self.assertEqual(smooth_res.smoothed_path[0], (0, 1))
        self.assertEqual(smooth_res.smoothed_path[-1], (1, 0))
        # The safe detour route must NOT be shortcut through the corner
        self.assertEqual(
            smooth_res.smoothed_path,
            [(0, 1), (0, 0), (1, 0)],
            "Smoothed path must preserve the safe detour around the blocked corner."
        )
        self.assertEqual(smooth_res.smoothed_waypoints, 3)

        # Verify every segment in the smoothed route satisfies the conservative line-of-sight check
        for i in range(len(smooth_res.smoothed_path) - 1):
            p1 = smooth_res.smoothed_path[i]
            p2 = smooth_res.smoothed_path[i + 1]
            self.assertTrue(
                has_line_of_sight(self.world, p1, p2),
                f"Segment {p1} -> {p2} must pass line-of-sight check."
            )


if __name__ == "__main__":
    unittest.main()
