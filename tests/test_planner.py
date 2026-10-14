"""
Unit tests for A* Path Planner (src/autonomous_nav/planner.py).
"""

import math
import unittest
import numpy as np

from autonomous_nav.planner import AStarPlanner, PathResult, find_path
from autonomous_nav.world import World, Obstacle, CELL_FREE, CELL_OCCUPIED


class TestAStarPlannerBasics(unittest.TestCase):
    """Test basic A* initialization, connectivity modes, and heuristics."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0)  # 10x10 grid

    def test_default_initialization(self) -> None:
        """4-connected should default to manhattan heuristic."""
        planner = AStarPlanner(self.world, connectivity=4)
        self.assertEqual(planner.connectivity, 4)
        self.assertEqual(planner.heuristic_name, "manhattan")

    def test_eight_connected_default_heuristic(self) -> None:
        """8-connected should default to euclidean heuristic."""
        planner = AStarPlanner(self.world, connectivity=8)
        self.assertEqual(planner.connectivity, 8)
        self.assertEqual(planner.heuristic_name, "euclidean")

    def test_inadmissible_heuristic_rejected(self) -> None:
        """Manhattan distance is inadmissible for 8-connected grids and must be rejected."""
        with self.assertRaises(ValueError) as ctx:
            AStarPlanner(self.world, connectivity=8, heuristic="manhattan")
        self.assertIn("inadmissible", str(ctx.exception).lower())

    def test_invalid_connectivity_rejected(self) -> None:
        """Non-4/8 connectivity values must raise ValueError."""
        with self.assertRaises(ValueError):
            AStarPlanner(self.world, connectivity=6)

    def test_invalid_heuristic_rejected(self) -> None:
        """Unknown heuristics must raise ValueError."""
        with self.assertRaises(ValueError):
            AStarPlanner(self.world, connectivity=4, heuristic="chebyshev")


class TestAStarEndpointsAndValidation(unittest.TestCase):
    """Test start and goal validation rules."""

    def setUp(self) -> None:
        self.world = World(width=50.0, height=50.0, grid_resolution=10.0)  # 5x5 grid
        self.world.add_obstacle_cell(2, 2)
        self.planner = AStarPlanner(self.world, connectivity=4)

    def test_out_of_bounds_start_rejected(self) -> None:
        """Out of bounds start must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.planner.plan((-1, 0), (4, 4))
        self.assertIn("out of bounds", str(ctx.exception).lower())

    def test_out_of_bounds_goal_rejected(self) -> None:
        """Out of bounds goal must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.planner.plan((0, 0), (5, 5))
        self.assertIn("out of bounds", str(ctx.exception).lower())

    def test_occupied_start_rejected(self) -> None:
        """Occupied start cell must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.planner.plan((2, 2), (4, 4))
        self.assertIn("occupied", str(ctx.exception).lower())

    def test_occupied_goal_rejected(self) -> None:
        """Occupied goal cell must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.planner.plan((0, 0), (2, 2))
        self.assertIn("occupied", str(ctx.exception).lower())

    def test_start_equals_goal_rejected(self) -> None:
        """Identical start and goal positions must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.planner.plan((1, 1), (1, 1))
        self.assertIn("identical", str(ctx.exception).lower())


class TestAStarPathfinding(unittest.TestCase):
    """Test path computation, costs, and optimality on small maps."""

    def test_open_grid_straight_line(self) -> None:
        """Straight-line path on 5x5 empty grid."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        planner = AStarPlanner(world, connectivity=4)
        result = planner.plan((0, 0), (4, 0))

        self.assertTrue(result.success)
        self.assertEqual(result.path, [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)])
        self.assertAlmostEqual(result.cost, 4.0)
        self.assertGreater(result.expanded_nodes, 0)

    def test_four_connected_diagonal_target(self) -> None:
        """In 4-connected, diagonal target (3, 3) from (0, 0) requires 6 steps, cost 6.0."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        planner = AStarPlanner(world, connectivity=4)
        result = planner.plan((0, 0), (3, 3))

        self.assertTrue(result.success)
        self.assertEqual(len(result.path), 7)  # 6 steps + start
        self.assertAlmostEqual(result.cost, 6.0)

        # Verify only orthogonal steps
        for i in range(len(result.path) - 1):
            p1, p2 = result.path[i], result.path[i + 1]
            dx = abs(p1[0] - p2[0])
            dy = abs(p1[1] - p2[1])
            self.assertEqual(dx + dy, 1, "4-connected moves must be strictly orthogonal")

    def test_eight_connected_diagonal_target(self) -> None:
        """In 8-connected, target (3, 3) from (0, 0) should use 3 diagonal steps, cost 3*sqrt(2)."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        planner = AStarPlanner(world, connectivity=8)
        result = planner.plan((0, 0), (3, 3))

        self.assertTrue(result.success)
        self.assertEqual(result.path, [(0, 0), (1, 1), (2, 2), (3, 3)])
        expected_cost = 3.0 * math.sqrt(2.0)
        self.assertAlmostEqual(result.cost, expected_cost, places=6)

    def test_path_around_single_obstacle(self) -> None:
        """Route navigating around a wall obstacle."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        # Place a vertical wall at x=2 from y=0 to y=3
        for y in range(4):
            world.add_obstacle_cell(2, y)

        planner = AStarPlanner(world, connectivity=4)
        result = planner.plan((0, 0), (4, 0))

        self.assertTrue(result.success)
        # Must pass through the gap at y=4
        y_coords = [p[1] for p in result.path]
        self.assertIn(4, y_coords)
        # Verify all cells in path are traversable
        for p in result.path:
            self.assertTrue(world.is_traversable(p[0], p[1]))

    def test_unreachable_goal_returns_failure(self) -> None:
        """Completely blocked goal returns failure with inf cost and empty path."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        # Surround (4, 4) with walls
        world.add_obstacle_cell(3, 4)
        world.add_obstacle_cell(4, 3)
        world.add_obstacle_cell(3, 3)

        planner = AStarPlanner(world, connectivity=4)
        result = planner.plan((0, 0), (4, 4))

        self.assertFalse(result.success)
        self.assertEqual(result.path, [])
        self.assertEqual(result.cost, float("inf"))
        self.assertGreater(result.expanded_nodes, 0)
        self.assertIn("no traversable path", result.message.lower())

    def test_diagonal_corner_cutting_prevention(self) -> None:
        """
        Diagonal move must not pass between two blocked adjacent cells.
        Scenario:
            S = (0, 0), G = (1, 1)
            Obstacles at (1, 0) and (0, 1)
        Passing between them is forbidden. Since grid is 2x2, no path exists.
        """
        world = World(width=20.0, height=20.0, grid_resolution=10.0)  # 2x2 grid
        world.add_obstacle_cell(1, 0)
        world.add_obstacle_cell(0, 1)

        planner = AStarPlanner(world, connectivity=8)
        result = planner.plan((0, 0), (1, 1))

        self.assertFalse(result.success)
        self.assertEqual(result.path, [])

    def test_strict_diagonal_clearance(self) -> None:
        """
        When strict_diagonal_clearance is True, diagonal move is forbidden
        if either adjacent cell is blocked.
        """
        world = World(width=30.0, height=30.0, grid_resolution=10.0)  # 3x3 grid
        world.add_obstacle_cell(1, 0)  # Only 1 adjacent cell blocked

        # Standard mode: move (0, 0) -> (1, 1) allowed since other adjacent (0, 1) is free
        planner_standard = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=False)
        res_standard = planner_standard.plan((0, 0), (1, 1))
        self.assertTrue(res_standard.success)
        self.assertEqual(res_standard.path, [(0, 0), (1, 1)])

        # Strict mode: move (0, 0) -> (1, 1) forbidden; must detour via (0, 1)
        planner_strict = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True)
        res_strict = planner_strict.plan((0, 0), (1, 1))
        self.assertTrue(res_strict.success)
        self.assertEqual(res_strict.path, [(0, 0), (0, 1), (1, 1)])
        self.assertAlmostEqual(res_strict.cost, 2.0)

    def test_deterministic_results_on_repeated_runs(self) -> None:
        """Repeated planning runs on identical setups must produce identical paths."""
        world = World(width=60.0, height=60.0, grid_resolution=10.0)
        world.add_obstacle_cell(2, 2)
        world.add_obstacle_cell(3, 2)
        world.add_obstacle_cell(2, 3)

        planner = AStarPlanner(world, connectivity=8)
        run1 = planner.plan((0, 0), (5, 5))
        run2 = planner.plan((0, 0), (5, 5))

        self.assertEqual(run1.path, run2.path)
        self.assertEqual(run1.cost, run2.cost)
        self.assertEqual(run1.expanded_nodes, run2.expanded_nodes)

    def test_occupancy_grid_unmutated_after_planning(self) -> None:
        """World occupancy grid must be completely unchanged by planning."""
        world = World(width=50.0, height=50.0, grid_resolution=10.0)
        world.add_obstacle_cell(1, 1)
        world.add_obstacle_cell(2, 3)
        grid_copy = world.grid.copy()

        planner = AStarPlanner(world, connectivity=8)
        planner.plan((0, 0), (4, 4))

        np.testing.assert_array_equal(world.grid, grid_copy)


if __name__ == "__main__":
    unittest.main()
