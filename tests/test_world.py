"""
Unit tests for World model, 2D discrete occupancy grid, and coordinate transformations.
"""

import math
import unittest
import numpy as np

from autonomous_nav.world import World, Obstacle, CELL_FREE, CELL_OCCUPIED


class TestWorldGridInitialization(unittest.TestCase):
    """Test world and grid initialization semantics."""

    def test_default_grid_dimensions(self) -> None:
        """800x600 world with resolution 10 should produce an 80x60 grid."""
        world = World(width=800.0, height=600.0, grid_resolution=10.0)
        self.assertEqual(world.grid_width, 80)
        self.assertEqual(world.grid_height, 60)
        self.assertEqual(world.grid.shape, (60, 80))  # (rows, cols) -> (y, x)
        self.assertEqual(world.grid.dtype, np.uint8)
        self.assertEqual(np.count_nonzero(world.grid), 0)

    def test_custom_grid_resolution(self) -> None:
        """Test non-default resolution grid calculation."""
        world = World(width=100.0, height=50.0, grid_resolution=5.0)
        self.assertEqual(world.grid_width, 20)
        self.assertEqual(world.grid_height, 10)
        self.assertEqual(world.grid.shape, (10, 20))

    def test_invalid_world_dimensions_rejected(self) -> None:
        """Non-positive dimensions or resolutions must raise ValueError."""
        with self.assertRaises(ValueError):
            World(width=-100.0, height=100.0, grid_resolution=10.0)
        with self.assertRaises(ValueError):
            World(width=100.0, height=0.0, grid_resolution=10.0)
        with self.assertRaises(ValueError):
            World(width=100.0, height=100.0, grid_resolution=-5.0)
        with self.assertRaises(ValueError):
            World(width=5.0, height=5.0, grid_resolution=10.0)


class TestCoordinateTransformations(unittest.TestCase):
    """Test conversions between continuous world and discrete grid coordinates."""

    def setUp(self) -> None:
        self.world = World(width=800.0, height=600.0, grid_resolution=10.0)

    def test_world_to_grid_valid(self) -> None:
        """Verify continuous to discrete coordinate conversion."""
        self.assertEqual(self.world.world_to_grid(0.0, 0.0), (0, 0))
        self.assertEqual(self.world.world_to_grid(15.0, 25.0), (1, 2))
        self.assertEqual(self.world.world_to_grid(799.9, 599.9), (79, 59))

    def test_world_to_grid_out_of_bounds_raises(self) -> None:
        """Out of bounds world coordinates must raise ValueError."""
        with self.assertRaises(ValueError):
            self.world.world_to_grid(-1.0, 50.0)
        with self.assertRaises(ValueError):
            self.world.world_to_grid(50.0, -0.1)
        with self.assertRaises(ValueError):
            self.world.world_to_grid(800.0, 50.0)  # boundary is strictly < width
        with self.assertRaises(ValueError):
            self.world.world_to_grid(50.0, 600.0)

    def test_grid_to_world_center(self) -> None:
        """Verify discrete grid cell maps to the continuous cell center."""
        # Cell (0, 0) with resolution 10 centers at (5.0, 5.0)
        self.assertEqual(self.world.grid_to_world(0, 0), (5.0, 5.0))
        self.assertEqual(self.world.grid_to_world(7, 3), (75.0, 35.0))

    def test_grid_to_world_out_of_bounds_raises(self) -> None:
        """Out of bounds grid coordinates must raise IndexError."""
        with self.assertRaises(IndexError):
            self.world.grid_to_world(-1, 0)
        with self.assertRaises(IndexError):
            self.world.grid_to_world(80, 0)
        with self.assertRaises(IndexError):
            self.world.grid_to_world(0, 60)

    def test_coordinate_roundtrip(self) -> None:
        """Converting grid -> world center -> grid should be an exact identity."""
        for gx in (0, 10, 45, 79):
            for gy in (0, 8, 32, 59):
                wx, wy = self.world.grid_to_world(gx, gy)
                rgx, rgy = self.world.world_to_grid(wx, wy)
                self.assertEqual((gx, gy), (rgx, rgy))


class TestOccupancyAndObstacles(unittest.TestCase):
    """Test occupancy checks, cell modifications, and obstacle rasterization."""

    def setUp(self) -> None:
        self.world = World(width=800.0, height=600.0, grid_resolution=10.0)

    def test_empty_grid_traversability(self) -> None:
        """An empty world should be completely traversable."""
        self.assertTrue(self.world.is_traversable(5, 5))
        self.assertFalse(self.world.is_occupied(5, 5))
        self.assertTrue(self.world.is_world_traversable(55.0, 55.0))
        self.assertFalse(self.world.is_world_occupied(55.0, 55.0))

    def test_add_and_remove_obstacle_cell(self) -> None:
        """Test manually toggling cell occupancy."""
        self.world.add_obstacle_cell(12, 18)
        self.assertTrue(self.world.is_occupied(12, 18))
        self.assertFalse(self.world.is_traversable(12, 18))
        # Check numpy storage index convention: grid[gy, gx]
        self.assertEqual(self.world.grid[18, 12], CELL_OCCUPIED)

        self.world.remove_obstacle_cell(12, 18)
        self.assertFalse(self.world.is_occupied(12, 18))
        self.assertTrue(self.world.is_traversable(12, 18))
        self.assertEqual(self.world.grid[18, 12], CELL_FREE)

    def test_cell_operations_out_of_bounds_raise(self) -> None:
        """Modifying or querying out-of-bounds cells must raise IndexError."""
        with self.assertRaises(IndexError):
            self.world.add_obstacle_cell(85, 20)
        with self.assertRaises(IndexError):
            self.world.remove_obstacle_cell(10, 65)
        with self.assertRaises(IndexError):
            self.world.is_occupied(-1, 5)
        with self.assertRaises(IndexError):
            self.world.is_traversable(5, -1)

    def test_circle_obstacle_rasterization(self) -> None:
        """Test rasterization of a circular obstacle."""
        # Circle at (100, 100) with radius 20 covers grid cells around (10, 10)
        circle_obs = Obstacle(id=1, obstacle_type="circle", position=(100.0, 100.0), radius=20.0)
        self.world.add_obstacle(circle_obs)

        # Center should be occupied
        self.assertTrue(self.world.is_occupied(10, 10))
        # Cells within radius 20: (100, 115) -> cell (10, 11) is occupied
        self.assertTrue(self.world.is_occupied(10, 11))
        # Far cells should remain free
        self.assertFalse(self.world.is_occupied(0, 0))
        self.assertFalse(self.world.is_occupied(20, 20))

    def test_rectangle_obstacle_rasterization(self) -> None:
        """Test rasterization of a rectangular obstacle."""
        # Rectangle from (200, 200) with w=30, h=40 spans x:[200, 230], y:[200, 240]
        # In grid coords: gx in [20, 23], gy in [20, 24]
        rect_obs = Obstacle(id=2, obstacle_type="rectangle", position=(200.0, 200.0), width=30.0, height=40.0)
        self.world.add_obstacle(rect_obs)

        self.assertTrue(self.world.is_occupied(20, 20))
        self.assertTrue(self.world.is_occupied(22, 23))
        # Outside rectangle
        self.assertFalse(self.world.is_occupied(19, 20))
        self.assertFalse(self.world.is_occupied(24, 25))

    def test_remove_obstacle_by_id(self) -> None:
        """Test removing an obstacle restores grid cells to free."""
        obs = Obstacle(id=99, obstacle_type="circle", position=(50.0, 50.0), radius=15.0)
        self.world.add_obstacle(obs)
        self.assertTrue(self.world.is_occupied(5, 5))

        removed = self.world.remove_obstacle_by_id(99)
        self.assertTrue(removed)
        self.assertFalse(self.world.is_occupied(5, 5))
        self.assertEqual(len(self.world.obstacles), 0)

        # Non-existent ID returns False
        self.assertFalse(self.world.remove_obstacle_by_id(999))


class TestPositionValidation(unittest.TestCase):
    """Test start and goal position validation rules."""

    def setUp(self) -> None:
        self.world = World(width=800.0, height=600.0, grid_resolution=10.0)
        # Place obstacle at (200, 200) radius 40
        self.world.add_obstacle(
            Obstacle(id=1, obstacle_type="circle", position=(200.0, 200.0), radius=40.0)
        )

    def test_valid_start_and_goal(self) -> None:
        """Valid non-overlapping, in-bounds positions must be accepted."""
        start_grid, goal_grid = self.world.validate_start_goal((50.0, 50.0), (750.0, 550.0))
        self.assertEqual(start_grid, (5, 5))
        self.assertEqual(goal_grid, (75, 55))

    def test_start_out_of_bounds_rejected(self) -> None:
        """Start outside world bounds must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.world.validate_start_goal((-10.0, 50.0), (750.0, 550.0))
        self.assertIn("Start position", str(ctx.exception))

    def test_goal_out_of_bounds_rejected(self) -> None:
        """Goal outside world bounds must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.world.validate_start_goal((50.0, 50.0), (850.0, 550.0))
        self.assertIn("Goal position", str(ctx.exception))

    def test_start_inside_obstacle_rejected(self) -> None:
        """Start located inside an obstacle must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.world.validate_start_goal((200.0, 200.0), (750.0, 550.0))
        self.assertIn("occupied", str(ctx.exception).lower())

    def test_goal_inside_obstacle_rejected(self) -> None:
        """Goal located inside an obstacle must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.world.validate_start_goal((50.0, 50.0), (200.0, 200.0))
        self.assertIn("occupied", str(ctx.exception).lower())

    def test_identical_start_and_goal_rejected(self) -> None:
        """Identical start and goal mapping to same cell must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            self.world.validate_start_goal((50.0, 50.0), (52.0, 53.0))
        self.assertIn("identical grid cell", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
