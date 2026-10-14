"""
Unit tests for Pure Pursuit Controller and Simulation Runner (src/autonomous_nav/controller.py).
"""

import math
from pathlib import Path
import unittest
import numpy as np

from autonomous_nav.world import World
from autonomous_nav.motion_model import RobotState, KinematicMotionModel
from autonomous_nav.controller import (
    PurePursuitController,
    ControllerStatus,
    step_simulation,
    run_simulation,
)
from autonomous_nav.config_loader import load_config
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.path_smoother import PathSmoother


class TestPurePursuitControllerBasics(unittest.TestCase):
    """Test controller initialization, path setting, and input validation."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0)  # 10x10 grid
        self.controller = PurePursuitController(self.world)

    def test_parameter_validation_on_init(self) -> None:
        """Invalid controller parameters raise ValueError."""
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, lookahead_distance=-5.0)
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, max_linear_speed=0.0)
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, max_angular_speed=-1.0)
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, goal_tolerance=0.0)
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, goal_tolerance=10.0, slowdown_radius=5.0)
        with self.assertRaises(ValueError):
            PurePursuitController(self.world, min_linear_speed=-2.0)

    def test_empty_path_handling(self) -> None:
        """Setting an empty path returns FAILED status and zero velocity."""
        self.controller.set_path([])
        state = RobotState(x=10.0, y=10.0, theta=0.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.FAILED)
        self.assertEqual(out.v_cmd, 0.0)
        self.assertEqual(out.omega_cmd, 0.0)

    def test_single_point_path_at_robot_declares_goal_arrival(self) -> None:
        """If path consists only of the current cell, goal arrival is declared."""
        # Grid (1, 1) -> world center (15.0, 15.0)
        self.controller.set_path([(1, 1)])
        state = RobotState(x=15.0, y=15.0, theta=0.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.REACHED_GOAL)
        self.assertEqual(out.v_cmd, 0.0)
        self.assertEqual(out.omega_cmd, 0.0)

    def test_grid_to_world_waypoint_conversion(self) -> None:
        """Waypoints are correctly converted using world.grid_to_world."""
        self.controller.set_path([(0, 0), (2, 3)])
        expected_world = [self.world.grid_to_world(0, 0), self.world.grid_to_world(2, 3)]
        self.assertEqual(self.controller.world_path, expected_world)


class TestPurePursuitSteeringGeometry(unittest.TestCase):
    """Test geometric steering calculation and angular rates."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0)
        self.controller = PurePursuitController(
            self.world,
            lookahead_distance=20.0,
            max_linear_speed=10.0,
            max_angular_speed=2.0,
        )

    def test_steering_straight_when_target_directly_ahead(self) -> None:
        """When target is directly in front along heading, omega is approximately 0."""
        # Path along x-axis from (0, 0) to (5, 0)
        self.controller.set_path([(0, 0), (5, 0)])
        # Robot at start, facing +x (theta=0)
        w0 = self.world.grid_to_world(0, 0)
        state = RobotState(x=w0[0], y=w0[1], theta=0.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.TRACKING)
        self.assertAlmostEqual(out.omega_cmd, 0.0, places=4)
        self.assertGreater(out.v_cmd, 0.0)

    def test_turning_left_when_target_to_the_left(self) -> None:
        """When target is to the left of heading, commanded omega is positive (CCW)."""
        # Target at (5, 5), robot at (5, 0) facing East (theta=0)
        self.controller.set_path([(0, 0), (0, 5)])  # Going North
        w0 = self.world.grid_to_world(0, 0)
        # Robot facing East (theta=0), target is North (+y, to the left)
        state = RobotState(x=w0[0], y=w0[1], theta=0.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.TRACKING)
        self.assertGreater(out.omega_cmd, 0.0)  # Positive turning rate (counter-clockwise)

    def test_turning_right_when_target_to_the_right(self) -> None:
        """When target is to the right of heading, commanded omega is negative (CW)."""
        # Target at (5, 0), robot facing North (theta=pi/2)
        self.controller.set_path([(0, 0), (5, 0)])
        w0 = self.world.grid_to_world(0, 0)
        state = RobotState(x=w0[0], y=w0[1], theta=math.pi / 2.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.TRACKING)
        self.assertLess(out.omega_cmd, 0.0)  # Negative turning rate (clockwise)

    def test_goal_arrival_and_stopping(self) -> None:
        """When robot enters goal tolerance, velocity is 0 and status is REACHED_GOAL."""
        self.controller.set_path([(0, 0), (3, 3)])
        goal_w = self.world.grid_to_world(3, 3)
        # Place robot 2 units from goal (within default goal_tolerance 5.0)
        state = RobotState(x=goal_w[0] + 1.0, y=goal_w[1] + 1.0, theta=0.0)
        out = self.controller.compute_command(state)

        self.assertEqual(out.status, ControllerStatus.REACHED_GOAL)
        self.assertEqual(out.v_cmd, 0.0)
        self.assertEqual(out.omega_cmd, 0.0)


class TestCollisionCheckingAndSimulationStep(unittest.TestCase):
    """Test step_simulation with obstacle and boundary collision rejection."""

    def setUp(self) -> None:
        self.world = World(width=50.0, height=50.0, grid_resolution=10.0)  # 5x5 grid
        self.controller = PurePursuitController(self.world)
        self.motion_model = KinematicMotionModel()

    def test_boundary_crossing_rejected(self) -> None:
        """Moving outside the world boundaries triggers COLLISION and stops the robot."""
        # Safe is False for negative coordinates
        is_safe, reason = self.controller.check_step_collision(-1.0, 5.0)
        self.assertFalse(is_safe)
        self.assertIn("outside world bounds", reason)

        # Safe is False for coordinates >= width/height
        is_safe, reason = self.controller.check_step_collision(51.0, 5.0)
        self.assertFalse(is_safe)

    def test_obstacle_entry_rejected(self) -> None:
        """Moving into an occupied cell triggers COLLISION and stops the robot."""
        # Add obstacle at grid (2, 2)
        self.world.add_obstacle_cell(2, 2)
        obs_center = self.world.grid_to_world(2, 2)

        is_safe, reason = self.controller.check_step_collision(obs_center[0], obs_center[1])
        self.assertFalse(is_safe)
        self.assertIn("occupied obstacle cell", reason)

    def test_step_simulation_halts_on_collision(self) -> None:
        """step_simulation stops robot state and returns COLLISION output on obstacle contact."""
        self.world.add_obstacle_cell(1, 0)  # Obstacle at (1, 0)
        self.controller.set_path([(0, 0), (2, 0)])

        w0 = self.world.grid_to_world(0, 0)  # (5.0, 5.0)
        state = RobotState(x=w0[0], y=w0[1], theta=0.0)

        # Large speed driving directly into obstacle cell (1, 0)
        new_state, ctrl_out = step_simulation(
            state, self.controller, self.motion_model, dt=1.0
        )

        # Robot must stop before entering obstacle cell
        self.assertEqual(ctrl_out.status, ControllerStatus.COLLISION)
        self.assertEqual(new_state.v, 0.0)
        self.assertEqual(new_state.omega, 0.0)
        self.assertFalse(self.world.is_world_occupied(new_state.x, new_state.y))

    def test_occupancy_grid_unmutated_after_simulation(self) -> None:
        """Simulation steps must not alter world occupancy grid."""
        grid_copy = self.world.grid.copy()
        self.controller.set_path([(0, 0), (4, 4)])

        state = RobotState(x=5.0, y=5.0, theta=0.0)
        for _ in range(10):
            state, _ = step_simulation(state, self.controller, self.motion_model, dt=0.05)

        np.testing.assert_array_equal(self.world.grid, grid_copy)


class TestEndToEndSimulationRun(unittest.TestCase):
    """Test full simulation execution and path convergence."""

    def setUp(self) -> None:
        self.world = World(width=100.0, height=100.0, grid_resolution=10.0, robot_radius=0.0)

    def test_straight_path_tracking_reaches_goal(self) -> None:
        """Robot smoothly tracks a straight line and reaches goal within max_steps."""
        path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        metrics = run_simulation(
            self.world,
            path=path,
            max_steps=500,
            dt=0.05,
            max_linear_speed=20.0,
            goal_tolerance=5.0,
            check_footprint=False,
        )

        self.assertTrue(metrics.success)
        self.assertTrue(metrics.goal_reached)
        self.assertFalse(metrics.collision_occurred)
        self.assertEqual(metrics.status, ControllerStatus.REACHED_GOAL)
        self.assertLess(metrics.steps, 500)

        goal_world = self.world.grid_to_world(4, 0)
        dist_to_goal = math.hypot(goal_world[0] - metrics.final_pose[0], goal_world[1] - metrics.final_pose[1])
        self.assertLessEqual(dist_to_goal, 5.0)

    def test_default_scenario_reaches_goal_headlessly(self) -> None:
        """End-to-end default scenario simulation completes with 100% collision-safety."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_scenario.json"
        self.assertTrue(cfg_path.is_file())

        cfg = load_config(cfg_path)
        world = World.from_config(cfg)
        planner = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True, use_footprint=True)
        astar_res = planner.plan((5, 5), (75, 55))
        self.assertTrue(astar_res.success)

        smoother = PathSmoother(world, check_footprint=True)
        smooth_res = smoother.smooth_result(astar_res)
        self.assertTrue(smooth_res.success)

        metrics = run_simulation(
            world=world,
            path=smooth_res.smoothed_path,
            max_steps=2000,
            dt=0.05,
            max_linear_speed=40.0,
            max_angular_speed=2.5,
            lookahead_distance=25.0,
            goal_tolerance=5.0,
            check_footprint=True,
            robot_radius=world.robot_radius,
        )

        self.assertTrue(metrics.success)
        self.assertTrue(metrics.goal_reached)
        self.assertFalse(metrics.collision_occurred)
        self.assertEqual(metrics.status, ControllerStatus.REACHED_GOAL)
        self.assertLess(metrics.steps, 1000)
        self.assertGreater(metrics.elapsed_time, 0.0)


if __name__ == "__main__":
    unittest.main()
