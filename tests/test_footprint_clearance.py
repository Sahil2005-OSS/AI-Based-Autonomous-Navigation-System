"""
Unit and integration tests for Task 07: Robot Footprint and Obstacle-Clearance Safety.

Tests cover:
- Configurable circular robot radius validation (positive finite numbers).
- Obstacle clearance with circular footprint (center in free cell but footprint overlaps obstacle).
- World boundary clearance with circular footprint (center inside bounds but footprint crosses edge).
- Safely separated footprints accepted.
- Start and goal validation with footprint clearance.
- Footprint-safe A* path planning avoiding obstacle clearance buffers.
- Path smoothing rejecting shortcuts that violate footprint clearance.
- Continuous motion controller rejecting unsafe candidate poses and retaining the last safe pose.
- Consistent clearance semantics across world, planner, smoother, and controller.
- Headless visualizer rendering with clearance zones and footprint rings.
- End-to-end default scenario navigability with R=15.0 footprint.
"""

import math
from pathlib import Path
import pytest
import numpy as np

from autonomous_nav.world import (
    World,
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_CLEARANCE_VIOLATION,
)
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.path_smoother import PathSmoother, has_line_of_sight
from autonomous_nav.motion_model import RobotState, KinematicMotionModel
from autonomous_nav.controller import (
    PurePursuitController,
    ControllerStatus,
    run_simulation,
    step_simulation,
)
from autonomous_nav.visualizer import SimulationVisualizer


class TestFootprintValidationAndGeometry:
    """Tests for radius validation and geometric collision checking."""

    def test_robot_radius_validation_valid(self):
        """World accepts valid non-negative finite robot radii."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        assert world.robot_radius == 15.0

        world.robot_radius = 0.0
        assert world.robot_radius == 0.0

        world.robot_radius = 25.5
        assert world.robot_radius == 25.5

    def test_robot_radius_validation_invalid(self):
        """World rejects negative, NaN, infinite, or non-numeric radius values."""
        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            World(100.0, 100.0, 10.0, robot_radius=-5.0)

        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            World(100.0, 100.0, 10.0, robot_radius=float("nan"))

        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            World(100.0, 100.0, 10.0, robot_radius=float("inf"))

        world = World(100.0, 100.0, 10.0, robot_radius=10.0)
        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            world.robot_radius = -1.0

        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            world.robot_radius = float("-inf")

    def test_non_finite_coordinates_rejected(self):
        """check_footprint_collision and is_footprint_safe reject non-finite poses."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)

        is_safe, reason = world.check_footprint_collision(float("nan"), 50.0)
        assert is_safe is False
        assert "non-finite" in reason.lower()
        assert world.is_footprint_safe(float("nan"), 50.0) is False

        is_safe, reason = world.check_footprint_collision(50.0, float("inf"))
        assert is_safe is False
        assert "non-finite" in reason.lower()
        assert world.is_footprint_safe(50.0, float("inf")) is False

    def test_center_in_free_cell_but_footprint_overlaps_obstacle(self):
        """
        Verify that a robot center located inside a free grid cell is rejected
        when its circular footprint overlaps an adjacent obstacle.
        """
        # World 100x100, resolution 10.0
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        # Obstacle at grid cell (5, 5) -> world [50, 60] x [50, 60]
        world.add_obstacle_cell(5, 5)

        # Robot center at (45.0, 55.0):
        # Grid cell is (4, 5) -> world [40, 50] x [50, 60]
        # Cell (4, 5) is completely free!
        assert world.is_traversable(4, 5) is True
        assert world.is_inside_world(45.0, 55.0) is True

        # Distance from (45, 55) to obstacle rectangle [50, 60] x [50, 60]:
        # Nearest x is 50.0, nearest y is 55.0. Distance = 5.0.
        # Since 5.0 <= radius (15.0), the footprint collides with the obstacle!
        is_safe, reason = world.check_footprint_collision(45.0, 55.0)
        assert is_safe is False
        assert "overlaps occupied obstacle" in reason.lower()
        assert world.is_footprint_safe(45.0, 55.0) is False

    def test_footprint_violates_world_boundary_with_center_inside(self):
        """
        Verify that a robot center located inside world boundaries is rejected
        if its footprint extends past any boundary edge.
        """
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)

        # Near left boundary: x=10.0, y=50.0 -> center is inside, but x - R = -5.0 < 0
        assert world.is_inside_world(10.0, 50.0) is True
        is_safe, reason = world.check_footprint_collision(10.0, 50.0)
        assert is_safe is False
        assert "outside world bounds" in reason.lower()
        assert world.is_footprint_safe(10.0, 50.0) is False

        # Near bottom boundary: x=50.0, y=10.0 -> y - R = -5.0 < 0
        is_safe, reason = world.check_footprint_collision(50.0, 10.0)
        assert is_safe is False
        assert "outside world bounds" in reason.lower()

        # Near right boundary: x=95.0, y=50.0 -> x + R = 110.0 > 100.0
        is_safe, reason = world.check_footprint_collision(95.0, 50.0)
        assert is_safe is False
        assert "outside world bounds" in reason.lower()

        # Near top boundary: x=50.0, y=95.0 -> y + R = 110.0 > 100.0
        is_safe, reason = world.check_footprint_collision(50.0, 95.0)
        assert is_safe is False
        assert "outside world bounds" in reason.lower()

    def test_safely_separated_footprint_accepted(self):
        """A pose with adequate obstacle and boundary clearance is accepted."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        world.add_obstacle_cell(0, 0)  # Obstacle at [0, 10] x [0, 10]

        # Center at (50.0, 50.0): distance to all boundaries is 50.0 >= 15.0
        # Distance to obstacle is > 50.0 >= 15.0
        is_safe, reason = world.check_footprint_collision(50.0, 50.0)
        assert is_safe is True
        assert reason == "Footprint is clear and safe."
        assert world.is_footprint_safe(50.0, 50.0) is True

    def test_start_and_goal_validation_enforces_footprint_clearance(self):
        """World.validate_start_goal rejects positions without footprint clearance."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        world.add_obstacle_cell(5, 5)  # [50, 60] x [50, 60]

        # Start close to left boundary (10, 50)
        with pytest.raises(ValueError, match="does not have adequate robot footprint clearance"):
            world.validate_start_goal((10.0, 50.0), (80.0, 80.0), check_footprint=True)

        # Start close to obstacle (48, 55)
        with pytest.raises(ValueError, match="does not have adequate robot footprint clearance"):
            world.validate_start_goal((48.0, 55.0), (80.0, 80.0), check_footprint=True)

        # Goal close to top boundary (80, 95)
        with pytest.raises(ValueError, match="does not have adequate robot footprint clearance"):
            world.validate_start_goal((25.0, 25.0), (80.0, 95.0), check_footprint=True)

        # When check_footprint is False (backward compatibility), point-in-bounds succeeds
        sg = world.validate_start_goal((10.0, 50.0), (80.0, 80.0), check_footprint=False)
        assert sg == ((1, 5), (8, 8))


class TestClearanceIntegration:
    """Tests verifying clearance policy integration with A*, PathSmoother, and Motion Controller."""

    def test_astar_avoids_footprint_clearance_buffer(self):
        """A* planner with use_footprint=True routes around clearance-inflated cells."""
        # World 200x200, resolution 10. Radius 15.
        world = World(200.0, 200.0, 10.0, robot_radius=15.0)
        # Place obstacle in the center column
        for gy in range(8, 13):
            world.add_obstacle_cell(10, gy)

        # Start on left (gx=4, gy=10) -> world (45, 105)
        # Goal on right (gx=16, gy=10) -> world (165, 105)
        start = (4, 10)
        goal = (16, 10)

        planner = AStarPlanner(world, connectivity=8, use_footprint=True)
        res = planner.plan(start, goal)

        assert res.success is True
        assert len(res.path) > 0
        # Check every waypoint in the path is footprint-safe
        for gx, gy in res.path:
            assert world.is_footprint_safe_grid(gx, gy) is True
            assert world.clearance_grid[gy, gx] == CELL_FREE

    def test_smoothing_rejects_shortcut_violating_clearance_zone(self):
        """Path smoother rejects shortcuts that pass through clearance zones."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        # Single obstacle cell at (5, 5) -> [50, 60] x [50, 60]
        world.add_obstacle_cell(5, 5)

        # Direct line from (2, 6) to (8, 6):
        # Row 6 is free of obstacles (world y in [60, 70]).
        # But cell (5, 6) is immediately adjacent to (5, 5) (distance 0 to 10 < 15.0)!
        # So cell (5, 6) is in the clearance zone!
        assert world.grid[6, 5] == CELL_FREE
        assert world.clearance_grid[6, 5] == CELL_CLEARANCE_VIOLATION

        # A shortcut between (2, 6) and (8, 6) would cross (5, 6).
        # Without footprint check:
        los_no_fp = has_line_of_sight(world, (2, 6), (8, 6), check_footprint=False)
        assert los_no_fp is True  # Cell (5, 6) has no obstacle cell center

        # With footprint check:
        los_with_fp = has_line_of_sight(world, (2, 6), (8, 6), check_footprint=True)
        assert los_with_fp is False  # Must be rejected because it violates clearance!

    def test_continuous_motion_rejects_unsafe_candidate_and_preserves_safe_pose(self):
        """PurePursuitController halts and preserves the last safe pose on clearance violation."""
        world = World(200.0, 200.0, 10.0, robot_radius=15.0)
        # Place obstacle at (10, 10) -> [100, 110] x [100, 110]
        world.add_obstacle_cell(10, 10)

        # Place robot facing the obstacle at x=60.0, y=105.0, theta=0.0 (moving +x)
        # The obstacle is at x in [100, 110].
        # Footprint boundary extends 15 ahead of robot center.
        init_state = RobotState(x=60.0, y=105.0, theta=0.0, v=20.0)

        # Create controller with path leading straight into obstacle
        ctrl = PurePursuitController(
            world=world,
            lookahead_distance=20.0,
            check_footprint=True,
            robot_radius=15.0,
        )
        ctrl.set_path([(6, 10), (7, 10), (8, 10), (9, 10), (10, 10)])
        motion_model = KinematicMotionModel()

        state = init_state
        last_safe_state = state
        collided = False

        for _ in range(60):
            prev_x, prev_y = state.x, state.y
            next_state, ctrl_out = step_simulation(state, ctrl, motion_model, dt=0.05)
            if ctrl_out.status == ControllerStatus.COLLISION:
                collided = True
                assert next_state.x == prev_x  # Preserved last safe pose!
                assert next_state.y == prev_y
                break
            state = next_state

        assert collided is True
        # Verify the preserved pose is still footprint-safe
        assert world.is_footprint_safe(state.x, state.y) is True

    def test_consistent_clearance_policy_across_modules(self):
        """
        Verify that World, Planner, Smoother, and Controller all agree on whether
        specific sample poses are safe or unsafe.
        """
        world = World(200.0, 200.0, 10.0, robot_radius=15.0)
        world.add_obstacle_cell(10, 10)  # [100, 110] x [100, 110]

        ctrl = PurePursuitController(world=world, check_footprint=True, robot_radius=15.0)

        test_points = [
            (25.0, 25.0, True),    # Interior free space
            (5.0, 25.0, False),    # Violates left boundary (5 - 15 = -10 < 0)
            (25.0, 5.0, False),    # Violates bottom boundary
            (195.0, 25.0, False),  # Violates right boundary (195 + 15 = 210 > 200)
            (25.0, 195.0, False),  # Violates top boundary
            (90.0, 105.0, False),  # Distance to [100, 110] is 10.0 <= 15.0 (obstacle clearance)
            (80.0, 105.0, True),   # Distance to [100, 110] is 20.0 > 15.0 (safe)
        ]

        for wx, wy, expected_safe in test_points:
            # 1. World API
            world_safe = world.is_footprint_safe(wx, wy)
            assert world_safe == expected_safe, f"World check mismatch at ({wx}, {wy})"

            # 2. Controller collision check
            ctrl_safe, _ = ctrl.check_step_collision(wx, wy)
            assert ctrl_safe == expected_safe, f"Controller check mismatch at ({wx}, {wy})"

            # 3. Grid clearance consistency at corresponding cell center
            gx, gy = world.world_to_grid(wx, wy)
            cell_center_x, cell_center_y = world.grid_to_world(gx, gy)
            cell_center_safe = world.is_footprint_safe(cell_center_x, cell_center_y)
            grid_safe = world.is_footprint_safe_grid(gx, gy)
            assert grid_safe == cell_center_safe, f"Grid vs World mismatch at cell ({gx}, {gy})"


class TestDefaultScenarioNavigability:
    """End-to-end navigability and metrics for default scenario with R=15.0."""

    def test_default_scenario_execution(self):
        """Default scenario successfully navigates end-to-end with robot radius 15.0."""
        from autonomous_nav.config_loader import load_config

        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_scenario.json"
        config_data = load_config(str(cfg_path))

        world = World.from_config(config_data)
        assert world.robot_radius == 15.0

        robot_cfg = config_data["robot"]
        start_pos = tuple(robot_cfg["start_position"])
        target_pos = tuple(robot_cfg["target_position"])

        start_grid, goal_grid = world.validate_start_goal(start_pos, target_pos, check_footprint=True)
        assert start_grid == (5, 5)
        assert goal_grid == (75, 55)

        # Plan with footprint clearance
        planner = AStarPlanner(world, connectivity=8, use_footprint=True)
        res = planner.plan(start_grid, goal_grid)
        assert res.success is True
        assert len(res.path) > 0
        assert all(world.is_footprint_safe_grid(gx, gy) for gx, gy in res.path)

        # Smooth with footprint clearance
        smoother = PathSmoother(world, check_footprint=True)
        smooth_res = smoother.smooth_result(res)
        assert smooth_res.success is True
        assert len(smooth_res.smoothed_path) < len(res.path)
        assert all(
            has_line_of_sight(world, p1, p2, check_footprint=True)
            for p1, p2 in zip(smooth_res.smoothed_path[:-1], smooth_res.smoothed_path[1:])
        )

        # Motion simulation with footprint clearance
        metrics = run_simulation(
            world=world,
            path=smooth_res.smoothed_path,
            max_steps=2000,
            dt=0.05,
            check_footprint=True,
            robot_radius=world.robot_radius,
        )
        assert metrics.goal_reached is True
        assert metrics.collision_occurred is False
        assert metrics.status == ControllerStatus.REACHED_GOAL
        assert metrics.steps < 1000

    def test_headless_visualizer_rendering(self):
        """SimulationVisualizer instantiates and renders without display errors headlessly."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        planner = AStarPlanner(world, connectivity=8, use_footprint=True)
        res = planner.plan((2, 2), (8, 8))

        vis = SimulationVisualizer(
            world=world,
            path_result=res,
            start_grid=(2, 2),
            goal_grid=(8, 8),
            connectivity=8,
            continuous_motion=True,
            check_footprint=True,
            robot_radius=15.0,
            init_display=False,
        )
        assert vis.robot_radius == 15.0
        assert vis.check_footprint is True

        # Render on offline surface
        import pygame
        surf = pygame.Surface((vis.window_width, vis.window_height))
        vis.draw_arena(surf)
        vis.draw_markers(surf)
        vis.draw_sidebar(surf)


class TestSweptFootprintSafety:
    """
    Targeted regression tests for Task 07.1: Continuous Swept-Footprint Collision Safety.
    Verifies that continuous robot motion checks the complete swept segment, not just endpoints.
    """

    def test_movement_with_safe_endpoints_crossing_obstacle_rejected(self):
        """
        A movement whose starting and ending positions are both footprint-safe,
        but whose center path tunnels through an occupied obstacle, must be rejected.
        """
        world = World(200.0, 200.0, 10.0, robot_radius=5.0)
        # Obstacle at grid cell (10, 10) -> world [100, 110] x [100, 110]
        world.add_obstacle_cell(10, 10)

        # Endpoints at x=80 (dist=20 > 5) and x=130 (dist=20 > 5) at y=105
        p0 = (80.0, 105.0)
        p1 = (130.0, 105.0)

        # Both endpoints are strictly footprint-safe in isolation
        assert world.is_footprint_safe(p0[0], p0[1], radius=5.0) is True
        assert world.is_footprint_safe(p1[0], p1[1], radius=5.0) is True

        # Swept movement crosses through the obstacle and must be rejected
        is_safe, reason = world.check_swept_footprint_collision(p0[0], p0[1], p1[0], p1[1], radius=5.0)
        assert is_safe is False
        assert "collides with or grazes occupied obstacle cell" in reason
        assert world.is_swept_footprint_safe(p0[0], p0[1], p1[0], p1[1], radius=5.0) is False

    def test_movement_with_safe_endpoints_grazing_obstacle_rejected(self):
        """
        A movement whose endpoints are safely clear, but whose intermediate trajectory
        grazes or penetrates an obstacle corner closer than the robot radius, must be rejected.
        """
        world = World(100.0, 100.0, 10.0, robot_radius=5.0)
        # Obstacle at (5, 5) -> [50, 60] x [50, 60]. Corner is at (50, 60).
        world.add_obstacle_cell(5, 5)

        # Segment from (40.0, 67.0) to (67.0, 40.0). Line x + y = 107.
        # Distance from corner (50, 60) to line is |50 + 60 - 107| / sqrt(2) = 3 / 1.414 = 2.12 <= 5.0
        # Endpoints have distance sqrt((40-50)^2 + (67-60)^2) = sqrt(149) = 12.2 > 5.0
        p0 = (40.0, 67.0)
        p1 = (67.0, 40.0)

        assert world.is_footprint_safe(p0[0], p0[1], radius=5.0) is True
        assert world.is_footprint_safe(p1[0], p1[1], radius=5.0) is True

        # Swept footprint penetrates the 5.0 radius buffer of corner (50, 60)
        is_safe, reason = world.check_swept_footprint_collision(p0[0], p0[1], p1[0], p1[1], radius=5.0)
        assert is_safe is False
        assert "collides with or grazes occupied obstacle cell" in reason
        assert world.is_swept_footprint_safe(p0[0], p0[1], p1[0], p1[1], radius=5.0) is False

    def test_safe_movement_with_sufficient_clearance_accepted(self):
        """A swept movement that maintains strict clearance along the entire segment is accepted."""
        world = World(100.0, 100.0, 10.0, robot_radius=5.0)
        world.add_obstacle_cell(5, 5)  # [50, 60] x [50, 60]

        # Parallel movement along y=80 from x=20 to x=80. Distance to y=60 is 20.0 > 5.0
        p0 = (20.0, 80.0)
        p1 = (80.0, 80.0)

        is_safe, reason = world.check_swept_footprint_collision(p0[0], p0[1], p1[0], p1[1], radius=5.0)
        assert is_safe is True
        assert reason == "Swept footprint is clear and safe."
        assert world.is_swept_footprint_safe(p0[0], p0[1], p1[0], p1[1], radius=5.0) is True

    def test_movement_violating_boundary_along_segment_rejected(self):
        """A movement that crosses or touches a world boundary along the segment is rejected."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)

        # Starts inside clearance: x=25 (25 - 15 = 10 > 0), moves to x=10 (10 - 15 = -5 < 0)
        is_safe, reason = world.check_swept_footprint_collision(25.0, 50.0, 10.0, 50.0, radius=15.0)
        assert is_safe is False
        assert "extends outside or touches world bounds" in reason

        # Tangential contact with world boundary: x=15 (15 - 15 = 0.0)
        # Conservative policy rejects tangential boundary contact
        is_safe_tangent, _ = world.check_swept_footprint_collision(25.0, 50.0, 15.0, 50.0, radius=15.0)
        assert is_safe_tangent is False

    def test_zero_length_movement_handled_correctly(self):
        """A zero-length segment (x0==x1, y0==y1) evaluates the stationary circular footprint."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)
        world.add_obstacle_cell(5, 5)  # [50, 60] x [50, 60]

        # Safe interior zero-length movement
        assert world.is_swept_footprint_safe(25.0, 25.0, 25.0, 25.0, radius=15.0) is True

        # Unsafe zero-length movement touching boundary (x=15)
        assert world.is_swept_footprint_safe(15.0, 50.0, 15.0, 50.0, radius=15.0) is False

        # Unsafe zero-length movement overlapping obstacle (center at (55, 55))
        assert world.is_swept_footprint_safe(55.0, 55.0, 55.0, 55.0, radius=15.0) is False

    def test_swept_validation_rejects_non_finite_and_invalid_inputs(self):
        """check_swept_footprint_collision strictly validates radius and coordinate inputs."""
        world = World(100.0, 100.0, 10.0, robot_radius=15.0)

        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            world.check_swept_footprint_collision(20.0, 20.0, 40.0, 40.0, radius=-2.0)

        with pytest.raises(ValueError, match="Robot radius must be a non-negative finite number"):
            world.check_swept_footprint_collision(20.0, 20.0, 40.0, 40.0, radius=float("nan"))

        is_safe, reason = world.check_swept_footprint_collision(float("nan"), 20.0, 40.0, 40.0)
        assert is_safe is False
        assert "non-finite or invalid" in reason

        is_safe, reason = world.check_swept_footprint_collision(20.0, 20.0, float("inf"), 40.0)
        assert is_safe is False
        assert "non-finite or invalid" in reason

    def test_controller_retains_last_safe_state_after_rejecting_swept_obstacle(self):
        """
        PurePursuitController motion stepping rejects a proposed step whose swept trajectory
        intersects an obstacle, halting the robot and preserving the last safe pose.
        """
        world = World(200.0, 200.0, 10.0, robot_radius=5.0)
        world.add_obstacle_cell(10, 10)  # [100, 110] x [100, 110]

        # Place robot close to obstacle moving in +x direction
        init_state = RobotState(x=92.0, y=105.0, theta=0.0, v=20.0)

        ctrl = PurePursuitController(
            world=world,
            lookahead_distance=20.0,
            check_footprint=True,
            robot_radius=5.0,
        )
        ctrl.set_path([(9, 10), (10, 10), (11, 10)])
        motion_model = KinematicMotionModel()

        # Step simulation: proposed candidate step moves x from 92.0 towards 93.0
        # which penetrates the obstacle [100, 110] when x + 5 >= 97 (or on entry)
        state = init_state
        collided = False
        for _ in range(20):
            prev_x, prev_y, prev_theta = state.x, state.y, state.theta
            next_state, ctrl_out = step_simulation(state, ctrl, motion_model, dt=0.05)
            if ctrl_out.status == ControllerStatus.COLLISION:
                collided = True
                assert next_state.x == prev_x
                assert next_state.y == prev_y
                assert next_state.theta == prev_theta
                assert next_state.v == 0.0
                assert next_state.omega == 0.0
                assert "Collision rejected" in ctrl_out.message
                break
            state = next_state

        assert collided is True
        assert world.is_footprint_safe(state.x, state.y, radius=5.0) is True

    def test_endpoint_only_vs_swept_segment_distinction(self):
        """
        Key regression test proving that endpoint-only checking passes an unsafe move,
        while swept-footprint checking correctly detects and rejects the collision.
        """
        world = World(200.0, 200.0, 10.0, robot_radius=5.0)
        world.add_obstacle_cell(10, 10)  # [100, 110] x [100, 110]

        # Choose endpoints on opposite sides of the obstacle
        x0, y0 = 80.0, 105.0
        x1, y1 = 130.0, 105.0

        # 1. Endpoint-only checks: both return SAFE (True)
        endpoint_0_safe = world.is_footprint_safe(x0, y0, radius=5.0)
        endpoint_1_safe = world.is_footprint_safe(x1, y1, radius=5.0)
        assert endpoint_0_safe is True
        assert endpoint_1_safe is True

        # 2. Swept-segment check: correctly returns UNSAFE (False)
        swept_safe = world.is_swept_footprint_safe(x0, y0, x1, y1, radius=5.0)
        assert swept_safe is False
