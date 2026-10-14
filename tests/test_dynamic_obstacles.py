"""
Unit and integration tests for Task 09: Dynamic Obstacles and Safe Replanning.

Tests cover:
- Deterministic dynamic obstacle motion rule, simulation-time parameterization, and bounds.
- Point-to-segment distance and route conflict detection with safety margins.
- Temporary planning world creation without corrupting base static world.
- Successful A* replanning when an alternate detour route exists.
- Safe stopping with ControllerStatus.BLOCKED when no safe route exists.
- Preservation of robot footprint and swept collision checks during replanning and motion.
- Elimination of unnecessary replanning when route remains safe.
- Evaluation suite integration and metric recording.
"""

from __future__ import annotations

import math
from pathlib import Path
import pytest
import numpy as np

from autonomous_nav.dynamic_obstacle import (
    DynamicObstacle,
    point_to_segment_distance,
    check_route_conflict,
    create_temporary_planning_world,
    find_safe_start_grid,
    check_moving_obstacle_step_collision,
)
from autonomous_nav.world import World, CELL_OCCUPIED, CELL_FREE
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.path_smoother import PathSmoother
from autonomous_nav.motion_model import RobotState, KinematicMotionModel
from autonomous_nav.controller import (
    PurePursuitController,
    ControllerStatus,
    run_simulation,
    step_simulation,
)
from autonomous_nav.evaluation import evaluate_scenario, run_evaluation_suite


class TestDynamicObstacleModel:
    """Tests for deterministic linear patrol motion, simulation-time stepping, and bounds."""

    def test_deterministic_motion_along_segment(self):
        """Obstacle position is a pure deterministic function of simulation time."""
        # 40-unit segment along x-axis from (10, 20) to (50, 20) at speed 20 units/s.
        # Total round trip: 80 units -> period = 4.0 s.
        obs = DynamicObstacle(
            obstacle_id=1,
            point_a=(10.0, 20.0),
            point_b=(50.0, 20.0),
            speed=20.0,
            radius=5.0,
        )

        # t = 0.0: at point_a
        pos_0 = obs.position_at(0.0)
        assert pytest.approx(pos_0[0], abs=1e-5) == 10.0
        assert pytest.approx(pos_0[1], abs=1e-5) == 20.0

        # t = 1.0: halfway to point_b (10 + 20 = 30)
        pos_1 = obs.position_at(1.0)
        assert pytest.approx(pos_1[0], abs=1e-5) == 30.0
        assert pytest.approx(pos_1[1], abs=1e-5) == 20.0

        # t = 2.0: exactly at point_b (turning point)
        pos_2 = obs.position_at(2.0)
        assert pytest.approx(pos_2[0], abs=1e-5) == 50.0
        assert pytest.approx(pos_2[1], abs=1e-5) == 20.0

        # t = 3.0: halfway back toward point_a (50 - 20 = 30)
        pos_3 = obs.position_at(3.0)
        assert pytest.approx(pos_3[0], abs=1e-5) == 30.0
        assert pytest.approx(pos_3[1], abs=1e-5) == 20.0

        # t = 4.0: exactly back at point_a (period completed)
        pos_4 = obs.position_at(4.0)
        assert pytest.approx(pos_4[0], abs=1e-5) == 10.0
        assert pytest.approx(pos_4[1], abs=1e-5) == 20.0

        # t = 5.0: second cycle halfway forward
        pos_5 = obs.position_at(5.0)
        assert pytest.approx(pos_5[0], abs=1e-5) == 30.0
        assert pytest.approx(pos_5[1], abs=1e-5) == 20.0

    def test_motion_strictly_within_bounds(self):
        """Obstacle position is strictly contained within segment bounding box for all times."""
        obs = DynamicObstacle(
            obstacle_id=2,
            point_a=(30.0, 40.0),
            point_b=(90.0, 120.0),
            speed=35.0,
            radius=8.0,
        )

        min_x, max_x = 30.0, 90.0
        min_y, max_y = 40.0, 120.0

        # Test at many fractional and large simulation timestamps
        for step in range(500):
            t = step * 0.073  # Incommensurate time step
            px, py = obs.position_at(t)
            assert min_x - 1e-6 <= px <= max_x + 1e-6
            assert min_y - 1e-6 <= py <= max_y + 1e-6

    def test_stationary_obstacle_degenerate_segment(self):
        """Obstacle with point_a == point_b remains strictly at point_a."""
        obs = DynamicObstacle(
            obstacle_id=3,
            point_a=(50.0, 50.0),
            point_b=(50.0, 50.0),
            speed=20.0,
            radius=10.0,
        )
        for t in [0.0, 1.5, 10.0, 100.0]:
            px, py = obs.position_at(t)
            assert (px, py) == (50.0, 50.0)

    def test_step_and_reset(self):
        """Advancing simulation time via step() updates current position; reset() restores t=0."""
        obs = DynamicObstacle(
            obstacle_id=4,
            point_a=(0.0, 0.0),
            point_b=(100.0, 0.0),
            speed=50.0,
            radius=5.0,
        )
        assert obs.current_position == (0.0, 0.0)

        obs.step(1.0)
        assert pytest.approx(obs.current_position[0]) == 50.0
        assert obs.current_time == 1.0

        obs.step(1.0)
        assert pytest.approx(obs.current_position[0]) == 100.0
        assert obs.current_time == 2.0

        obs.reset()
        assert obs.current_time == 0.0
        assert obs.current_position == (0.0, 0.0)

    def test_collision_with_circle_and_point(self):
        """Geometry collision checks correctly detect overlap with circular bodies."""
        obs = DynamicObstacle(
            obstacle_id=5,
            point_a=(50.0, 50.0),
            point_b=(50.0, 50.0),
            speed=0.0,
            radius=10.0,
        )
        # Point inside
        assert obs.is_point_colliding(55.0, 50.0)
        # Point outside
        assert not obs.is_point_colliding(65.0, 50.0)

        # Circle overlapping: center at (65, 50) with radius 10 -> distance 15 <= 10 + 10 = 20
        assert obs.is_circle_colliding(65.0, 50.0, other_radius=10.0)
        # Circle clear: center at (75, 50) with radius 10 -> distance 25 > 20
        assert not obs.is_circle_colliding(75.0, 50.0, other_radius=10.0)

    def test_validation_rejects_invalid_inputs(self):
        """Validation raises ValueError on non-finite, negative, or malformed definitions."""
        with pytest.raises(ValueError, match="Radius must be a positive finite number"):
            DynamicObstacle(obstacle_id=1, point_a=(0, 0), point_b=(10, 10), radius=-1.0)

        with pytest.raises(ValueError, match="Radius must be a positive finite number"):
            DynamicObstacle(obstacle_id=1, point_a=(0, 0), point_b=(10, 10), radius=float("nan"))

        with pytest.raises(ValueError, match="Speed must be a non-negative finite number"):
            DynamicObstacle(obstacle_id=1, point_a=(0, 0), point_b=(10, 10), speed=-5.0)


class TestGeometryAndRouteConflict:
    """Tests for Euclidean distance calculations and remaining-route conflict detection."""

    def test_point_to_segment_distance(self):
        """point_to_segment_distance handles perpendicular projections, collinear points, and endpoints."""
        # Segment from (0, 0) to (100, 0)
        # 1. Perpendicular projection directly above segment
        dist1 = point_to_segment_distance(50.0, 25.0, 0.0, 0.0, 100.0, 0.0)
        assert pytest.approx(dist1) == 25.0

        # 2. Point off to the left (before point 1)
        dist2 = point_to_segment_distance(-20.0, 0.0, 0.0, 0.0, 100.0, 0.0)
        assert pytest.approx(dist2) == 20.0

        # 3. Point off to the right (beyond point 2)
        dist3 = point_to_segment_distance(130.0, 40.0, 0.0, 0.0, 100.0, 0.0)
        assert pytest.approx(dist3) == 50.0  # hypot(30, 40) = 50

        # 4. Point directly on the segment
        dist4 = point_to_segment_distance(75.0, 0.0, 0.0, 0.0, 100.0, 0.0)
        assert pytest.approx(dist4) == 0.0

    def test_check_route_conflict_detected(self):
        """Route conflict is detected when obstacle buffer penetrates a route segment."""
        # Route along y = 50: [(0, 50), (100, 50), (200, 50)]
        route = [(0.0, 50.0), (100.0, 50.0), (200.0, 50.0)]

        # Dynamic obstacle at (100, 65) with radius 10. Robot radius 10. Safety margin 5.
        # Threshold: r_rob (10) + r_obs (10) + margin (5) = 25.
        # Distance from (100, 65) to segment is 15.0 <= 25 -> CONFLICT!
        obs = DynamicObstacle(
            obstacle_id=1,
            point_a=(100.0, 65.0),
            point_b=(100.0, 65.0),
            speed=0.0,
            radius=10.0,
        )

        is_conflict, conf_obs, seg_idx = check_route_conflict(
            route_points=route,
            dynamic_obstacles=[obs],
            robot_radius=10.0,
            safety_margin=5.0,
            current_time=0.0,
        )
        assert is_conflict
        assert conf_obs is obs
        assert seg_idx == 0

    def test_check_route_conflict_clear(self):
        """No conflict detected when dynamic obstacle is sufficiently separated."""
        route = [(0.0, 50.0), (100.0, 50.0), (200.0, 50.0)]

        # Distance is 40.0 > 25.0 threshold -> CLEAR!
        obs = DynamicObstacle(
            obstacle_id=1,
            point_a=(100.0, 90.0),
            point_b=(100.0, 90.0),
            speed=0.0,
            radius=10.0,
        )

        is_conflict, conf_obs, seg_idx = check_route_conflict(
            route_points=route,
            dynamic_obstacles=[obs],
            robot_radius=10.0,
            safety_margin=5.0,
            current_time=0.0,
        )
        assert not is_conflict
        assert conf_obs is None


class TestTemporaryPlanningWorld:
    """Tests verifying temporary planning grid creation and non-mutation of base world."""

    def test_temporary_planning_world_leaves_base_world_unmodified(self):
        """Temporary planning world overlays dynamic obstacles without mutating base world."""
        world = World(200.0, 200.0, 10.0, robot_radius=10.0)
        # Base world is completely empty
        initial_grid = np.copy(world.grid)
        initial_clearance = np.copy(world.clearance_grid)

        obs = DynamicObstacle(
            obstacle_id=10,
            point_a=(100.0, 100.0),
            point_b=(100.0, 100.0),
            speed=0.0,
            radius=15.0,
        )

        temp_world = create_temporary_planning_world(
            base_world=world,
            dynamic_obstacles=[obs],
            safety_margin=5.0,
            current_time=0.0,
        )

        # Base world remains completely unmutated
        np.testing.assert_array_equal(world.grid, initial_grid)
        np.testing.assert_array_equal(world.clearance_grid, initial_clearance)

        # Temporary world has obstacle cells occupied around (100, 100)
        gx, gy = temp_world.world_to_grid(100.0, 100.0)
        assert temp_world.grid[gy, gx] == CELL_OCCUPIED
        assert not temp_world.is_footprint_safe(100.0, 100.0)

    def test_find_safe_start_grid(self):
        """find_safe_start_grid maps continuous pose to nearest footprint-clear cell."""
        world = World(100.0, 100.0, 10.0, robot_radius=10.0)
        temp_world = world.clone()

        # Center at (20.0, 20.0) is safe
        safe_cell = find_safe_start_grid(world, temp_world, 20.0, 20.0, search_radius=2)
        assert safe_cell is not None
        assert temp_world.is_footprint_safe_grid(safe_cell[0], safe_cell[1])


class TestDynamicReplanningIntegration:
    """Tests for end-to-end dynamic obstacle replanning, detours, and safe stopping."""

    def test_dynamic_obstacle_replan_success_on_scenario(self):
        """Scenario dynamic_obstacle.json detects route conflict, replans detour, and reaches goal."""
        cfg_path = Path("config/scenarios/dynamic_obstacle.json")
        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        assert res.planning_success
        assert res.simulation_outcome == "REACHED_GOAL"
        assert res.motion_footprint_safe
        assert res.all_safety_checks_passed
        assert res.replans_attempted is not None and res.replans_attempted >= 1
        assert res.replans_succeeded is not None and res.replans_succeeded >= 1
        assert res.final_distance_to_goal is not None and res.final_distance_to_goal <= 5.0
        assert not res.collision_occurred

    def test_dynamic_obstacle_safe_stop_when_blocked(self):
        """Scenario dynamic_obstacle_blocked.json halts safely when no detour route exists."""
        cfg_path = Path("config/scenarios/dynamic_obstacle_blocked.json")
        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        assert res.planning_success  # Initial static path was found
        assert res.simulation_outcome == "BLOCKED"
        assert res.replans_attempted == 1
        assert res.replans_succeeded == 0
        assert not res.collision_occurred
        assert res.motion_footprint_safe
        assert res.all_safety_checks_passed
        assert "Safe stop" in res.notes

    def test_no_unnecessary_replanning_when_clear(self):
        """Simulation does not trigger replanning when dynamic obstacle stays clear of route."""
        # Create a world where obstacle patrols far in the corner
        world = World(300.0, 200.0, 10.0, robot_radius=10.0)
        obs_far = DynamicObstacle(
            obstacle_id=99,
            point_a=(250.0, 180.0),
            point_b=(280.0, 180.0),
            speed=10.0,
            radius=10.0,
        )
        world.dynamic_obstacles = [obs_far]

        # Path along lower edge from (2, 2) to (20, 2)
        start_grid = (2, 2)
        goal_grid = (20, 2)
        planner = AStarPlanner(world, connectivity=8, use_footprint=True)
        plan_res = planner.plan(start_grid, goal_grid)
        assert plan_res.success

        smoother = PathSmoother(world, check_footprint=True)
        smooth_res = smoother.smooth_result(plan_res)

        sim_metrics = run_simulation(
            world=world,
            path=smooth_res.smoothed_path,
            max_steps=500,
            dt=0.05,
            dynamic_obstacles=world.dynamic_obstacles,
            enable_replanning=True,
        )

        assert sim_metrics.goal_reached
        assert not sim_metrics.collision_occurred
        assert sim_metrics.replans_attempted == 0
        assert sim_metrics.replans_succeeded == 0

    def test_replan_cooldown_prevents_looping(self):
        """Replan cooldown window prevents replanning from triggering every frame."""
        world = World(200.0, 100.0, 10.0, robot_radius=10.0)
        # Stationary obstacle directly across path at (100, 30)
        obs = DynamicObstacle(
            obstacle_id=50,
            point_a=(100.0, 30.0),
            point_b=(100.0, 30.0),
            speed=0.0,
            radius=15.0,
        )
        world.dynamic_obstacles = [obs]

        # Path through (100, 30)
        path = [(2, 3), (10, 3), (18, 3)]

        metrics = run_simulation(
            world=world,
            path=path,
            max_steps=100,
            dt=0.05,
            dynamic_obstacles=world.dynamic_obstacles,
            enable_replanning=True,
            replan_cooldown=1.0,
        )
        # Should attempt 1 replan and stop safely
        assert metrics.replans_attempted == 1


class TestSweptFootprintDynamicCollision:
    """Tests verifying step_simulation swept-footprint checks against dynamic obstacles."""

    def test_continuous_step_rejects_dynamic_collision(self):
        """step_simulation rejects candidate pose if candidate step collides with dynamic obstacle."""
        world = World(200.0, 200.0, 10.0, robot_radius=10.0)
        obs = DynamicObstacle(
            obstacle_id=88,
            point_a=(50.0, 50.0),
            point_b=(50.0, 50.0),
            speed=0.0,
            radius=10.0,
        )

        state = RobotState(x=35.0, y=50.0, theta=0.0, v=0.0, omega=0.0)
        controller = PurePursuitController(
            world=world,
            lookahead_distance=20.0,
            robot_radius=10.0,
        )
        controller.set_path([(3, 5), (6, 5)])  # Moving forward along y=50 straight toward obstacle at (50, 50)
        model = KinematicMotionModel()

        # Step robot forward toward obstacle
        next_state, ctrl_out = step_simulation(
            robot_state=state,
            controller=controller,
            motion_model=model,
            dt=0.5,
            dynamic_obstacles=[obs],
        )

        assert ctrl_out.status == ControllerStatus.COLLISION
        assert next_state.x == 35.0  # Safe previous pose retained
        assert "dynamic obstacle" in ctrl_out.message.lower()


class TestEvaluationSuiteIntegration:
    """Tests verifying benchmark evaluation suite with dynamic obstacles."""

    def test_full_evaluation_suite_includes_dynamic_scenario(self):
        """run_evaluation_suite runs all default scenarios and records replanning metrics."""
        report = run_evaluation_suite(print_console=False)

        assert report.scenarios_evaluated == 6
        assert report.navigation_successes == 4
        assert report.expected_planning_failures == 2
        assert report.unexpected_evaluation_errors == 0
        assert report.scenarios_passed == 4
        assert report.scenarios_failed == 2

        # Check dynamic scenario specifically
        dyn_result = next(r for r in report.results if r.scenario_name == "dynamic_obstacle_detour")
        assert dyn_result.planning_success
        assert dyn_result.simulation_outcome == "REACHED_GOAL"
        assert dyn_result.replans_attempted is not None and dyn_result.replans_attempted > 0
        assert dyn_result.replans_succeeded is not None and dyn_result.replans_succeeded > 0
        assert dyn_result.dynamic_obstacles_count == 1
        assert dyn_result.all_safety_checks_passed


class TestDynamicSafetyAudit:
    """Targeted regression and invariant tests for Task 09.1 Dynamic Safety Audit."""

    def test_moving_obstacle_crossing_collision_detected(self):
        """
        Regression test for between-update crossing collision gap:
        Robot moves horizontally from (40, 50) to (60, 50) in interval [0, 1].
        Obstacle moves vertically from (50, 38) to (50, 62) in interval [0, 1].
        Both bodies have radius 5.0 (r_total = 10.0).

        At t=0: dist = hypot(10, 12) = 15.62 > 10.0 (endpoints clear).
        At t=1: dist = hypot(10, 12) = 15.62 > 10.0 (endpoints clear).
        Static swept distance to obstacle at t=0 or t=1 is 12.0 > 10.0.
        BUT at t=0.5: both are at (50, 50), dist = 0.0 <= 10.0 (direct crossing collision).

        Synchronized relative continuous motion check MUST detect this collision!
        """
        obs = DynamicObstacle(
            obstacle_id=99,
            point_a=(50.0, 38.0),
            point_b=(50.0, 62.0),
            speed=24.0,  # 24 world units in 1 second
            radius=5.0,
        )

        # 1. Test relative continuous motion checker directly
        collided, min_dist, reason = check_moving_obstacle_step_collision(
            robot_start=(40.0, 50.0),
            robot_end=(60.0, 50.0),
            robot_radius=5.0,
            obstacle=obs,
            t_start=0.0,
            t_end=1.0,
        )
        assert collided is True
        assert min_dist <= 1e-6
        assert "collides with moving dynamic obstacle" in reason.lower()

        # 2. Test step_simulation integration rejects the candidate step
        world = World(200.0, 200.0, 10.0, robot_radius=5.0)
        state = RobotState(x=40.0, y=50.0, theta=0.0, v=20.0, omega=0.0)
        controller = PurePursuitController(
            world=world,
            lookahead_distance=25.0,
            robot_radius=5.0,
        )
        controller.set_path([(4, 5), (6, 5)])
        model = KinematicMotionModel(max_linear_speed=20.0)

        next_state, ctrl_out = step_simulation(
            robot_state=state,
            controller=controller,
            motion_model=model,
            dt=1.0,
            dynamic_obstacles=[obs],
            current_time=0.0,
        )

        assert ctrl_out.status == ControllerStatus.COLLISION
        assert next_state.x == 40.0
        assert next_state.v == 0.0
        assert "collision rejected" in ctrl_out.message.lower()

    def test_boundary_and_tangency_rules(self):
        """
        Boundary and tangency rules match static swept-footprint checker:
        Distance <= (robot_radius + obstacle_radius + margin) is a collision.
        Distance > (robot_radius + obstacle_radius + margin) is clear.
        """
        obs = DynamicObstacle(
            obstacle_id=101,
            point_a=(50.0, 50.0),
            point_b=(50.0, 50.0),
            speed=0.0,
            radius=10.0,
        )
        robot_radius = 5.0
        margin = 2.0
        r_total = 17.0

        # Exact boundary tangency (dist == 17.0)
        collided, dist, _ = check_moving_obstacle_step_collision(
            robot_start=(50.0 + r_total, 50.0),
            robot_end=(50.0 + r_total, 50.0),
            robot_radius=robot_radius,
            obstacle=obs,
            t_start=0.0,
            t_end=0.1,
            margin=margin,
        )
        assert collided is True
        assert pytest.approx(dist, abs=1e-5) == r_total

        # Strictly outside boundary by epsilon (dist == 17.0 + 0.01)
        collided, dist, _ = check_moving_obstacle_step_collision(
            robot_start=(50.0 + r_total + 0.01, 50.0),
            robot_end=(50.0 + r_total + 0.01, 50.0),
            robot_radius=robot_radius,
            obstacle=obs,
            t_start=0.0,
            t_end=0.1,
            margin=margin,
        )
        assert collided is False
        assert pytest.approx(dist, abs=1e-5) == r_total + 0.01

    def test_simulation_clock_advancement_per_step(self):
        """
        Dynamic obstacle clock advances monotonically and deterministically by dt per step.
        """
        world = World(200.0, 100.0, 10.0, robot_radius=5.0)
        obs = DynamicObstacle(
            obstacle_id=102,
            point_a=(10.0, 50.0),
            point_b=(90.0, 50.0),
            speed=10.0,
            radius=5.0,
        )
        world.dynamic_obstacles = [obs]
        path = [(2, 2), (4, 2)]

        dt = 0.05
        metrics = run_simulation(
            world=world,
            path=path,
            max_steps=5,
            dt=dt,
            dynamic_obstacles=world.dynamic_obstacles,
            enable_replanning=False,
        )

        assert metrics.steps == 5
        assert pytest.approx(obs.current_time, abs=1e-5) == 5 * dt

    def test_max_replans_exhaustion_triggers_safe_stop(self):
        """
        When max_replans budget is reached while route remains in conflict,
        robot halts cleanly with ControllerStatus.BLOCKED and zero velocities.
        """
        world = World(200.0, 100.0, 10.0, robot_radius=10.0)
        # Block route permanently
        obs = DynamicObstacle(
            obstacle_id=103,
            point_a=(50.0, 30.0),
            point_b=(50.0, 30.0),
            speed=0.0,
            radius=20.0,
        )
        world.dynamic_obstacles = [obs]
        path = [(1, 3), (10, 3), (18, 3)]

        metrics = run_simulation(
            world=world,
            path=path,
            max_steps=100,
            dt=0.05,
            dynamic_obstacles=world.dynamic_obstacles,
            enable_replanning=True,
            replan_cooldown=0.05,
            max_replans=2,
        )

        assert metrics.status == ControllerStatus.BLOCKED
        assert metrics.goal_reached is False
        assert metrics.collision_occurred is False
        assert metrics.replans_attempted == 2
        assert "replanning limit reached" in metrics.message.lower() or "safe stop" in metrics.message.lower()

    def test_temporary_planning_world_immutability(self):
        """
        Creating a temporary planning world does not mutate the static base world,
        and mutating the temporary world does not affect the base world.
        """
        world = World(100.0, 100.0, 10.0, robot_radius=5.0)
        base_grid_before = np.copy(world.grid)
        base_clearance_before = np.copy(world.clearance_grid)

        dyn = DynamicObstacle(
            obstacle_id=104,
            point_a=(50.0, 50.0),
            point_b=(50.0, 50.0),
            speed=0.0,
            radius=10.0,
        )

        temp_world = create_temporary_planning_world(world, [dyn], safety_margin=5.0)

        # Base world remains unchanged
        assert np.array_equal(world.grid, base_grid_before)
        assert np.array_equal(world.clearance_grid, base_clearance_before)

        # Modifying temp_world does not mutate base world
        temp_world.grid[0, 0] = CELL_OCCUPIED
        assert world.grid[0, 0] == CELL_FREE

    def test_dynamic_clearance_expansion_no_double_counting(self):
        """
        Verify dynamic obstacle expansion in temporary world:
        effective_radius = dyn.radius + safety_margin.
        Clearance grid is inflated by robot_radius.
        Total inflated clearance zone = dyn.radius + safety_margin + robot_radius.
        """
        world = World(200.0, 200.0, 5.0, robot_radius=5.0)
        dyn = DynamicObstacle(
            obstacle_id=105,
            point_a=(100.0, 100.0),
            point_b=(100.0, 100.0),
            speed=0.0,
            radius=10.0,
        )
        safety_margin = 5.0
        # Total clearance around (100, 100) should be 10 (radius) + 5 (margin) + 5 (robot_radius) = 20.0
        temp_world = create_temporary_planning_world(world, [dyn], safety_margin=safety_margin)

        # Point at distance 15.0 from center is occupied by the rasterized temporary obstacle
        gx_inner, gy_inner = temp_world.world_to_grid(114.0, 100.0)
        assert temp_world.is_occupied(gx_inner, gy_inner)

        # Point at distance 18.0 is free of obstacle cells but within robot_radius of the effective obstacle
        gx_zone, gy_zone = temp_world.world_to_grid(118.0, 100.0)
        assert not temp_world.is_occupied(gx_zone, gy_zone)
        assert not temp_world.is_footprint_safe_grid(gx_zone, gy_zone)

        # Point at distance 25.0 is safely outside the total clearance zone
        gx_outer, gy_outer = temp_world.world_to_grid(125.0, 100.0)
        assert not temp_world.is_occupied(gx_outer, gy_outer)
        assert temp_world.is_footprint_safe_grid(gx_outer, gy_outer)
