"""
Unit and regression tests for reproducible evaluation and benchmarking framework.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest

from autonomous_nav.evaluation import (
    DEFAULT_SCENARIOS,
    SAFETY_DISCLAIMER,
    ScenarioEvaluationResult,
    EvaluationSuiteReport,
    compute_path_length_world,
    evaluate_scenario,
    run_evaluation_suite,
)
from autonomous_nav.main import parse_args, run
from autonomous_nav.world import World


class TestWorldPathLengthComputation(unittest.TestCase):
    """Test path length computation in continuous world coordinates."""

    def setUp(self) -> None:
        self.world = World(width=200.0, height=200.0, grid_resolution=10.0, robot_radius=10.0)

    def test_empty_and_single_waypoint_paths(self) -> None:
        """Paths with 0 or 1 point have zero length."""
        self.assertEqual(compute_path_length_world(self.world, []), 0.0)
        self.assertEqual(compute_path_length_world(self.world, [(5, 5)]), 0.0)

    def test_straight_horizontal_and_vertical_lengths(self) -> None:
        """Horizontal and vertical segments compute exact world distances."""
        # Horizontal: (2, 2) -> (6, 2) is 4 grid steps * 10.0 resolution = 40.0 world units
        path_h = [(2, 2), (3, 2), (4, 2), (5, 2), (6, 2)]
        self.assertAlmostEqual(compute_path_length_world(self.world, path_h), 40.0, places=5)

        # Shortcut directly from (2, 2) to (6, 2) has identical length
        self.assertAlmostEqual(compute_path_length_world(self.world, [(2, 2), (6, 2)]), 40.0, places=5)

    def test_diagonal_path_length(self) -> None:
        """Diagonal segments compute exact Euclidean distances."""
        # (0, 0) -> (3, 3) is 3 * sqrt(2) * 10.0 = 42.4264 world units
        path_diag = [(0, 0), (1, 1), (2, 2), (3, 3)]
        expected = 3 * math.sqrt(2) * 10.0
        self.assertAlmostEqual(compute_path_length_world(self.world, path_diag), expected, places=4)


class TestScenarioEvaluationReachable(unittest.TestCase):
    """Test evaluation on valid reachable navigation scenarios."""

    def test_open_world_evaluation(self) -> None:
        """Open world scenario finds path, simplifies route, and reaches goal."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "scenarios" / "open_world.json"
        self.assertTrue(cfg_path.is_file())

        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        self.assertTrue(res.planning_success)
        self.assertGreater(res.expanded_nodes, 0)
        self.assertGreater(res.planning_time_ms, 0.0)
        self.assertIsNotNone(res.original_waypoints)
        self.assertIsNotNone(res.smoothed_waypoints)
        self.assertLessEqual(res.smoothed_waypoints, res.original_waypoints)
        self.assertEqual(res.smoothed_waypoints, 2)  # Straight line shortcuts to 2 endpoints
        self.assertGreater(res.waypoint_reduction_pct, 50.0)
        self.assertIsNotNone(res.original_path_length_world)
        self.assertIsNotNone(res.smoothed_path_length_world)
        self.assertAlmostEqual(res.original_path_length_world, 197.9899, places=2)
        self.assertTrue(res.path_footprint_safe)
        self.assertEqual(res.simulation_outcome, "REACHED_GOAL")
        self.assertIsNotNone(res.simulation_steps)
        self.assertIsNotNone(res.simulated_time_s)
        self.assertIsNotNone(res.actual_distance_traveled_world)
        self.assertIsNotNone(res.final_distance_to_goal)
        self.assertLessEqual(res.final_distance_to_goal, 5.0)
        self.assertTrue(res.motion_footprint_safe)
        self.assertTrue(res.all_safety_checks_passed)

    def test_obstacle_detour_evaluation(self) -> None:
        """Detour scenario plans around the wall and reaches goal safely."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "scenarios" / "obstacle_detour.json"
        self.assertTrue(cfg_path.is_file())

        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        self.assertTrue(res.planning_success)
        self.assertGreater(res.expanded_nodes, 0)
        self.assertIsNotNone(res.smoothed_waypoints)
        self.assertLess(res.smoothed_waypoints, res.original_waypoints)
        self.assertEqual(res.simulation_outcome, "REACHED_GOAL")
        self.assertTrue(res.path_footprint_safe)
        self.assertTrue(res.motion_footprint_safe)
        self.assertTrue(res.all_safety_checks_passed)
        self.assertLessEqual(res.final_distance_to_goal, 5.0)


class TestScenarioEvaluationUnreachable(unittest.TestCase):
    """Test evaluation on deliberately blocked and unreachable scenarios."""

    def test_blocked_unreachable_scenario(self) -> None:
        """Blocked wall scenario records planning failure cleanly without crashing."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "scenarios" / "blocked_unreachable.json"
        self.assertTrue(cfg_path.is_file())

        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        self.assertFalse(res.planning_success)
        self.assertGreater(res.expanded_nodes, 0)
        self.assertGreaterEqual(res.planning_time_ms, 0.0)
        self.assertIsNone(res.original_waypoints)
        self.assertIsNone(res.smoothed_waypoints)
        self.assertIsNone(res.waypoint_reduction_pct)
        self.assertIsNone(res.original_path_length_world)
        self.assertIsNone(res.smoothed_path_length_world)
        self.assertIsNone(res.path_footprint_safe)
        self.assertEqual(res.simulation_outcome, "NOT_RUN")
        self.assertIsNone(res.simulation_steps)
        self.assertIsNone(res.simulated_time_s)
        self.assertIsNone(res.actual_distance_traveled_world)
        self.assertIsNone(res.final_distance_to_goal)
        self.assertIsNone(res.motion_footprint_safe)
        self.assertIsNone(res.all_safety_checks_passed)
        self.assertIn("No", res.notes)

    def test_insufficient_clearance_scenario(self) -> None:
        """Bottleneck too narrow for configured footprint fails footprint planning."""
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "scenarios" / "insufficient_clearance.json"
        self.assertTrue(cfg_path.is_file())

        # Under footprint-aware mode (radius=15.0, corridor width=20.0), planning fails
        res_footprint = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)
        self.assertFalse(res_footprint.planning_success)
        self.assertIsNone(res_footprint.original_waypoints)
        self.assertEqual(res_footprint.simulation_outcome, "NOT_RUN")

        # If footprint clearance is bypassed with zero radius, path is traversable
        res_zero_radius = evaluate_scenario(
            cfg_path, connectivity=8, check_footprint=False, radius_override=0.0
        )
        self.assertTrue(res_zero_radius.planning_success)
        self.assertIsNotNone(res_zero_radius.original_waypoints)


class TestSuiteExecutionAndFailureContinuity(unittest.TestCase):
    """Test evaluation suite execution across multiple scenarios and error resilience."""

    def test_continued_evaluation_after_planning_failure(self) -> None:
        """Suite continues evaluating subsequent scenarios when an earlier scenario fails."""
        scenarios = [
            Path("config/scenarios/blocked_unreachable.json"),
            Path("config/scenarios/open_world.json"),
        ]
        with tempfile.TemporaryDirectory() as tmp_dir:
            json_out = Path(tmp_dir) / "test_report.json"
            report = run_evaluation_suite(
                scenario_paths=scenarios,
                output_json_path=json_out,
                print_console=False,
            )

            self.assertEqual(report.scenarios_evaluated, 2)
            self.assertEqual(report.scenarios_passed, 1)
            self.assertEqual(report.scenarios_failed, 1)
            self.assertEqual(len(report.results), 2)

            # First scenario failed
            self.assertFalse(report.results[0].planning_success)
            self.assertEqual(report.results[0].scenario_name, "blocked_unreachable")

            # Second scenario evaluated successfully despite first failure
            self.assertTrue(report.results[1].planning_success)
            self.assertEqual(report.results[1].scenario_name, "open_world_straight")

    def test_json_serialization_and_report_schema(self) -> None:
        """Report serializes to standard JSON containing disclaimer and required metrics."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            json_out = Path(tmp_dir) / "benchmark_report.json"
            report = run_evaluation_suite(
                scenario_paths=[Path("config/scenarios/open_world.json")],
                output_json_path=json_out,
                print_console=False,
            )

            self.assertTrue(json_out.is_file())
            with json_out.open("r", encoding="utf-8") as f:
                data = json.load(f)

            self.assertIn("timestamp", data)
            self.assertIn("disclaimer", data)
            self.assertEqual(data["disclaimer"], SAFETY_DISCLAIMER)
            self.assertEqual(data["scenarios_evaluated"], 1)
            self.assertEqual(data["scenarios_passed"], 1)
            self.assertEqual(data["scenarios_failed"], 0)
            self.assertEqual(len(data["scenarios"]), 1)

            scen = data["scenarios"][0]
            self.assertEqual(scen["scenario_name"], "open_world_straight")
            self.assertIn("robot_radius", scen)
            self.assertIn("world_dimensions", scen)
            self.assertIn("original_path_length_world", scen)
            self.assertIn("actual_distance_traveled_world", scen)
            self.assertIn("simulated_time_s", scen)
            self.assertIn("simulation_wall_time_ms", scen)
            self.assertIn("all_safety_checks_passed", scen)
            self.assertIn("raw_path_footprint_safe", scen)
            self.assertIn("smoothed_path_footprint_safe", scen)
            self.assertEqual(scen["raw_path_footprint_safe"], True)
            self.assertEqual(scen["smoothed_path_footprint_safe"], True)
            self.assertEqual(scen["evaluation_category"], "NAVIGATION_SUCCESS")

            # Check new 4-way count breakdowns in JSON root
            self.assertEqual(data["navigation_successes"], 1)
            self.assertEqual(data["expected_planning_failures"], 0)
            self.assertEqual(data["unexpected_evaluation_errors"], 0)


class TestFootprintSafetyAudit(unittest.TestCase):
    """Audit tests ensuring consistent swept-footprint checking across raw and smoothed routes."""

    def test_raw_and_smoothed_swept_footprint_consistency(self) -> None:
        """Both raw path segments and smoothed shortcut segments pass swept footprint checks."""
        cfg_path = Path("config/scenarios/obstacle_detour.json")
        res = evaluate_scenario(cfg_path, connectivity=8, check_footprint=True)

        self.assertTrue(res.planning_success)
        self.assertTrue(res.raw_path_footprint_safe)
        self.assertTrue(res.smoothed_path_footprint_safe)
        self.assertTrue(res.path_footprint_safe)
        self.assertTrue(res.motion_footprint_safe)
        self.assertTrue(res.all_safety_checks_passed)
        self.assertEqual(res.evaluation_category, "NAVIGATION_SUCCESS")

    def test_swept_footprint_detects_segment_piercing_obstacle(self) -> None:
        """is_swept_footprint_safe rejects straight moves connecting endpoints that span an obstacle."""
        world = World(200.0, 200.0, 10.0, robot_radius=5.0)
        world.add_obstacle_cell(10, 10)  # Obstacle at [100, 110] x [100, 110]

        # Endpoints (80, 105) and (130, 105) are both free and safe
        p1 = (80.0, 105.0)
        p2 = (130.0, 105.0)
        self.assertTrue(world.is_footprint_safe(p1[0], p1[1], radius=5.0))
        self.assertTrue(world.is_footprint_safe(p2[0], p2[1], radius=5.0))

        # Connecting segment traverses the obstacle and must be rejected
        self.assertFalse(world.is_swept_footprint_safe(p1[0], p1[1], p2[0], p2[1], radius=5.0))


class TestEvaluationCategoryAndErrorSeparation(unittest.TestCase):
    """Test separation of navigation successes, expected failures, and unexpected errors."""

    def test_distinct_counts_and_no_error_swallowing(self) -> None:
        """Suite separates successes, expected failures, and config load errors."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            # 1. Reachable scenario
            scen_ok = Path("config/scenarios/open_world.json")
            # 2. Blocked scenario (expected planning failure)
            scen_blocked = Path("config/scenarios/blocked_unreachable.json")
            # 3. Non-existent file (unexpected evaluation error)
            scen_missing = Path(tmp_dir) / "non_existent.json"

            report = run_evaluation_suite(
                scenario_paths=[scen_ok, scen_blocked, scen_missing],
                output_json_path=Path(tmp_dir) / "report.json",
                print_console=False,
            )

            self.assertEqual(report.scenarios_evaluated, 3)
            self.assertEqual(report.navigation_successes, 1)
            self.assertEqual(report.expected_planning_failures, 1)
            self.assertEqual(report.unexpected_evaluation_errors, 1)

            # Backward compatibility aliases
            self.assertEqual(report.scenarios_passed, 1)
            self.assertEqual(report.scenarios_failed, 2)

            self.assertEqual(report.results[0].evaluation_category, "NAVIGATION_SUCCESS")
            self.assertEqual(report.results[1].evaluation_category, "EXPECTED_FAILURE")
            self.assertEqual(report.results[2].evaluation_category, "UNEXPECTED_ERROR")

    def test_unexpected_programming_errors_are_not_swallowed(self) -> None:
        """Internal programming bugs (e.g. AttributeError, TypeError) raise directly."""
        # Pass an object that causes an AttributeError when treated as Path
        class BuggyScenarioPath:
            def __str__(self):
                raise TypeError("Simulated programming error inside runner")

        with self.assertRaises(TypeError):
            run_evaluation_suite(
                scenario_paths=[BuggyScenarioPath()],  # type: ignore
                print_console=False,
            )


class TestCLIEvaluationIntegration(unittest.TestCase):
    """Test CLI argument parsing and execution with --evaluate."""

    def test_parse_args_evaluate_flags(self) -> None:
        """CLI arguments --evaluate and --eval-output are parsed correctly."""
        args_default = parse_args([])
        self.assertFalse(args_default.evaluate)
        self.assertEqual(args_default.eval_output, "results/evaluation_report.json")

        args_eval = parse_args(["--evaluate", "--eval-output", "custom_results/report.json"])
        self.assertTrue(args_eval.evaluate)
        self.assertEqual(args_eval.eval_output, "custom_results/report.json")

    def test_run_headless_evaluation_via_main(self) -> None:
        """Calling run() with evaluate=True exits cleanly and writes report."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            json_out = Path(tmp_dir) / "main_eval.json"
            open_cfg = str(Path("config/scenarios/open_world.json"))
            ret = run(
                config_path=open_cfg,
                evaluate=True,
                eval_output=str(json_out),
            )
            self.assertEqual(ret, 0)
            self.assertTrue(json_out.is_file())


if __name__ == "__main__":
    unittest.main()
