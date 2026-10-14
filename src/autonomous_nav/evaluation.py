"""
Reproducible Navigation Evaluation and Benchmarking Framework.

Provides automated, headless evaluation of navigation scenarios across planning,
smoothing, continuous robot motion, safety clearance, and execution metrics.
Produces structured console tables and machine-readable JSON reports.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from autonomous_nav.config_loader import load_config
from autonomous_nav.controller import ControllerStatus, run_simulation
from autonomous_nav.path_smoother import PathSmoother, has_line_of_sight
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.world import World


DEFAULT_SCENARIOS = [
    Path("config/default_scenario.json"),
    Path("config/scenarios/open_world.json"),
    Path("config/scenarios/obstacle_detour.json"),
    Path("config/scenarios/blocked_unreachable.json"),
    Path("config/scenarios/insufficient_clearance.json"),
    Path("config/scenarios/dynamic_obstacle.json"),
]

SAFETY_DISCLAIMER = (
    "Tested runs verify collision-freedom under deterministic simulation assumptions "
    "(kinematic unicycle model, static discrete obstacles, swept footprint checking). "
    "Zero collisions in a tested run does not constitute an unconstrained universal safety "
    "guarantee across dynamic environments, sensor latency, or unmodeled physics."
)


def compute_path_length_world(world: World, path: Sequence[Tuple[int, int]]) -> float:
    """
    Calculate the cumulative Euclidean length of a discrete grid waypoint sequence
    evaluated in continuous world coordinates.

    Args:
        world: World environment containing grid resolution and coordinate mappings.
        path: Ordered list of discrete grid waypoints [(gx, gy), ...].

    Returns:
        Cumulative path length in continuous world units. Returns 0.0 if path has < 2 points.
    """
    if not path or len(path) < 2:
        return 0.0

    total_len = 0.0
    for i in range(len(path) - 1):
        x0, y0 = world.grid_to_world(*path[i])
        x1, y1 = world.grid_to_world(*path[i + 1])
        total_len += math.hypot(x1 - x0, y1 - y0)
    return total_len


@dataclass
class ScenarioEvaluationResult:
    """
    Detailed evaluation metrics and execution telemetry for a single navigation scenario.

    Attributes:
        scenario_name: Stable identifier of the scenario.
        config_path: Path to scenario JSON configuration.
        robot_radius: Configured robot circular footprint radius (world units).
        world_dimensions: Tuple of (width, height) in world units.
        grid_resolution: Cell dimension in world units / cell.
        planning_success: True if A* found a collision-safe path, False otherwise.
        planning_time_ms: Actual wall-clock time spent in A* search (milliseconds).
        expanded_nodes: Count of nodes expanded during A* search.
        original_waypoints: Count of discrete waypoints in raw A* path, or None if unreached.
        smoothed_waypoints: Count of waypoints after line-of-sight smoothing, or None.
        waypoint_reduction_pct: Percentage reduction in waypoint count (orig - smooth) / orig * 100.
        original_path_length_world: Planned raw path length in world units.
        smoothed_path_length_world: Planned smoothed path length in world units.
        length_reduction_pct: Percentage reduction in path Euclidean length.
        path_footprint_safe: True if all path cells and smoothed segments are footprint-clear.
        simulation_outcome: Outcome string ('REACHED_GOAL', 'COLLISION', 'STEP_LIMIT', 'NOT_RUN').
        simulation_steps: Discrete simulation integration steps executed.
        simulated_time_s: Virtual physics time elapsed in simulation (steps * dt, seconds).
        simulation_wall_time_ms: Actual CPU wall-clock time spent running simulation (ms).
        actual_distance_traveled_world: Actual Euclidean distance traversed by robot center.
        final_distance_to_goal: Euclidean distance from final robot position to target (world units).
        motion_footprint_safe: True if continuous motion executed with zero collision violations.
        all_safety_checks_passed: True if both path and motion passed all footprint checks.
        notes: Informative diagnostics or failure explanation.
    """

    scenario_name: str
    config_path: str
    robot_radius: float
    world_dimensions: Tuple[float, float]
    grid_resolution: float
    planning_success: bool
    planning_time_ms: float
    expanded_nodes: int
    original_waypoints: Optional[int] = None
    smoothed_waypoints: Optional[int] = None
    waypoint_reduction_pct: Optional[float] = None
    original_path_length_world: Optional[float] = None
    smoothed_path_length_world: Optional[float] = None
    length_reduction_pct: Optional[float] = None
    path_footprint_safe: Optional[bool] = None
    raw_path_footprint_safe: Optional[bool] = None
    smoothed_path_footprint_safe: Optional[bool] = None
    simulation_outcome: Optional[str] = "NOT_RUN"
    simulation_steps: Optional[int] = None
    simulated_time_s: Optional[float] = None
    simulation_wall_time_ms: Optional[float] = None
    actual_distance_traveled_world: Optional[float] = None
    final_distance_to_goal: Optional[float] = None
    motion_footprint_safe: Optional[bool] = None
    all_safety_checks_passed: Optional[bool] = None
    replans_attempted: Optional[int] = None
    replans_succeeded: Optional[int] = None
    dynamic_obstacles_count: Optional[int] = None
    collision_occurred: Optional[bool] = None
    evaluation_category: str = "NAVIGATION_SUCCESS"
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """Convert result dataclass to a clean, JSON-serializable dictionary."""
        d = asdict(self)
        d["world_dimensions"] = list(self.world_dimensions)
        for key in (
            "planning_time_ms",
            "simulation_wall_time_ms",
            "waypoint_reduction_pct",
            "original_path_length_world",
            "smoothed_path_length_world",
            "length_reduction_pct",
            "simulated_time_s",
            "actual_distance_traveled_world",
            "final_distance_to_goal",
        ):
            val = d.get(key)
            if val is not None and isinstance(val, (int, float)):
                d[key] = round(float(val), 4)
        return d


@dataclass
class EvaluationSuiteReport:
    """
    Comprehensive benchmark report aggregating multiple scenario evaluations.
    """

    timestamp: str
    disclaimer: str
    scenarios_evaluated: int
    navigation_successes: int
    expected_planning_failures: int
    unexpected_evaluation_errors: int
    scenarios_passed: int
    scenarios_failed: int
    results: List[ScenarioEvaluationResult] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize full suite report to JSON-compatible dictionary."""
        return {
            "timestamp": self.timestamp,
            "disclaimer": self.disclaimer,
            "scenarios_evaluated": self.scenarios_evaluated,
            "navigation_successes": self.navigation_successes,
            "expected_planning_failures": self.expected_planning_failures,
            "unexpected_evaluation_errors": self.unexpected_evaluation_errors,
            "scenarios_passed": self.scenarios_passed,
            "scenarios_failed": self.scenarios_failed,
            "scenarios": [r.to_dict() for r in self.results],
        }

    def save_json(self, output_path: str | Path) -> Path:
        """Write the suite report to a formatted JSON file."""
        out = Path(output_path).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        return out

    def format_summary_table(self) -> str:
        """Render a readable ASCII summary table of evaluation metrics."""
        headers = [
            "Scenario Name",
            "Plan",
            "Nodes",
            "Plan Time",
            "Waypoints",
            "Planned Len",
            "Traveled Len",
            "Sim Time",
            "Replans",
            "Outcome",
            "Safety",
        ]
        rows: List[List[str]] = []

        for r in self.results:
            plan_str = "PASS" if r.planning_success else "FAIL"
            nodes_str = str(r.expanded_nodes)
            time_str = f"{r.planning_time_ms:.2f} ms"

            if r.planning_success and r.original_waypoints is not None and r.smoothed_waypoints is not None:
                red_pct = f"{r.waypoint_reduction_pct:.1f}%" if r.waypoint_reduction_pct is not None else "0%"
                wpt_str = f"{r.original_waypoints}->{r.smoothed_waypoints} (-{red_pct})"
            else:
                wpt_str = "N/A"

            if r.smoothed_path_length_world is not None:
                plan_len_str = f"{r.smoothed_path_length_world:.1f} u"
            elif r.original_path_length_world is not None:
                plan_len_str = f"{r.original_path_length_world:.1f} u"
            else:
                plan_len_str = "N/A"

            if r.actual_distance_traveled_world is not None:
                trav_len_str = f"{r.actual_distance_traveled_world:.1f} u"
            else:
                trav_len_str = "N/A"

            if r.simulated_time_s is not None:
                sim_time_str = f"{r.simulated_time_s:.2f} s"
            else:
                sim_time_str = "N/A"

            if r.replans_attempted is not None and r.replans_attempted > 0:
                replan_str = f"{r.replans_succeeded}/{r.replans_attempted}"
            elif r.dynamic_obstacles_count is not None and r.dynamic_obstacles_count > 0:
                replan_str = "0/0"
            else:
                replan_str = "N/A"

            outcome_str = r.simulation_outcome or "NOT_RUN"

            if r.all_safety_checks_passed is True:
                safety_str = "SAFE"
            elif r.all_safety_checks_passed is False:
                safety_str = "VIOLATION"
            else:
                safety_str = "N/A"

            rows.append([
                r.scenario_name[:24],
                plan_str,
                nodes_str,
                time_str,
                wpt_str,
                plan_len_str,
                trav_len_str,
                sim_time_str,
                replan_str,
                outcome_str,
                safety_str,
            ])

        # Compute column widths
        col_widths = [len(h) for h in headers]
        for row in rows:
            for i, cell in enumerate(row):
                col_widths[i] = max(col_widths[i], len(cell))

        def make_row(cells: List[str]) -> str:
            return "| " + " | ".join(c.ljust(col_widths[i]) for i, c in enumerate(cells)) + " |"

        sep = "+-" + "-+-".join("-" * w for w in col_widths) + "-+"

        table_lines = [
            sep,
            make_row(headers),
            sep,
        ]
        for row in rows:
            table_lines.append(make_row(row))
        table_lines.append(sep)

        return "\n".join(table_lines)


def evaluate_scenario(
    config_source: str | Path | Dict[str, Any],
    connectivity: int = 8,
    check_footprint: bool = True,
    radius_override: Optional[float] = None,
    max_simulation_steps: int = 2000,
    simulation_dt: float = 0.05,
) -> ScenarioEvaluationResult:
    """
    Execute repeatable evaluation on an individual scenario.

    Runs footprint-aware A* planning, line-of-sight shortcut smoothing, and continuous
    kinematic simulation. Calculates all comparative planning, motion, and safety metrics.

    Args:
        config_source: File path (str or Path) or pre-loaded dictionary of scenario config.
        connectivity: A* connectivity (4 or 8, default: 8).
        check_footprint: Enforce circular footprint clearance throughout all pipeline stages.
        radius_override: Optional override for robot radius in world units.
        max_simulation_steps: Step limit for continuous motion simulation.
        simulation_dt: Integration timestep in seconds.

    Returns:
        ScenarioEvaluationResult containing comprehensive benchmark telemetry.
    """
    if isinstance(config_source, (str, Path)):
        cfg_path_str = str(config_source)
        config_data = load_config(config_source)
    else:
        cfg_path_str = "<in-memory-dict>"
        config_data = config_source

    scenario_name = config_data.get("scenario_name", Path(cfg_path_str).stem)
    world = World.from_config(config_data)
    if radius_override is not None:
        world.robot_radius = radius_override

    robot_cfg = config_data.get("robot", {})
    start_pos = tuple(robot_cfg.get("start_position", [0.0, 0.0]))
    target_pos = tuple(robot_cfg.get("target_position", [0.0, 0.0]))

    # Validate start and target positions
    try:
        start_grid, goal_grid = world.validate_start_goal(
            start_pos, target_pos, check_footprint=check_footprint
        )
    except ValueError as err:
        return ScenarioEvaluationResult(
            scenario_name=scenario_name,
            config_path=cfg_path_str,
            robot_radius=world.robot_radius,
            world_dimensions=(world.width, world.height),
            grid_resolution=world.grid_resolution,
            planning_success=False,
            planning_time_ms=0.0,
            expanded_nodes=0,
            simulation_outcome="NOT_RUN",
            all_safety_checks_passed=False,
            replans_attempted=0,
            replans_succeeded=0,
            dynamic_obstacles_count=len(world.dynamic_obstacles),
            collision_occurred=False,
            evaluation_category="EXPECTED_FAILURE",
            notes=f"Start or goal validation rejected: {err}",
        )

    # 1. A* Path Planning
    t_plan_start = time.perf_counter()
    planner = AStarPlanner(
        world,
        connectivity=connectivity,
        strict_diagonal_clearance=True,
        use_footprint=check_footprint,
    )
    plan_result = planner.plan(start_grid, goal_grid)
    planning_time_ms = (time.perf_counter() - t_plan_start) * 1000.0

    if not plan_result.success or not plan_result.path:
        return ScenarioEvaluationResult(
            scenario_name=scenario_name,
            config_path=cfg_path_str,
            robot_radius=world.robot_radius,
            world_dimensions=(world.width, world.height),
            grid_resolution=world.grid_resolution,
            planning_success=False,
            planning_time_ms=planning_time_ms,
            expanded_nodes=plan_result.expanded_nodes,
            simulation_outcome="NOT_RUN",
            all_safety_checks_passed=None,
            replans_attempted=0,
            replans_succeeded=0,
            dynamic_obstacles_count=len(world.dynamic_obstacles),
            collision_occurred=False,
            evaluation_category="EXPECTED_FAILURE",
            notes=f"Planning failed: {plan_result.message or 'No path found'}",
        )

    # 2. Path Safety Verification & World Path Lengths
    raw_path = plan_result.path
    raw_wpts = len(raw_path)
    raw_path_len_world = compute_path_length_world(world, raw_path)
    raw_cells_safe = all(world.is_footprint_safe_grid(x, y) for x, y in raw_path)
    raw_segments_safe = (
        all(
            world.is_swept_footprint_safe(
                *world.grid_to_world(*p1),
                *world.grid_to_world(*p2),
                radius=world.robot_radius,
            )
            for p1, p2 in zip(raw_path[:-1], raw_path[1:])
        )
        if (check_footprint and len(raw_path) >= 2)
        else True
    )
    raw_path_footprint_safe = raw_cells_safe and raw_segments_safe

    # 3. Path Smoothing
    smoother = PathSmoother(world, check_footprint=check_footprint)
    smoothed_result = smoother.smooth_result(plan_result)

    smooth_wpts = smoothed_result.smoothed_waypoints
    smoothed_path = smoothed_result.smoothed_path
    smoothed_path_len_world = compute_path_length_world(world, smoothed_path)

    wpt_reduction_pct = (
        ((raw_wpts - smooth_wpts) / raw_wpts * 100.0) if raw_wpts > 0 else 0.0
    )
    len_reduction_pct = (
        ((raw_path_len_world - smoothed_path_len_world) / raw_path_len_world * 100.0)
        if raw_path_len_world > 0.0
        else 0.0
    )

    # Validate all smoothed segments against footprint clearance
    smoothed_segments_safe = (
        all(
            has_line_of_sight(world, p1, p2, check_footprint=check_footprint)
            for p1, p2 in zip(smoothed_path[:-1], smoothed_path[1:])
        )
        if len(smoothed_path) >= 2
        else True
    )
    smoothed_path_footprint_safe = smoothed_segments_safe
    path_footprint_safe = raw_path_footprint_safe and smoothed_path_footprint_safe

    # 4. Continuous Kinematic Motion Simulation
    sim_cfg = config_data.get("simulation", {})
    max_steps = sim_cfg.get("max_steps", max_simulation_steps)
    dt = sim_cfg.get("time_step", simulation_dt)

    t_sim_start = time.perf_counter()
    sim_metrics = run_simulation(
        world=world,
        path=smoothed_path,
        max_steps=max_steps,
        dt=dt,
        max_linear_speed=float(robot_cfg.get("max_speed", 40.0)),
        max_angular_speed=2.5,
        lookahead_distance=25.0,
        goal_tolerance=5.0,
        check_footprint=check_footprint,
        robot_radius=world.robot_radius,
        dynamic_obstacles=world.dynamic_obstacles,
        enable_replanning=True,
    )
    simulation_wall_time_ms = (time.perf_counter() - t_sim_start) * 1000.0

    motion_safe = not sim_metrics.collision_occurred
    all_safety_passed = bool(path_footprint_safe and motion_safe)

    # Classify outcome string and evaluation category
    if sim_metrics.goal_reached and not sim_metrics.collision_occurred:
        outcome = "REACHED_GOAL"
        eval_cat = "NAVIGATION_SUCCESS"
    elif sim_metrics.collision_occurred:
        outcome = "COLLISION"
        eval_cat = "EXPECTED_FAILURE"
    elif sim_metrics.steps >= max_steps:
        outcome = "STEP_LIMIT"
        eval_cat = "EXPECTED_FAILURE"
    else:
        outcome = sim_metrics.status.value
        eval_cat = "EXPECTED_FAILURE"

    return ScenarioEvaluationResult(
        scenario_name=scenario_name,
        config_path=cfg_path_str,
        robot_radius=world.robot_radius,
        world_dimensions=(world.width, world.height),
        grid_resolution=world.grid_resolution,
        planning_success=True,
        planning_time_ms=planning_time_ms,
        expanded_nodes=plan_result.expanded_nodes,
        original_waypoints=raw_wpts,
        smoothed_waypoints=smooth_wpts,
        waypoint_reduction_pct=wpt_reduction_pct,
        original_path_length_world=raw_path_len_world,
        smoothed_path_length_world=smoothed_path_len_world,
        length_reduction_pct=len_reduction_pct,
        path_footprint_safe=path_footprint_safe,
        raw_path_footprint_safe=raw_path_footprint_safe,
        smoothed_path_footprint_safe=smoothed_path_footprint_safe,
        simulation_outcome=outcome,
        simulation_steps=sim_metrics.steps,
        simulated_time_s=sim_metrics.elapsed_time,
        simulation_wall_time_ms=simulation_wall_time_ms,
        actual_distance_traveled_world=sim_metrics.distance_traveled,
        final_distance_to_goal=sim_metrics.final_distance_to_goal,
        motion_footprint_safe=motion_safe,
        all_safety_checks_passed=all_safety_passed,
        replans_attempted=sim_metrics.replans_attempted,
        replans_succeeded=sim_metrics.replans_succeeded,
        dynamic_obstacles_count=sim_metrics.dynamic_obstacles_count,
        collision_occurred=sim_metrics.collision_occurred,
        evaluation_category=eval_cat,
        notes=sim_metrics.message,
    )


def run_evaluation_suite(
    scenario_paths: Optional[Sequence[str | Path]] = None,
    output_json_path: Optional[str | Path] = None,
    connectivity: int = 8,
    check_footprint: bool = True,
    radius_override: Optional[float] = None,
    print_console: bool = True,
) -> EvaluationSuiteReport:
    """
    Run evaluation suite across multiple scenarios, print console summaries,
    and save machine-readable JSON output.

    Args:
        scenario_paths: Sequence of scenario config paths. Defaults to DEFAULT_SCENARIOS.
        output_json_path: Path to write JSON report. Defaults to 'results/evaluation_report.json'.
        connectivity: A* connectivity (4 or 8).
        check_footprint: Enforce footprint safety.
        radius_override: Optional robot radius override.
        print_console: If True, prints formatted summary table to standard output.

    Returns:
        EvaluationSuiteReport containing results for all evaluated scenarios.
    """
    if scenario_paths is None:
        scenario_paths = DEFAULT_SCENARIOS

    results: List[ScenarioEvaluationResult] = []
    nav_success_count = 0
    expected_failure_count = 0
    unexpected_error_count = 0

    if print_console:
        print("=" * 100)
        print("AI-BASED AUTONOMOUS NAVIGATION SYSTEM - REPRODUCIBLE EVALUATION BENCHMARK")
        print("=" * 100)
        print(f"Total scenarios scheduled: {len(scenario_paths)}")
        print(f"Footprint-aware verification: {'ACTIVE' if check_footprint else 'DISABLED'}")
        print("-" * 100)

    for p in scenario_paths:
        path_obj = Path(p)
        try:
            res = evaluate_scenario(
                path_obj,
                connectivity=connectivity,
                check_footprint=check_footprint,
                radius_override=radius_override,
            )
            results.append(res)
            if res.evaluation_category == "NAVIGATION_SUCCESS":
                nav_success_count += 1
            elif res.evaluation_category == "EXPECTED_FAILURE":
                expected_failure_count += 1
            else:
                unexpected_error_count += 1
        except Exception as exc:
            # Catch only external scenario loading or schema validation errors
            if isinstance(exc, (FileNotFoundError, ValueError)):
                unexpected_error_count += 1
                results.append(
                    ScenarioEvaluationResult(
                        scenario_name=path_obj.stem,
                        config_path=str(p),
                        robot_radius=15.0 if radius_override is None else radius_override,
                        world_dimensions=(0.0, 0.0),
                        grid_resolution=10.0,
                        planning_success=False,
                        planning_time_ms=0.0,
                        expanded_nodes=0,
                        simulation_outcome="NOT_RUN",
                        all_safety_checks_passed=False,
                        evaluation_category="UNEXPECTED_ERROR",
                        notes=f"Scenario load/validation error: {exc}",
                    )
                )
            else:
                # Re-raise unexpected internal programming errors (AttributeError, TypeError, IndexError, etc.)
                raise

    report = EvaluationSuiteReport(
        timestamp=datetime.now(timezone.utc).isoformat(),
        disclaimer=SAFETY_DISCLAIMER,
        scenarios_evaluated=len(results),
        navigation_successes=nav_success_count,
        expected_planning_failures=expected_failure_count,
        unexpected_evaluation_errors=unexpected_error_count,
        scenarios_passed=nav_success_count,
        scenarios_failed=expected_failure_count + unexpected_error_count,
        results=results,
    )

    # Save JSON report if output path is provided or default results dir exists
    dest_path = (
        Path(output_json_path)
        if output_json_path is not None
        else Path("results/evaluation_report.json")
    )
    saved_file = report.save_json(dest_path)

    if print_console:
        print(report.format_summary_table())
        print()
        print(
            f"Summary: {len(results)} evaluated | {nav_success_count} navigation successes | "
            f"{expected_failure_count} expected planning failures | {unexpected_error_count} unexpected evaluation errors"
        )
        print(f"Machine-readable JSON report saved to: {saved_file}")
        print()
        print("CRITICAL DISTINCTIONS & SAFETY POLICY:")
        print("1. Simulated Time vs. Wall-Clock Time:")
        print("   'Sim Time' represents simulated physics time (steps * dt), whereas 'Plan Time' and")
        print("   simulation wall-clock time represent actual CPU computation duration.")
        print("2. Planned Path Length vs. Actual Distance Traveled:")
        print("   'Planned Len' is the discrete/smoothed geometric path length; 'Traveled Len' is the actual")
        print("   continuous Euclidean distance covered by the kinematic robot under Pure Pursuit tracking.")
        print("3. Absence of Collision in Tested Run vs. Universal Safety Guarantee:")
        print(f"   {SAFETY_DISCLAIMER}")
        print("=" * 100)

    return report
