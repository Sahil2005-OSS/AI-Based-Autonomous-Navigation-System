"""
Main entry point for the Autonomous Navigation System.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Optional
import numpy as np

from autonomous_nav.config_loader import load_config
from autonomous_nav.world import World, CELL_OCCUPIED, CELL_CLEARANCE_VIOLATION
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.path_smoother import PathSmoother, has_line_of_sight


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="AI-Based Autonomous Navigation System"
    )
    default_cfg = Path(__file__).resolve().parents[2] / "config" / "default_scenario.json"
    parser.add_argument(
        "--config",
        type=str,
        default=str(default_cfg),
        help="Path to scenario JSON configuration file",
    )
    parser.add_argument(
        "--connectivity",
        type=int,
        choices=[4, 8],
        default=8,
        help="Grid connectivity for A* path planning (4 or 8, default: 8)",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        default=False,
        help="Launch interactive 2D Pygame simulation window",
    )
    parser.add_argument(
        "--no-smooth",
        action="store_true",
        default=False,
        help="Disable collision-safe path smoothing post-processor",
    )
    parser.add_argument(
        "--motion",
        action="store_true",
        default=False,
        help="Enable continuous kinematic robot motion and Pure Pursuit path tracking",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=None,
        help="Optional override for robot circular footprint radius (world units)",
    )
    parser.add_argument(
        "--evaluate",
        action="store_true",
        default=False,
        help="Run reproducible evaluation and benchmarking across navigation scenarios (headless)",
    )
    parser.add_argument(
        "--eval-output",
        type=str,
        default="results/evaluation_report.json",
        help="Destination path for evaluation JSON report (default: results/evaluation_report.json)",
    )
    return parser.parse_args(args)


def run(
    config_path: str,
    connectivity: int = 8,
    visualize: bool = False,
    no_smooth: bool = False,
    motion: bool = False,
    radius: float | None = None,
    evaluate: bool = False,
    eval_output: str | None = None,
) -> int:
    """
    Initialize simulation environment from configuration, run path planning demonstration,
    perform collision-safe path smoothing, execute continuous motion simulation if requested,
    run reproducible evaluation benchmarking if requested, and optionally launch the visualizer.
    """
    if evaluate:
        from autonomous_nav.evaluation import run_evaluation_suite

        default_cfg = Path(__file__).resolve().parents[2] / "config" / "default_scenario.json"
        scenario_paths = None
        if Path(config_path).resolve() != default_cfg.resolve():
            scenario_paths = [Path(config_path)]

        run_evaluation_suite(
            scenario_paths=scenario_paths,
            output_json_path=eval_output,
            connectivity=connectivity,
            check_footprint=True,
            radius_override=radius,
            print_console=True,
        )
        return 0
    print("=" * 60)
    print("AI-Based Autonomous Navigation System - Environment & A* Planner")
    print("=" * 60)
    print(f"Loading configuration from: {config_path}")

    config_data = load_config(config_path)
    scenario_name = config_data.get("scenario_name", "Unnamed Scenario")
    print(f"Scenario: {scenario_name}")

    world = World.from_config(config_data)
    if radius is not None:
        world.robot_radius = radius
    print(f"World continuous bounds: {world.width} x {world.height} (resolution: {world.grid_resolution})")
    print(f"Discrete occupancy grid: {world.grid_width} cols x {world.grid_height} rows (shape: {world.grid.shape})")
    print(f"Robot circular footprint: radius = {world.robot_radius:.1f} world units")
    print(f"Geometric obstacles loaded: {len(world.obstacles)}")

    occupied_count = int(np.count_nonzero(world.grid == CELL_OCCUPIED))
    clearance_count = int(np.count_nonzero(world.clearance_grid == CELL_CLEARANCE_VIOLATION))
    total_cells = world.grid_width * world.grid_height
    free_cells = total_cells - occupied_count
    safe_cells = total_cells - clearance_count
    print(f"Grid cell stats: {occupied_count} occupied obstacles, {free_cells} free cells ({total_cells} total)")
    print(f"Clearance stats: {clearance_count} footprint-restricted cells (obstacles + boundary buffer), {safe_cells} footprint-safe cells")

    start_grid: Optional[tuple[int, int]] = None
    goal_grid: Optional[tuple[int, int]] = None
    result = None
    smoothed_result = None

    robot_cfg = config_data.get("robot")
    if robot_cfg:
        start_pos = tuple(robot_cfg.get("start_position", [0.0, 0.0]))
        target_pos = tuple(robot_cfg.get("target_position", [0.0, 0.0]))
        try:
            start_grid, goal_grid = world.validate_start_goal(start_pos, target_pos, check_footprint=True)
            print(f"Robot start valid: world {start_pos} -> grid {start_grid} (footprint-safe)")
            print(f"Robot target valid: world {target_pos} -> grid {goal_grid} (footprint-safe)")

            print("-" * 60)
            print(f"Running Footprint-Safe A* Planner ({connectivity}-connected, radius={world.robot_radius:.1f})...")
            planner = AStarPlanner(
                world, connectivity=connectivity, strict_diagonal_clearance=True, use_footprint=True
            )
            result = planner.plan(start_grid, goal_grid)

            print(f"Path-found status : {'SUCCESS' if result.success else 'FAILED'}")
            print(f"Path cost         : {result.cost:.4f}")
            print(f"Expanded nodes    : {result.expanded_nodes}")
            print(f"Waypoints in path : {len(result.path)}")
            if result.success and result.path:
                print(f"Path trajectory   : Start {result.path[0]} -> ... -> Goal {result.path[-1]}")
                all_safe = all(world.is_footprint_safe_grid(x, y) for x, y in result.path)
                print(f"Footprint safety  : {'PASS (all path cells footprint-safe)' if all_safe else 'FAIL'}")

                # Path Smoothing Stage
                if not no_smooth:
                    print("-" * 60)
                    print(f"Running Collision-Safe Path Smoothing (Line-of-Sight, radius={world.robot_radius:.1f})...")
                    smoother = PathSmoother(world, check_footprint=True)
                    smoothed_result = smoother.smooth_result(result)

                    orig_wpts = smoothed_result.original_waypoints
                    smooth_wpts = smoothed_result.smoothed_waypoints
                    reduced = smoothed_result.waypoints_removed
                    reduction_pct = (reduced / orig_wpts) * 100 if orig_wpts > 0 else 0

                    print(f"Original waypoints: {orig_wpts}")
                    print(f"Smoothed waypoints: {smooth_wpts} ({reduction_pct:.1f}% reduction, {reduced} waypoints eliminated)")
                    print(f"Original cost     : {smoothed_result.original_cost:.4f}")
                    print(f"Smoothed cost     : {smoothed_result.smoothed_cost:.4f}")

                    # Validate all smoothed segments against clearance grid
                    segments_safe = all(
                        has_line_of_sight(world, p1, p2, check_footprint=True)
                        for p1, p2 in zip(smoothed_result.smoothed_path[:-1], smoothed_result.smoothed_path[1:])
                    )
                    print(f"Segment safety    : {'PASS (100% footprint-clear)' if segments_safe else 'FAIL'}")
                else:
                    print("-" * 60)
                    print("Path smoothing bypassed (--no-smooth flag active).")

                # Continuous Kinematic Motion & Pure Pursuit Demonstration
                if motion:
                    print("-" * 60)
                    print(f"Running Continuous Kinematic Motion & Pure Pursuit (Footprint radius={world.robot_radius:.1f})...")
                    from autonomous_nav.controller import run_simulation
                    tracking_path = (
                        smoothed_result.smoothed_path
                        if (smoothed_result and smoothed_result.success)
                        else result.path
                    )
                    sim_metrics = run_simulation(
                        world=world,
                        path=tracking_path,
                        max_steps=2000,
                        dt=0.05,
                        max_linear_speed=40.0,
                        max_angular_speed=2.5,
                        lookahead_distance=25.0,
                        goal_tolerance=5.0,
                        check_footprint=True,
                        robot_radius=world.robot_radius,
                    )
                    print(f"Simulation Status   : {sim_metrics.status.value}")
                    print(f"Goal Reached        : {'YES' if sim_metrics.goal_reached else 'NO'}")
                    print(f"Collision / OOB     : {'YES (COLLISION DETECTED)' if sim_metrics.collision_occurred else 'NO (Collision-Free)'}")
                    print(f"Steps Executed      : {sim_metrics.steps}")
                    print(f"Simulated Time      : {sim_metrics.elapsed_time:.2f} s")
                    print(f"Initial Pose        : (x={sim_metrics.initial_pose[0]:.2f}, y={sim_metrics.initial_pose[1]:.2f}, theta={math.degrees(sim_metrics.initial_pose[2]):.1f} deg)")
                    print(f"Final Pose          : (x={sim_metrics.final_pose[0]:.2f}, y={sim_metrics.final_pose[1]:.2f}, theta={math.degrees(sim_metrics.final_pose[2]):.1f} deg)")
                    print(f"Max Commanded Speed : Linear {sim_metrics.max_linear_speed:.2f} u/s, Angular {sim_metrics.max_angular_speed:.2f} rad/s")
                    if sim_metrics.replans_attempted > 0:
                        print(f"Replans Executed    : {sim_metrics.replans_succeeded} succeeded / {sim_metrics.replans_attempted} attempted")
                    print(f"Diagnostics         : {sim_metrics.message}")

        except ValueError as err:
            print(f"Warning: Validation or planning failed: {err}")

    print("=" * 60)

    # Launch Pygame visual simulation if requested
    if visualize:
        if start_grid is None or goal_grid is None or result is None:
            print("Cannot launch visualization: Start/goal positions or planner result unavailable.")
            return 1

        print("Launching interactive Pygame visualizer...")
        from autonomous_nav.visualizer import SimulationVisualizer

        if motion:
            print("Mode: CONTINUOUS MOTION (Pure Pursuit)")
            print("Controls: [SPACE / P] Pause/Resume | [R] Reset to Start Pose | [ESC / Q] Quit")
            sim = SimulationVisualizer(
                world=world,
                path_result=result,
                start_grid=start_grid,
                goal_grid=goal_grid,
                connectivity=connectivity,
                smoothed_path=smoothed_result.smoothed_path if smoothed_result else None,
                continuous_motion=True,
                max_linear_speed=40.0,
                max_angular_speed=2.5,
                lookahead_distance=25.0,
                goal_tolerance=5.0,
                check_footprint=True,
                robot_radius=world.robot_radius,
            )
        else:
            print("Mode: DISCRETE WAYPOINTS (Manual Step & Autoplay)")
            print("Controls: [SPACE] Step 1 Waypoint | [P / A] Toggle Autoplay | [R] Reset | [ESC / Q] Quit")
            sim = SimulationVisualizer(
                world=world,
                path_result=result,
                start_grid=start_grid,
                goal_grid=goal_grid,
                connectivity=connectivity,
                smoothed_path=smoothed_result.smoothed_path if smoothed_result else None,
                continuous_motion=False,
                check_footprint=True,
                robot_radius=world.robot_radius,
            )

        sim.run()
        print("Visualizer closed cleanly.")
    else:
        if not motion:
            print("A* Path Planning demonstration complete. Use '--visualize' or '--motion' to run simulation.")
        else:
            print("Continuous motion demonstration complete. Use '--visualize --motion' to view GUI animation.")

    return 0


def main() -> None:
    """CLI execution entry point."""
    args = parse_args()
    sys.exit(
        run(
            args.config,
            connectivity=args.connectivity,
            visualize=args.visualize,
            no_smooth=args.no_smooth,
            motion=args.motion,
            radius=args.radius,
            evaluate=args.evaluate,
            eval_output=args.eval_output,
        )
    )


if __name__ == "__main__":
    main()
