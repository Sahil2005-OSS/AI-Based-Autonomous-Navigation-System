"""
Automated Visual Asset Capture Utility for AI-Based Autonomous Navigation System.

Headlessly executes the navigation simulations and captures high-resolution
demonstration screenshots directly into assets/ for documentation and portfolio presentation.
"""

import os
from pathlib import Path
import math

os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame
import numpy as np

from autonomous_nav.config_loader import load_config
from autonomous_nav.world import World
from autonomous_nav.planner import AStarPlanner
from autonomous_nav.path_smoother import PathSmoother
from autonomous_nav.visualizer import SimulationVisualizer
from autonomous_nav.controller import ControllerStatus


def capture_static_arena(output_path: Path) -> None:
    """Capture static arena navigation with smoothed route and mid-trajectory robot pose."""
    print("Capturing static arena demo...")
    cfg = load_config(Path("config/default_scenario.json"))
    world = World.from_config(cfg)
    start_pos = tuple(cfg["robot"]["start_position"])
    goal_pos = tuple(cfg["robot"]["target_position"])
    start_grid, goal_grid = world.validate_start_goal(start_pos, goal_pos, check_footprint=True)

    planner = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True, use_footprint=True)
    plan_res = planner.plan(start_grid, goal_grid)

    smoother = PathSmoother(world, check_footprint=True)
    smooth_res = smoother.smooth_path(plan_res.path)

    sim = SimulationVisualizer(
        world=world,
        path_result=plan_res,
        start_grid=start_grid,
        goal_grid=goal_grid,
        connectivity=8,
        smoothed_path=smooth_res.smoothed_path,
        continuous_motion=True,
        init_display=True,
        check_footprint=True,
        robot_radius=world.robot_radius,
    )

    # Step motion forward so robot is actively traversing along smoothed path
    for _ in range(160):
        sim.step_motion(0.05)

    sim.render_frame()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(sim.screen, str(output_path))
    print(f"Saved: {output_path}")


def capture_dynamic_detour(output_path: Path) -> None:
    """Capture dynamic obstacle avoidance and online detour replanning."""
    print("Capturing dynamic obstacle detour demo...")
    cfg = load_config(Path("config/scenarios/dynamic_obstacle.json"))
    world = World.from_config(cfg)
    start_pos = tuple(cfg["robot"]["start_position"])
    goal_pos = tuple(cfg["robot"]["target_position"])
    start_grid, goal_grid = world.validate_start_goal(start_pos, goal_pos, check_footprint=True)

    planner = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True, use_footprint=True)
    plan_res = planner.plan(start_grid, goal_grid)

    smoother = PathSmoother(world, check_footprint=True)
    smooth_res = smoother.smooth_path(plan_res.path)

    sim = SimulationVisualizer(
        world=world,
        path_result=plan_res,
        start_grid=start_grid,
        goal_grid=goal_grid,
        connectivity=8,
        smoothed_path=smooth_res.smoothed_path,
        continuous_motion=True,
        init_display=True,
        check_footprint=True,
        robot_radius=world.robot_radius,
    )

    # Step simulation through initial detour encounter
    for _ in range(120):
        sim.step_motion(0.05)

    sim.render_frame()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(sim.screen, str(output_path))
    print(f"Saved: {output_path}")


def capture_blocked_safestop(output_path: Path) -> None:
    """Capture blocked corridor scenario with fail-safe zero-velocity halting."""
    print("Capturing blocked safe stop demo...")
    cfg = load_config(Path("config/scenarios/dynamic_obstacle_blocked.json"))
    world = World.from_config(cfg)
    start_pos = tuple(cfg["robot"]["start_position"])
    goal_pos = tuple(cfg["robot"]["target_position"])
    start_grid, goal_grid = world.validate_start_goal(start_pos, goal_pos, check_footprint=True)

    planner = AStarPlanner(world, connectivity=8, strict_diagonal_clearance=True, use_footprint=True)
    plan_res = planner.plan(start_grid, goal_grid)

    smoother = PathSmoother(world, check_footprint=True)
    smooth_res = smoother.smooth_path(plan_res.path)

    sim = SimulationVisualizer(
        world=world,
        path_result=plan_res,
        start_grid=start_grid,
        goal_grid=goal_grid,
        connectivity=8,
        smoothed_path=smooth_res.smoothed_path,
        continuous_motion=True,
        init_display=True,
        check_footprint=True,
        robot_radius=world.robot_radius,
    )

    # Step simulation until blocked condition is reached
    for _ in range(30):
        sim.step_motion(0.05)

    sim.render_frame()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(sim.screen, str(output_path))
    print(f"Saved: {output_path}")


def main() -> None:
    assets_dir = Path("assets")
    capture_static_arena(assets_dir / "demo_static_arena.png")
    capture_dynamic_detour(assets_dir / "demo_dynamic_detour.png")
    capture_blocked_safestop(assets_dir / "demo_blocked_safestop.png")
    print("All demonstration assets generated successfully.")


if __name__ == "__main__":
    main()
