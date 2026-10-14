"""
Collision-Safe Path Smoothing Module.

Implements line-of-sight shortcutting for discrete A* paths over a 2D occupancy grid.
Reduces redundant waypoints and turns while guaranteeing that every retained segment
is verified collision-free and does not cut blocked obstacle corners.

Guarantees & Constraints:
-------------------------
1. Collision Safety: Every retained straight-line segment is checked against the
   occupancy grid using conservative DDA grid raycasting.
2. Endpoint Preservation: The original start and goal waypoints are strictly preserved.
3. Grid Non-Mutation: The world model and occupancy grid are never modified.
4. Conservative Fallback: If line-of-sight cannot be guaranteed for a segment, the
   shortcut is rejected and intermediate safe waypoints are retained.
5. Non-Global Optimality: The smoother performs greedy line-of-sight shortcutting;
   it is not guaranteed to find a globally optimal trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple, Union

from autonomous_nav.world import World, CELL_OCCUPIED
from autonomous_nav.planner import PathResult


@dataclass
class SmoothedPathResult:
    """
    Result returned by the path smoothing process.

    Attributes:
        success: True if the path was valid and processed; False if input was invalid/failed.
        original_path: The input path before smoothing.
        smoothed_path: The resulting path with redundant waypoints removed.
        original_waypoints: Total waypoint count of original path.
        smoothed_waypoints: Total waypoint count of smoothed path.
        original_cost: Cumulative movement cost of the original path.
        smoothed_cost: Cumulative Euclidean length of the smoothed path.
        waypoints_removed: Number of intermediate waypoints eliminated.
        message: Explanatory status message.
    """
    success: bool
    original_path: List[Tuple[int, int]]
    smoothed_path: List[Tuple[int, int]]
    original_waypoints: int
    smoothed_waypoints: int
    original_cost: float
    smoothed_cost: float
    waypoints_removed: int
    message: str = ""


def get_traversed_cells(
    x0: int,
    y0: int,
    x1: int,
    y1: int
) -> List[Tuple[int, int]]:
    """
    Return all discrete grid cells whose interior is traversed by the straight segment
    connecting the center of (x0, y0) to the center of (x1, y1).

    Uses a conservative grid DDA algorithm that accurately tracks all intersected cells.
    """
    if (x0, y0) == (x1, y1):
        return [(x0, y0)]

    dx = x1 - x0
    dy = y1 - y0

    # Pure vertical
    if dx == 0:
        step = 1 if dy > 0 else -1
        return [(x0, y) for y in range(y0, y1 + step, step)]

    # Pure horizontal
    if dy == 0:
        step = 1 if dx > 0 else -1
        return [(x, y0) for x in range(x0, x1 + step, step)]

    # General segment using grid DDA
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    xA, yA = x0 + 0.5, y0 + 0.5

    gx, gy = x0, y0
    cells = [(gx, gy)]

    t_next_x = ((gx + 1 - xA) / dx) if sx > 0 else ((gx - xA) / dx)
    t_next_y = ((gy + 1 - yA) / dy) if sy > 0 else ((gy - yA) / dy)
    dt_x = 1.0 / abs(dx)
    dt_y = 1.0 / abs(dy)

    eps = 1e-9
    while (gx, gy) != (x1, y1):
        if abs(t_next_x - t_next_y) < eps:
            gx += sx
            gy += sy
            t_next_x += dt_x
            t_next_y += dt_y
        elif t_next_x < t_next_y:
            gx += sx
            t_next_x += dt_x
        else:
            gy += sy
            t_next_y += dt_y

        cells.append((gx, gy))

    return cells


def has_line_of_sight(
    world: World,
    p0: Tuple[int, int],
    p1: Tuple[int, int],
    check_footprint: bool = False,
) -> bool:
    """
    Check if a direct line-of-sight segment exists between p0 and p1 without
    crossing any occupied obstacle cells or cutting blocked corners.
    Optionally enforces circular footprint clearance along the entire segment.

    Conservative Safety Checks:
    1. Endpoint verification: p0 and p1 must be inside the grid and free (and footprint-safe).
    2. Ray traversal: every grid cell whose interior is pierced by the ray must be free.
    3. Corner-crossing prevention: whenever the segment passes through an exact grid
       vertex between diagonal cells, the two adjacent orthogonal cells (gx + sx, gy)
       and (gx, gy + sy) sharing that vertex are evaluated. If either adjacent cell is
       occupied or outside the grid (or violates clearance), the segment is rejected.
    4. Continuous footprint sampling: if check_footprint is True, continuous points along
       the segment are checked for full circular footprint clearance.

    Args:
        world: The environment World instance.
        p0: Starting grid coordinate (gx, gy).
        p1: Ending grid coordinate (gx, gy).
        check_footprint: If True, checks clearance against world.clearance_grid and
                         continuous robot footprint radius.

    Returns:
        True if the entire segment is collision-free; False otherwise.
    """
    x0, y0 = p0
    x1, y1 = p1

    def is_cell_free(gx: int, gy: int) -> bool:
        if not world.is_inside_grid(gx, gy):
            return False
        if check_footprint:
            return world.is_footprint_safe_grid(gx, gy)
        return world.is_traversable(gx, gy)

    def is_cell_blocked(gx: int, gy: int) -> bool:
        if not world.is_inside_grid(gx, gy):
            return True
        if check_footprint:
            return not world.is_footprint_safe_grid(gx, gy)
        return world.is_occupied(gx, gy)

    # 1. Endpoint validity
    if not (world.is_inside_grid(x0, y0) and world.is_inside_grid(x1, y1)):
        return False
    if not is_cell_free(x0, y0) or not is_cell_free(x1, y1):
        return False

    if p0 == p1:
        return True

    dx = x1 - x0
    dy = y1 - y0

    # 2. Pure orthogonal lines
    if dx == 0:
        step = 1 if dy > 0 else -1
        for y in range(y0, y1 + step, step):
            if is_cell_blocked(x0, y):
                return False
    elif dy == 0:
        step = 1 if dx > 0 else -1
        for x in range(x0, x1 + step, step):
            if is_cell_blocked(x, y0):
                return False
    else:
        # 3. Arbitrary angle: Grid DDA with vertex collision checks
        sx = 1 if dx > 0 else -1
        sy = 1 if dy > 0 else -1
        xA, yA = x0 + 0.5, y0 + 0.5

        gx, gy = x0, y0

        t_next_x = ((gx + 1 - xA) / dx) if sx > 0 else ((gx - xA) / dx)
        t_next_y = ((gy + 1 - yA) / dy) if sy > 0 else ((gy - yA) / dy)
        dt_x = 1.0 / abs(dx)
        dt_y = 1.0 / abs(dy)

        eps = 1e-9
        while (gx, gy) != (x1, y1):
            if abs(t_next_x - t_next_y) < eps:
                # Segment crosses exactly through a grid vertex!
                adj_x = (gx + sx, gy)
                adj_y = (gx, gy + sy)

                # Conservative corner-cutting prevention: reject if EITHER adjacent
                # orthogonal cell is blocked or outside the grid bounds.
                if is_cell_blocked(*adj_x) or is_cell_blocked(*adj_y):
                    return False

                gx += sx
                gy += sy
                t_next_x += dt_x
                t_next_y += dt_y

            elif t_next_x < t_next_y:
                gx += sx
                t_next_x += dt_x
            else:
                gy += sy
                t_next_y += dt_y

            if is_cell_blocked(gx, gy):
                return False

    # 4. Exact swept footprint verification along segment if requested
    if check_footprint:
        wx0, wy0 = world.grid_to_world(x0, y0)
        wx1, wy1 = world.grid_to_world(x1, y1)
        if not world.is_swept_footprint_safe(wx0, wy0, wx1, wy1):
            return False

    return True


def calculate_path_cost(path: List[Tuple[int, int]]) -> float:
    """Calculate the cumulative Euclidean length of a waypoint sequence."""
    if len(path) < 2:
        return 0.0
    cost = 0.0
    for i in range(len(path) - 1):
        p1, p2 = path[i], path[i + 1]
        cost += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    return cost


class PathSmoother:
    """
    Greedy line-of-sight path post-processor for discrete 2D occupancy grids.
    """

    def __init__(self, world: World, check_footprint: bool = False) -> None:
        """
        Initialize the PathSmoother.

        Args:
            world: An initialized `World` instance.
            check_footprint: If True, enforces robot footprint clearance for all shortcuts.
        """
        self.world = world
        self.check_footprint = bool(check_footprint)

    def smooth_path(self, path: List[Tuple[int, int]]) -> SmoothedPathResult:
        """
        Smooth an existing waypoint sequence using greedy line-of-sight shortcutting.

        Args:
            path: List of (gx, gy) grid coordinates.

        Returns:
            SmoothedPathResult containing both original and smoothed paths with metrics.

        Raises:
            ValueError: If path contains invalid coordinate formats, out-of-bounds,
                        or occupied cells.
        """
        # 1. Handle empty path
        if not path:
            return SmoothedPathResult(
                success=False,
                original_path=[],
                smoothed_path=[],
                original_waypoints=0,
                smoothed_waypoints=0,
                original_cost=float("inf"),
                smoothed_cost=float("inf"),
                waypoints_removed=0,
                message="Input path was empty.",
            )

        # 2. Validate all waypoints in input path
        for idx, pt in enumerate(path):
            if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
                raise ValueError(
                    f"Waypoint at index {idx} must be a 2-element coordinate pair, got {pt}"
                )
            gx, gy = int(pt[0]), int(pt[1])
            if not self.world.is_inside_grid(gx, gy):
                raise ValueError(
                    f"Waypoint at index {idx} ({gx}, {gy}) is out of bounds for grid shape "
                    f"({self.world.grid_width}, {self.world.grid_height})."
                )
            if self.world.is_occupied(gx, gy):
                raise ValueError(
                    f"Waypoint at index {idx} ({gx}, {gy}) lies on an occupied obstacle cell."
                )
            if self.check_footprint and not self.world.is_footprint_safe_grid(gx, gy):
                raise ValueError(
                    f"Waypoint at index {idx} ({gx}, {gy}) lacks robot footprint clearance."
                )

        # 3. Handle single-point path
        if len(path) == 1:
            return SmoothedPathResult(
                success=True,
                original_path=list(path),
                smoothed_path=list(path),
                original_waypoints=1,
                smoothed_waypoints=1,
                original_cost=0.0,
                smoothed_cost=0.0,
                waypoints_removed=0,
                message="Single-point path returned unmodified.",
            )

        # 4. Handle 2-point path
        if len(path) == 2:
            orig_cost = calculate_path_cost(path)
            return SmoothedPathResult(
                success=True,
                original_path=list(path),
                smoothed_path=list(path),
                original_waypoints=2,
                smoothed_waypoints=2,
                original_cost=orig_cost,
                smoothed_cost=orig_cost,
                waypoints_removed=0,
                message="Two-point path already minimal.",
            )

        # 5. Greedy Line-of-Sight Shortcutting
        original_waypoints = len(path)
        original_cost = calculate_path_cost(path)

        smoothed: List[Tuple[int, int]] = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            found_shortcut = False
            for target_idx in range(len(path) - 1, current_idx + 1, -1):
                if has_line_of_sight(
                    self.world,
                    path[current_idx],
                    path[target_idx],
                    check_footprint=self.check_footprint,
                ):
                    smoothed.append(path[target_idx])
                    current_idx = target_idx
                    found_shortcut = True
                    break

            if not found_shortcut:
                current_idx += 1
                smoothed.append(path[current_idx])

        smoothed_cost = calculate_path_cost(smoothed)
        removed_count = original_waypoints - len(smoothed)

        return SmoothedPathResult(
            success=True,
            original_path=list(path),
            smoothed_path=smoothed,
            original_waypoints=original_waypoints,
            smoothed_waypoints=len(smoothed),
            original_cost=original_cost,
            smoothed_cost=smoothed_cost,
            waypoints_removed=removed_count,
            message="Path successfully smoothed with collision-safe shortcuts.",
        )

    def smooth_result(self, result: PathResult) -> SmoothedPathResult:
        """
        Smooth the path contained within a PathResult.

        Args:
            result: PathResult from AStarPlanner.

        Returns:
            SmoothedPathResult.
        """
        if not result.success or not result.path:
            return SmoothedPathResult(
                success=False,
                original_path=[],
                smoothed_path=[],
                original_waypoints=0,
                smoothed_waypoints=0,
                original_cost=float("inf"),
                smoothed_cost=float("inf"),
                waypoints_removed=0,
                message="Cannot smooth unsuccessful or empty path result.",
            )

        smoothed_res = self.smooth_path(result.path)
        smoothed_res.original_cost = result.cost
        return smoothed_res


def smooth_path(
    world: World,
    path_or_result: Union[List[Tuple[int, int]], PathResult]
) -> SmoothedPathResult:
    """
    Convenience function to smooth a path or PathResult instance.
    """
    smoother = PathSmoother(world)
    if isinstance(path_or_result, PathResult):
        return smoother.smooth_result(path_or_result)
    return smoother.smooth_path(path_or_result)
