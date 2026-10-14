"""
A* Path Planning and Heuristic Search Module.

This module implements an A* pathfinder operating over the 2D discrete occupancy
grid provided by `autonomous_nav.world.World`.

Coordinate System:
------------------
- Grid coordinates: (gx, gy) where:
    0 <= gx < world.grid_width  (column along X-axis)
    0 <= gy < world.grid_height (row along Y-axis)

Connectivity & Movement Costs:
------------------------------
- 4-Connected Movement:
    - Orthogonal moves: dx in {-1, 0, 1}, dy in {-1, 0, 1} with |dx| + |dy| == 1.
    - Movement cost: 1.0.
    - Supported Heuristic: 'manhattan' (admissible & consistent), 'euclidean' (admissible).
- 8-Connected Movement:
    - Orthogonal moves: cost 1.0.
    - Diagonal moves: cost sqrt(2) (~1.41421356).
    - Supported Heuristic: 'euclidean' (admissible).
    - Note: 'manhattan' is inadmissible for 8-connected grids and will be rejected.

Diagonal Corner-Cutting Prevention:
-----------------------------------
A diagonal move from (x, y) to (x + dx, y + dy) is blocked if the move passes
between two blocked adjacent orthogonal cells (i.e. both (x + dx, y) and (x, y + dy)
are occupied). If `strict_diagonal_clearance` is True, a diagonal move is blocked if
either adjacent orthogonal cell is occupied.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from autonomous_nav.world import World, CELL_OCCUPIED


@dataclass
class PathResult:
    """
    Result returned by A* path planning.

    Attributes:
        success: True if a complete valid path was found, False otherwise.
        path: List of (gx, gy) coordinates from start to goal (inclusive).
              Empty list if no path was found.
        cost: Total accumulated movement cost along the path (inf if not found).
        expanded_nodes: Total count of nodes expanded from the open set.
        message: Informative status or failure reason.
    """
    success: bool
    path: List[Tuple[int, int]]
    cost: float
    expanded_nodes: int
    message: str = ""


# Deterministic neighbor offsets
ORTHOGONAL_NEIGHBORS: List[Tuple[int, int]] = [
    (0, -1),  # Up / North
    (1, 0),   # Right / East
    (0, 1),   # Down / South
    (-1, 0),  # Left / West
]

DIAGONAL_NEIGHBORS: List[Tuple[int, int]] = [
    (1, -1),  # North-East
    (1, 1),   # South-East
    (-1, 1),  # South-West
    (-1, -1), # North-West
]


class AStarPlanner:
    """
    A* path planner for discrete 2D occupancy grids.
    """

    def __init__(
        self,
        world: World,
        connectivity: int = 4,
        heuristic: Optional[str] = None,
        strict_diagonal_clearance: Optional[bool] = None,
        use_footprint: bool = False,
    ) -> None:
        """
        Initialize the A* planner.

        Args:
            world: An initialized `World` object.
            connectivity: 4 or 8 connected grid topology (default: 4).
            heuristic: 'manhattan' or 'euclidean'. Defaults to 'manhattan'
                       for 4-connected and 'euclidean' for 8-connected.
            strict_diagonal_clearance: If True, diagonal moves require both
                                       adjacent cells to be free. If False,
                                       only moves between two blocked cells
                                       are rejected. Defaults to True if use_footprint is True.
            use_footprint: If True, plans against the footprint clearance grid
                           and validates start/goal poses for robot radius clearance.

        Raises:
            ValueError: If connectivity or heuristic combination is invalid.
        """
        self.world = world

        if connectivity not in (4, 8):
            raise ValueError(
                f"Unsupported connectivity mode: {connectivity}. Expected 4 or 8."
            )
        self.connectivity = connectivity
        self.use_footprint = bool(use_footprint)
        if strict_diagonal_clearance is None:
            self.strict_diagonal_clearance = True if self.use_footprint else False
        else:
            self.strict_diagonal_clearance = bool(strict_diagonal_clearance)

        # Determine and validate heuristic
        if heuristic is None:
            self.heuristic_name = "manhattan" if connectivity == 4 else "euclidean"
        else:
            h_norm = heuristic.lower().strip()
            if h_norm not in ("manhattan", "euclidean"):
                raise ValueError(
                    f"Unknown heuristic '{heuristic}'. Supported: 'manhattan', 'euclidean'."
                )
            if connectivity == 8 and h_norm == "manhattan":
                raise ValueError(
                    "Manhattan heuristic is inadmissible for 8-connected movement "
                    "because it overestimates diagonal path costs. Use 'euclidean'."
                )
            self.heuristic_name = h_norm

    # -------------------------------------------------------------------------
    # Heuristics
    # -------------------------------------------------------------------------

    def calculate_heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate heuristic cost between grid points a and b."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if self.heuristic_name == "manhattan":
            return float(dx + dy)
        elif self.heuristic_name == "euclidean":
            return math.hypot(dx, dy)
        raise ValueError(f"Unsupported heuristic: {self.heuristic_name}")

    # -------------------------------------------------------------------------
    # Endpoint Validation
    # -------------------------------------------------------------------------

    def validate_endpoint(self, point: Tuple[int, int], name: str = "Point") -> Tuple[int, int]:
        """
        Validate that grid coordinates point=(gx, gy) are in-bounds and free.
        Optionally validates circular footprint clearance if use_footprint is enabled.

        Raises:
            ValueError: If point is malformed, out of bounds, occupied, or lacks footprint clearance.
        """
        if not (isinstance(point, (list, tuple)) and len(point) == 2):
            raise ValueError(f"{name} must be a 2-element grid coordinate pair (gx, gy), got {point}")

        gx, gy = int(point[0]), int(point[1])
        if not self.world.is_inside_grid(gx, gy):
            raise ValueError(
                f"{name} ({gx}, {gy}) is out of bounds for grid dimensions "
                f"({self.world.grid_width}, {self.world.grid_height})."
            )

        if self.world.is_occupied(gx, gy):
            raise ValueError(
                f"{name} ({gx}, {gy}) is occupied by an obstacle."
            )

        if self.use_footprint:
            wx, wy = self.world.grid_to_world(gx, gy)
            is_safe, reason = self.world.check_footprint_collision(wx, wy)
            if not is_safe:
                raise ValueError(
                    f"{name} ({gx}, {gy}) does not have adequate robot footprint clearance: {reason}"
                )

        return (gx, gy)

    # -------------------------------------------------------------------------
    # Neighbor Expansion
    # -------------------------------------------------------------------------

    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """
        Get valid, traversable neighbors and their transition costs in deterministic order.
        Respects footprint clearance map when use_footprint is active.

        Returns:
            List of ((nx, ny), step_cost).
        """
        cx, cy = current
        neighbors: List[Tuple[Tuple[int, int], float]] = []

        def is_cell_free(gx: int, gy: int) -> bool:
            if not self.world.is_inside_grid(gx, gy):
                return False
            if self.use_footprint:
                return self.world.is_footprint_safe_grid(gx, gy)
            return self.world.is_traversable(gx, gy)

        def is_cell_blocked(gx: int, gy: int) -> bool:
            if not self.world.is_inside_grid(gx, gy):
                return True
            if self.use_footprint:
                return not self.world.is_footprint_safe_grid(gx, gy)
            return self.world.is_occupied(gx, gy)

        # 1. Orthogonal moves (cost 1.0)
        for dx, dy in ORTHOGONAL_NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if is_cell_free(nx, ny):
                neighbors.append(((nx, ny), 1.0))

        # 2. Diagonal moves (cost sqrt(2)) if 8-connected
        if self.connectivity == 8:
            sqrt2 = math.sqrt(2.0)
            for dx, dy in DIAGONAL_NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if not is_cell_free(nx, ny):
                    continue

                # Diagonal corner-cutting check
                adj_x_blocked = is_cell_blocked(cx + dx, cy)
                adj_y_blocked = is_cell_blocked(cx, cy + dy)

                if self.strict_diagonal_clearance:
                    # Strictest: neither orthogonal neighbor may be blocked
                    if adj_x_blocked or adj_y_blocked:
                        continue
                else:
                    # Standard: cannot pass between two blocked adjacent cells
                    if adj_x_blocked and adj_y_blocked:
                        continue

                neighbors.append(((nx, ny), sqrt2))

        return neighbors

    # -------------------------------------------------------------------------
    # Main Search Routine
    # -------------------------------------------------------------------------

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> PathResult:
        """
        Find the shortest path from start to goal in grid coordinates.

        Args:
            start: Grid coordinates (gx, gy).
            goal: Grid coordinates (gx, gy).

        Returns:
            PathResult containing success status, path, cost, and expanded nodes.

        Raises:
            ValueError: If start or goal is invalid, out of bounds, occupied,
                        or if start and goal are identical.
        """
        start_pt = self.validate_endpoint(start, name="Start position")
        goal_pt = self.validate_endpoint(goal, name="Goal position")

        if start_pt == goal_pt:
            raise ValueError(
                f"Start position {start} and goal position {goal} are identical."
            )

        # Priority queue entries: (f_score, h_score, tie_breaker_counter, current_cell)
        counter: int = 0
        h_start = self.calculate_heuristic(start_pt, goal_pt)
        open_set: List[Tuple[float, float, int, Tuple[int, int]]] = [
            (h_start, h_start, counter, start_pt)
        ]

        g_score: Dict[Tuple[int, int], float] = {start_pt: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        closed_set: Set[Tuple[int, int]] = set()

        expanded_nodes: int = 0

        while open_set:
            f, h, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)
            expanded_nodes += 1

            # Check if goal reached
            if current == goal_pt:
                path = self._reconstruct_path(came_from, current)
                return PathResult(
                    success=True,
                    path=path,
                    cost=g_score[current],
                    expanded_nodes=expanded_nodes,
                    message="Path found successfully.",
                )

            current_g = g_score[current]

            for neighbor, step_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_neighbor = self.calculate_heuristic(neighbor, goal_pt)
                    f_neighbor = tentative_g + h_neighbor
                    counter += 1
                    heapq.heappush(open_set, (f_neighbor, h_neighbor, counter, neighbor))

        # Open set exhausted without reaching goal
        return PathResult(
            success=False,
            path=[],
            cost=float("inf"),
            expanded_nodes=expanded_nodes,
            message="No traversable path exists from start to goal.",
        )

    @staticmethod
    def _reconstruct_path(
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Reconstruct the sequence of grid coordinates from start to goal."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path


def find_path(
    world: World,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    connectivity: int = 4,
    heuristic: Optional[str] = None,
    strict_diagonal_clearance: bool = False,
) -> PathResult:
    """
    Convenience function to instantiate an AStarPlanner and search for a path.
    """
    planner = AStarPlanner(
        world=world,
        connectivity=connectivity,
        heuristic=heuristic,
        strict_diagonal_clearance=strict_diagonal_clearance,
    )
    return planner.plan(start, goal)
