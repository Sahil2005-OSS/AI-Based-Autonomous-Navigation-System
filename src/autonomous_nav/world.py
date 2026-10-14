"""
World representation and discrete 2D occupancy grid module.

Coordinate Convention:
----------------------
1. Continuous World Coordinates: (x, y) where:
     0.0 <= x < width
     0.0 <= y < height
   Units: Continuous floating-point metric/pixel units.

2. Discrete Grid Coordinates: (gx, gy) where:
     0 <= gx < grid_width  (columns along X-axis)
     0 <= gy < grid_height (rows along Y-axis)

3. NumPy Storage Indexing: grid[gy, gx]
   Array Shape: (grid_height, grid_width) -> (rows, columns).
   Accessing coordinate (gx, gy) maps directly to self.grid[gy, gx].

Occupancy Values:
-----------------
- CELL_FREE (0): Free, traversable space.
- CELL_OCCUPIED (1): Impassable obstacle.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Tuple
import numpy as np

CELL_FREE: int = 0
CELL_OCCUPIED: int = 1
CELL_CLEARANCE_VIOLATION: int = 2


@dataclass
class Obstacle:
    """Represents a geometric obstacle in the continuous world."""
    id: int
    obstacle_type: str
    position: Tuple[float, float]
    radius: float = 0.0
    width: float = 0.0
    height: float = 0.0

    def contains_point(self, x: float, y: float) -> bool:
        """
        Check if continuous point (x, y) is inside this obstacle.

        - circle: Euclidean distance from position <= radius.
        - rectangle: position is (top_left_x, top_left_y), spanning width and height.
        """
        ox, oy = self.position
        otype = self.obstacle_type.lower()
        if otype == "circle":
            return (x - ox) ** 2 + (y - oy) ** 2 <= self.radius ** 2
        elif otype == "rectangle":
            return ox <= x <= (ox + self.width) and oy <= y <= (oy + self.height)
        return False


def point_to_segment_distance(
    px: float, py: float, x0: float, y0: float, x1: float, y1: float
) -> float:
    """Exact Euclidean distance from point (px, py) to line segment (x0, y0)-(x1, y1)."""
    dx = x1 - x0
    dy = y1 - y0
    l_sq = dx * dx + dy * dy
    if l_sq < 1e-12:
        return math.hypot(px - x0, py - y0)
    t = ((px - x0) * dx + (py - y0) * dy) / l_sq
    t_clamped = max(0.0, min(1.0, t))
    qx = x0 + t_clamped * dx
    qy = y0 + t_clamped * dy
    return math.hypot(px - qx, py - qy)


def point_to_rectangle_distance(
    px: float, py: float, x_min: float, x_max: float, y_min: float, y_max: float
) -> float:
    """Exact Euclidean distance from point (px, py) to solid axis-aligned rectangle."""
    cx = max(x_min, min(px, x_max))
    cy = max(y_min, min(py, y_max))
    return math.hypot(px - cx, py - cy)


def segment_intersects_rectangle(
    x0: float, y0: float, x1: float, y1: float,
    x_min: float, x_max: float, y_min: float, y_max: float
) -> bool:
    """Check if line segment (x0, y0)-(x1, y1) intersects or touches rectangle [x_min, x_max] x [y_min, y_max]."""
    if x_min <= x0 <= x_max and y_min <= y0 <= y_max:
        return True
    if x_min <= x1 <= x_max and y_min <= y1 <= y_max:
        return True

    dx = x1 - x0
    dy = y1 - y0

    if abs(dx) < 1e-12:
        if x0 < x_min or x0 > x_max:
            return False
        t_xmin, t_xmax = -float("inf"), float("inf")
    else:
        t1 = (x_min - x0) / dx
        t2 = (x_max - x0) / dx
        t_xmin, t_xmax = min(t1, t2), max(t1, t2)

    if abs(dy) < 1e-12:
        if y0 < y_min or y0 > y_max:
            return False
        t_ymin, t_ymax = -float("inf"), float("inf")
    else:
        t1 = (y_min - y0) / dy
        t2 = (y_max - y0) / dy
        t_ymin, t_ymax = min(t1, t2), max(t1, t2)

    t_enter = max(t_xmin, t_ymin)
    t_exit = min(t_xmax, t_ymax)
    return t_enter <= t_exit and max(0.0, t_enter) <= min(1.0, t_exit)


def segment_to_rectangle_distance(
    x0: float, y0: float, x1: float, y1: float,
    x_min: float, x_max: float, y_min: float, y_max: float
) -> float:
    """
    Exact minimum Euclidean distance between line segment (x0, y0)-(x1, y1)
    and solid axis-aligned rectangle [x_min, x_max] x [y_min, y_max].
    """
    if segment_intersects_rectangle(x0, y0, x1, y1, x_min, x_max, y_min, y_max):
        return 0.0

    d0 = point_to_rectangle_distance(x0, y0, x_min, x_max, y_min, y_max)
    d1 = point_to_rectangle_distance(x1, y1, x_min, x_max, y_min, y_max)
    c1 = point_to_segment_distance(x_min, y_min, x0, y0, x1, y1)
    c2 = point_to_segment_distance(x_min, y_max, x0, y0, x1, y1)
    c3 = point_to_segment_distance(x_max, y_min, x0, y0, x1, y1)
    c4 = point_to_segment_distance(x_max, y_max, x0, y0, x1, y1)

    return min(d0, d1, c1, c2, c3, c4)


@dataclass
class World:
    """
    Represents the simulation environment and its discrete 2D occupancy grid.

    Public Coordinates:
        - (gx, gy) for discrete grid coordinates.
        - (wx, wy) for continuous world coordinates.
    NumPy Indexing:
        - self.grid[gy, gx]
    """
    width: float
    height: float
    grid_resolution: float = 10.0
    obstacles: List[Obstacle] = field(default_factory=list)
    robot_radius: float = 15.0
    dynamic_obstacles: List[Any] = field(default_factory=list)

    grid_width: int = field(init=False)
    grid_height: int = field(init=False)
    grid: np.ndarray = field(init=False)
    clearance_grid: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        """Initialize grid dimensions, validate radius, and rasterize obstacles."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError(
                f"World dimensions must be strictly positive, got width={self.width}, height={self.height}"
            )
        if self.grid_resolution <= 0:
            raise ValueError(
                f"Grid resolution must be strictly positive, got {self.grid_resolution}"
            )
        if not isinstance(self.robot_radius, (int, float)) or not math.isfinite(self.robot_radius) or self.robot_radius < 0.0:
            raise ValueError(
                f"Robot radius must be a non-negative finite number, got {self.robot_radius}"
            )
        self.robot_radius = float(self.robot_radius)

        self.grid_width = int(self.width // self.grid_resolution)
        self.grid_height = int(self.height // self.grid_resolution)

        if self.grid_width <= 0 or self.grid_height <= 0:
            raise ValueError(
                f"Derived grid dimensions ({self.grid_width}x{self.grid_height}) must be positive. "
                f"World ({self.width}x{self.height}) with resolution {self.grid_resolution}."
            )

        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        self.clearance_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        self.rasterize_all_obstacles()
        self.update_clearance_grid()

    def __setattr__(self, name: str, value: Any) -> None:
        """Validate attribute mutations such as robot_radius and recompute clearance grid."""
        if name == "robot_radius":
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"Robot radius must be a non-negative finite number, got {value}"
                )
            super().__setattr__(name, float(value))
            if hasattr(self, "clearance_grid") and hasattr(self, "grid"):
                self.update_clearance_grid()
            return
        super().__setattr__(name, value)

    # -------------------------------------------------------------------------
    # Coordinate & Boundary Methods
    # -------------------------------------------------------------------------

    def is_inside_world(self, wx: float, wy: float) -> bool:
        """Check if continuous world coordinates fall within [0, width) x [0, height)."""
        return 0.0 <= wx < self.width and 0.0 <= wy < self.height

    def is_inside_grid(self, gx: int, gy: int) -> bool:
        """Check if discrete grid coordinates fall within [0, grid_width) x [0, grid_height)."""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """
        Convert continuous world coordinates (wx, wy) to discrete grid coordinates (gx, gy).

        Raises:
            ValueError: If (wx, wy) is outside world boundaries.
        """
        if not self.is_inside_world(wx, wy):
            raise ValueError(
                f"World coordinates ({wx}, {wy}) are out of bounds for world size ({self.width}, {self.height})"
            )
        gx = int(wx // self.grid_resolution)
        gy = int(wy // self.grid_resolution)
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """
        Convert discrete grid coordinates (gx, gy) to continuous world cell center (wx, wy).

        Raises:
            IndexError: If (gx, gy) is outside the grid bounds.
        """
        if not self.is_inside_grid(gx, gy):
            raise IndexError(
                f"Grid coordinates ({gx}, {gy}) are out of bounds for grid shape ({self.grid_width}, {self.grid_height})"
            )
        wx = (gx + 0.5) * self.grid_resolution
        wy = (gy + 0.5) * self.grid_resolution
        return (wx, wy)

    # -------------------------------------------------------------------------
    # Occupancy & Traversability
    # -------------------------------------------------------------------------

    def is_occupied(self, gx: int, gy: int) -> bool:
        """
        Check if grid cell (gx, gy) is occupied by an obstacle.

        Raises:
            IndexError: If (gx, gy) is out of bounds.
        """
        if not self.is_inside_grid(gx, gy):
            raise IndexError(
                f"Grid coordinates ({gx}, {gy}) out of bounds for grid shape ({self.grid_width}, {self.grid_height})"
            )
        return bool(self.grid[gy, gx] == CELL_OCCUPIED)

    def is_traversable(self, gx: int, gy: int) -> bool:
        """
        Check if grid cell (gx, gy) is free and traversable.

        Raises:
            IndexError: If (gx, gy) is out of bounds.
        """
        return not self.is_occupied(gx, gy)

    def is_world_occupied(self, wx: float, wy: float) -> bool:
        """Check if continuous world position (wx, wy) maps to an occupied cell."""
        gx, gy = self.world_to_grid(wx, wy)
        return self.is_occupied(gx, gy)

    def is_world_traversable(self, wx: float, wy: float) -> bool:
        """Check if continuous world position (wx, wy) maps to a traversable cell."""
        gx, gy = self.world_to_grid(wx, wy)
        return self.is_traversable(gx, gy)

    # -------------------------------------------------------------------------
    # Obstacle Management & Rasterization
    # -------------------------------------------------------------------------

    def add_obstacle_cell(self, gx: int, gy: int) -> None:
        """
        Directly mark a grid cell (gx, gy) as occupied.

        Raises:
            IndexError: If (gx, gy) is out of bounds.
        """
        if not self.is_inside_grid(gx, gy):
            raise IndexError(
                f"Cannot add obstacle to out-of-bounds cell ({gx}, {gy}) on grid ({self.grid_width}, {self.grid_height})"
            )
        self.grid[gy, gx] = CELL_OCCUPIED
        self.update_clearance_grid()

    def remove_obstacle_cell(self, gx: int, gy: int) -> None:
        """
        Directly clear an obstacle from grid cell (gx, gy), making it free.

        Raises:
            IndexError: If (gx, gy) is out of bounds.
        """
        if not self.is_inside_grid(gx, gy):
            raise IndexError(
                f"Cannot remove obstacle from out-of-bounds cell ({gx}, {gy}) on grid ({self.grid_width}, {self.grid_height})"
            )
        self.grid[gy, gx] = CELL_FREE
        self.update_clearance_grid()

    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Add an Obstacle object to the world and rasterize it onto the occupancy grid."""
        self.obstacles.append(obstacle)
        self._rasterize_obstacle(obstacle)
        self.update_clearance_grid()

    def remove_obstacle_by_id(self, obstacle_id: int) -> bool:
        """
        Remove an obstacle by ID and rebuild the occupancy grid.

        Returns:
            True if an obstacle with obstacle_id was found and removed, False otherwise.
        """
        initial_count = len(self.obstacles)
        self.obstacles = [obs for obs in self.obstacles if obs.id != obstacle_id]
        if len(self.obstacles) < initial_count:
            self.rebuild_grid()
            return True
        return False

    def rebuild_grid(self) -> None:
        """Clear the grid and rasterize all current obstacles."""
        self.grid.fill(CELL_FREE)
        self.rasterize_all_obstacles()
        self.update_clearance_grid()

    def rasterize_all_obstacles(self) -> None:
        """Rasterize all obstacles currently in self.obstacles onto self.grid."""
        for obs in self.obstacles:
            self._rasterize_obstacle(obs)

    def clone(self) -> World:
        """Create a deep copy of the World instance including grid arrays and obstacles."""
        from autonomous_nav.dynamic_obstacle import DynamicObstacle

        w = World(
            width=self.width,
            height=self.height,
            grid_resolution=self.grid_resolution,
            robot_radius=self.robot_radius,
        )
        w.grid = np.copy(self.grid)
        w.clearance_grid = np.copy(self.clearance_grid)
        w.obstacles = [
            Obstacle(
                id=obs.id,
                obstacle_type=obs.obstacle_type,
                position=obs.position,
                radius=obs.radius,
                width=obs.width,
                height=obs.height,
            )
            for obs in self.obstacles
        ]
        w.dynamic_obstacles = [
            DynamicObstacle(
                id=d.id,
                radius=d.radius,
                start_position=d.start_position,
                end_position=d.end_position,
                speed=d.speed,
                current_time=d.current_time,
                obstacle_type=d.obstacle_type,
            )
            for d in self.dynamic_obstacles
        ]
        return w

    def step_dynamic_obstacles(self, dt: float) -> None:
        """Advance all configured dynamic obstacles by simulation timestep dt."""
        for dyn in self.dynamic_obstacles:
            dyn.step(dt)

    def reset_dynamic_obstacles(self) -> None:
        """Reset all configured dynamic obstacles to t=0."""
        for dyn in self.dynamic_obstacles:
            dyn.reset()

    def update_clearance_grid(self) -> None:
        """
        Derive the inflated clearance grid from the raw occupancy grid self.grid
        and self.robot_radius.

        A cell (gx, gy) is marked:
        - CELL_OCCUPIED (1) if it contains an actual obstacle cell in self.grid.
        - CELL_CLEARANCE_VIOLATION (2) if placing the robot center at that cell would:
            a) violate world boundaries with its circular footprint of radius R.
            b) overlap any occupied obstacle cell rectangle in self.grid.
        - CELL_FREE (0) if the robot center at that cell has full clearance.

        This derives a fresh clearance map without repeatedly inflating obstacles.
        """
        self.clearance_grid.fill(CELL_FREE)
        self.clearance_grid[self.grid == CELL_OCCUPIED] = CELL_OCCUPIED

        if self.robot_radius <= 0.0:
            return

        R = self.robot_radius
        res = self.grid_resolution

        # 1. World boundary clearance inflation
        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                wx = (gx + 0.5) * res
                wy = (gy + 0.5) * res
                if (wx - R < 0.0 or wx + R > self.width or
                        wy - R < 0.0 or wy + R > self.height):
                    if self.clearance_grid[gy, gx] == CELL_FREE:
                        self.clearance_grid[gy, gx] = CELL_CLEARANCE_VIOLATION

        # 2. Obstacle clearance inflation
        occupied_indices = np.argwhere(self.grid == CELL_OCCUPIED)
        if len(occupied_indices) == 0:
            return

        k_max = int(math.ceil(R / res))
        R_sq = R * R

        for oy, ox in occupied_indices:
            for dy in range(-k_max, k_max + 1):
                for dx in range(-k_max, k_max + 1):
                    gy = oy + dy
                    gx = ox + dx
                    if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                        if self.clearance_grid[gy, gx] == CELL_FREE:
                            # Exact continuous distance from cell center (gx, gy) to cell (ox, oy) rectangle
                            dist_x = max(0.0, abs(gx - ox) - 0.5) * res
                            dist_y = max(0.0, abs(gy - oy) - 0.5) * res
                            if dist_x * dist_x + dist_y * dist_y <= R_sq:
                                self.clearance_grid[gy, gx] = CELL_CLEARANCE_VIOLATION

    def _rasterize_obstacle(self, obstacle: Obstacle) -> None:
        """Rasterize a single geometric obstacle onto the discrete grid."""
        otype = obstacle.obstacle_type.lower()
        if otype == "circle":
            cx, cy = obstacle.position
            r = obstacle.radius

            # Compute bounding box in grid coordinates
            min_gx = max(0, int(math.floor((cx - r) / self.grid_resolution)))
            max_gx = min(self.grid_width - 1, int(math.floor((cx + r) / self.grid_resolution)))
            min_gy = max(0, int(math.floor((cy - r) / self.grid_resolution)))
            max_gy = min(self.grid_height - 1, int(math.floor((cy + r) / self.grid_resolution)))

            for gy in range(min_gy, max_gy + 1):
                for gx in range(min_gx, max_gx + 1):
                    wx, wy = self.grid_to_world(gx, gy)
                    if (wx - cx) ** 2 + (wy - cy) ** 2 <= r ** 2:
                        self.grid[gy, gx] = CELL_OCCUPIED

            # Ensure anchor center cell is occupied if inside bounds
            if self.is_inside_world(cx, cy):
                cgx, cgy = self.world_to_grid(cx, cy)
                self.grid[cgy, cgx] = CELL_OCCUPIED

        elif otype == "rectangle":
            ox, oy = obstacle.position
            w, h = obstacle.width, obstacle.height

            min_gx = max(0, int(math.floor(ox / self.grid_resolution)))
            max_gx = min(self.grid_width - 1, int(math.floor((ox + w) / self.grid_resolution)))
            min_gy = max(0, int(math.floor(oy / self.grid_resolution)))
            max_gy = min(self.grid_height - 1, int(math.floor((oy + h) / self.grid_resolution)))

            for gy in range(min_gy, max_gy + 1):
                for gx in range(min_gx, max_gx + 1):
                    wx, wy = self.grid_to_world(gx, gy)
                    if ox <= wx <= (ox + w) and oy <= wy <= (oy + h):
                        self.grid[gy, gx] = CELL_OCCUPIED

            # Ensure anchor cell is occupied if inside bounds
            if self.is_inside_world(ox, oy):
                ogx, ogy = self.world_to_grid(ox, oy)
                self.grid[ogy, ogx] = CELL_OCCUPIED

    # -------------------------------------------------------------------------
    # Footprint-Aware Collision Checking APIs
    # -------------------------------------------------------------------------

    def check_footprint_collision(
        self,
        wx: float,
        wy: float,
        radius: Optional[float] = None
    ) -> Tuple[bool, str]:
        """
        Check whether a robot circular footprint centered at continuous world position
        (wx, wy) is completely collision-free from both world boundaries and obstacles.

        Accounts for the full 2D area occupied by grid cells (not only their center points).

        Args:
            wx: Continuous X position of the robot center.
            wy: Continuous Y position of the robot center.
            radius: Circular footprint radius in world units. Defaults to self.robot_radius.

        Returns:
            Tuple of (is_safe: bool, reason: str).
            is_safe is True if the entire footprint is clear, False if unsafe.

        Raises:
            ValueError: If radius is negative or non-finite.
        """
        r = self.robot_radius if radius is None else radius
        if not isinstance(r, (int, float)) or not math.isfinite(r) or r < 0.0:
            raise ValueError(f"Robot radius must be a non-negative finite number, got: {r}")
        r = float(r)

        # Numerical validation of position
        if not (isinstance(wx, (int, float)) and isinstance(wy, (int, float))):
            return (False, f"Position coordinates must be real numbers, got ({wx}, {wy})")
        if not (math.isfinite(wx) and math.isfinite(wy)):
            return (False, f"Position coordinates are non-finite: ({wx}, {wy})")

        wx = float(wx)
        wy = float(wy)

        # 1. World boundary clearance check
        if wx - r < 0.0 or wx + r > self.width or wy - r < 0.0 or wy + r > self.height:
            return (
                False,
                f"Footprint (radius={r:.1f}) extends outside world bounds "
                f"[0, {self.width:.1f}] x [0, {self.height:.1f}] at ({wx:.2f}, {wy:.2f})."
            )

        if r == 0.0:
            # Point robot check
            gx, gy = self.world_to_grid(wx, wy)
            if self.is_occupied(gx, gy):
                return (False, f"Point position ({wx:.2f}, {wy:.2f}) maps to occupied cell ({gx}, {gy}).")
            return (True, "Footprint is clear and safe.")

        # 2. Obstacle area overlap check
        res = self.grid_resolution
        gx_min = max(0, int(math.floor((wx - r) / res)))
        gx_max = min(self.grid_width - 1, int(math.floor((wx + r) / res)))
        gy_min = max(0, int(math.floor((wy - r) / res)))
        gy_max = min(self.grid_height - 1, int(math.floor((wy + r) / res)))

        r_sq = r * r
        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                if self.grid[gy, gx] == CELL_OCCUPIED:
                    # Closest point on the cell rectangle [gx*res, (gx+1)*res] x [gy*res, (gy+1)*res]
                    x1 = gx * res
                    x2 = (gx + 1) * res
                    y1 = gy * res
                    y2 = (gy + 1) * res
                    cx = min(max(wx, x1), x2)
                    cy = min(max(wy, y1), y2)
                    if (wx - cx) ** 2 + (wy - cy) ** 2 <= r_sq:
                        return (
                            False,
                            f"Footprint (radius={r:.1f}) overlaps occupied obstacle cell "
                            f"({gx}, {gy}) at ({wx:.2f}, {wy:.2f})."
                        )

        return (True, "Footprint is clear and safe.")

    def is_footprint_safe(
        self,
        wx: float,
        wy: float,
        radius: Optional[float] = None
    ) -> bool:
        """
        Check whether a robot circular footprint at (wx, wy) is safe.

        Returns:
            True if collision-free, False otherwise.
        """
        is_safe, _ = self.check_footprint_collision(wx, wy, radius)
        return is_safe

    def is_footprint_safe_grid(
        self,
        gx: int,
        gy: int,
        radius: Optional[float] = None
    ) -> bool:
        """
        Check whether discrete grid cell (gx, gy) is safe for placing the robot center.

        Returns:
            True if cell is in bounds and footprint-safe, False otherwise.
        """
        if not self.is_inside_grid(gx, gy):
            return False
        if radius is None or radius == self.robot_radius:
            return bool(self.clearance_grid[gy, gx] == CELL_FREE)
        wx, wy = self.grid_to_world(gx, gy)
        return self.is_footprint_safe(wx, wy, radius)

    def check_swept_footprint_collision(
        self,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        radius: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Check whether a robot circular footprint moving along the continuous line segment
        from (x0, y0) to (x1, y1) remains collision-free throughout the entire movement.

        Accounts for:
        1. Full circular footprint radius throughout the continuous movement (swept capsule).
        2. Every occupied grid cell treated as a solid 2D rectangle.
        3. World-boundary clearance throughout the segment.
        4. Zero-length movements (x0 == x1 and y0 == y1).
        5. Exact geometric segment-to-rectangle distance calculations.
        6. Tangential contact policy: touching an obstacle boundary or world boundary
           (distance <= radius) is conservatively rejected as a collision.

        Args:
            x0: Starting continuous X position of robot center.
            y0: Starting continuous Y position of robot center.
            x1: Target continuous X position of robot center.
            y1: Target continuous Y position of robot center.
            radius: Circular footprint radius. Defaults to self.robot_radius.

        Returns:
            Tuple of (is_safe: bool, reason: str).
            is_safe is True if the swept footprint is strictly collision-free.
            is_safe is False if the swept footprint intersects or touches any obstacle or boundary.

        Raises:
            ValueError: If radius is negative or non-finite.
        """
        r = self.robot_radius if radius is None else radius
        if not isinstance(r, (int, float)) or not math.isfinite(r) or r < 0.0:
            raise ValueError(f"Robot radius must be a non-negative finite number, got: {r}")
        r = float(r)

        # Numerical validation of position coordinates
        for name, coord in (("x0", x0), ("y0", y0), ("x1", x1), ("y1", y1)):
            if not (isinstance(coord, (int, float)) and math.isfinite(coord)):
                return (False, f"Position coordinate '{name}' is non-finite or invalid: {coord}")

        x0 = float(x0)
        y0 = float(y0)
        x1 = float(x1)
        y1 = float(y1)

        min_x = min(x0, x1)
        max_x = max(x0, x1)
        min_y = min(y0, y1)
        max_y = max(y0, y1)

        # 1. World boundary clearance check along entire segment
        # Conservative policy: touching the boundary (distance <= radius) is rejected
        if r > 0.0:
            boundary_violation = (
                min_x - r <= 0.0
                or max_x + r >= self.width
                or min_y - r <= 0.0
                or max_y + r >= self.height
            )
        else:
            boundary_violation = (
                min_x < 0.0
                or max_x >= self.width
                or min_y < 0.0
                or max_y >= self.height
            )

        if boundary_violation:
            return (
                False,
                f"Swept footprint (radius={r:.1f}) extends outside or touches world bounds "
                f"[0, {self.width:.1f}] x [0, {self.height:.1f}] along segment "
                f"({x0:.2f}, {y0:.2f}) -> ({x1:.2f}, {y1:.2f})."
            )

        # 2. Obstacle area overlap check along entire segment
        res = self.grid_resolution
        gx_min = max(0, int(math.floor((min_x - r) / res)))
        gx_max = min(self.grid_width - 1, int(math.floor((max_x + r) / res)))
        gy_min = max(0, int(math.floor((min_y - r) / res)))
        gy_max = min(self.grid_height - 1, int(math.floor((max_y + r) / res)))

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                if self.grid[gy, gx] == CELL_OCCUPIED:
                    rx_min = gx * res
                    rx_max = (gx + 1) * res
                    ry_min = gy * res
                    ry_max = (gy + 1) * res

                    dist = segment_to_rectangle_distance(
                        x0, y0, x1, y1, rx_min, rx_max, ry_min, ry_max
                    )
                    # Conservative policy: touching (dist <= r) is rejected as collision
                    if dist <= r:
                        return (
                            False,
                            f"Swept footprint (radius={r:.1f}) collides with or grazes occupied obstacle cell "
                            f"({gx}, {gy}) (dist={dist:.2f} <= {r:.1f}) along segment "
                            f"({x0:.2f}, {y0:.2f}) -> ({x1:.2f}, {y1:.2f})."
                        )

        return (True, "Swept footprint is clear and safe.")

    def is_swept_footprint_safe(
        self,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        radius: Optional[float] = None,
    ) -> bool:
        """
        Check whether swept circular footprint along segment from (x0, y0) to (x1, y1)
        is completely collision-free.

        Returns:
            True if collision-free, False otherwise.
        """
        is_safe, _ = self.check_swept_footprint_collision(x0, y0, x1, y1, radius)
        return is_safe

    # -------------------------------------------------------------------------
    # Position Validation (Start & Goal)
    # -------------------------------------------------------------------------

    def validate_position(
        self,
        pos: Tuple[float, float],
        name: str = "Position",
        check_footprint: bool = False,
        radius: Optional[float] = None,
    ) -> Tuple[int, int]:
        """
        Validate that continuous coordinates pos=(x, y) lie inside bounds and are not occupied.
        Optionally validates circular footprint clearance.

        Returns:
            The corresponding discrete grid coordinates (gx, gy).

        Raises:
            ValueError: If position is outside bounds, inside an obstacle, or lacks footprint clearance.
        """
        if not (isinstance(pos, (list, tuple)) and len(pos) == 2):
            raise ValueError(f"{name} must be a 2-element coordinate pair (x, y), got {pos}")

        x, y = float(pos[0]), float(pos[1])
        if not self.is_inside_world(x, y):
            raise ValueError(
                f"{name} ({x}, {y}) is outside world bounds [0, {self.width}) x [0, {self.height})"
            )

        gx, gy = self.world_to_grid(x, y)
        if self.is_occupied(gx, gy):
            raise ValueError(
                f"{name} ({x}, {y}) maps to grid cell ({gx}, {gy}), which is occupied by an obstacle."
            )

        if check_footprint:
            is_safe, reason = self.check_footprint_collision(x, y, radius=radius)
            if not is_safe:
                raise ValueError(
                    f"{name} ({x}, {y}) does not have adequate robot footprint clearance: {reason}"
                )

        return (gx, gy)

    def validate_start_goal(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        check_footprint: bool = False,
        radius: Optional[float] = None,
    ) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """
        Validate both start and goal positions, ensuring neither is out of bounds,
        neither is inside an obstacle, neither violates footprint clearance,
        and they do not map to the exact same cell.

        Returns:
            Tuple containing ((start_gx, start_gy), (goal_gx, goal_gy)).

        Raises:
            ValueError: If either position is invalid or if start equals goal.
        """
        start_grid = self.validate_position(
            start, name="Start position", check_footprint=check_footprint, radius=radius
        )
        goal_grid = self.validate_position(
            goal, name="Goal position", check_footprint=check_footprint, radius=radius
        )

        if start_grid == goal_grid:
            raise ValueError(
                f"Start position {start} and goal position {goal} map to the identical grid cell {start_grid}."
            )
        return (start_grid, goal_grid)

    # -------------------------------------------------------------------------
    # Factory Constructor
    # -------------------------------------------------------------------------

    @classmethod
    def from_config(cls, config_data: Dict[str, Any]) -> World:
        """Create a World instance from a scenario configuration dictionary."""
        world_cfg = config_data.get("world", {})
        width = float(world_cfg.get("width", 800))
        height = float(world_cfg.get("height", 600))
        grid_resolution = float(world_cfg.get("grid_resolution", 10))

        robot_cfg = config_data.get("robot", {})
        robot_radius = float(robot_cfg.get("radius", 15.0))

        raw_obstacles = config_data.get("obstacles", [])
        obstacles: List[Obstacle] = []
        for obs in raw_obstacles:
            pos = tuple(obs.get("position", [0.0, 0.0]))
            obstacles.append(
                Obstacle(
                    id=int(obs.get("id", 0)),
                    obstacle_type=str(obs.get("type", "unknown")),
                    position=(float(pos[0]), float(pos[1])),
                    radius=float(obs.get("radius", 0.0)),
                    width=float(obs.get("width", 0.0)),
                    height=float(obs.get("height", 0.0)),
                )
            )

        raw_dyn = config_data.get("dynamic_obstacles", [])
        from autonomous_nav.dynamic_obstacle import DynamicObstacle
        dyn_obstacles: List[DynamicObstacle] = [
            DynamicObstacle.from_dict(d) for d in raw_dyn
        ]

        return cls(
            width=width,
            height=height,
            grid_resolution=grid_resolution,
            obstacles=obstacles,
            robot_radius=robot_radius,
            dynamic_obstacles=dyn_obstacles,
        )
