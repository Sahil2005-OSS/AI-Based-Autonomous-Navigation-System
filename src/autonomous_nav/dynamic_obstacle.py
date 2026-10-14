"""
Dynamic Obstacles and Deterministic Trajectory Modeling.

Provides deterministic moving obstacle representation for 2D simulation,
route conflict detection with configurable safety margins, and temporary
planning world generation for safe A* replanning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple
import numpy as np

from autonomous_nav.world import Obstacle, World


def point_to_segment_distance(
    px: float, py: float, x1: float, y1: float, x2: float, y2: float
) -> float:
    """
    Calculate the shortest Euclidean distance from a 2D point (px, py)
    to a finite line segment [(x1, y1), (x2, y2)].

    Args:
        px, py: Query point coordinates.
        x1, y1: Segment start point coordinates.
        x2, y2: Segment end point coordinates.

    Returns:
        Shortest Euclidean distance in world units.
    """
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq < 1e-12:
        return math.hypot(px - x1, py - y1)

    # Project point onto segment parameter t in [0, 1]
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))

    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


class DynamicObstacle:
    """
    Deterministic moving obstacle with bounded oscillatory or periodic trajectory.

    The obstacle moves back and forth along a bounded continuous segment
    between start_position and end_position at a constant configured speed.
    All position updates are functions of simulation time, never wall-clock time.

    Attributes:
        id: Unique identifier for the obstacle.
        radius: Circular obstacle radius in world units.
        start_position: Tuple (x, y) defining patrol start endpoint.
        end_position: Tuple (x, y) defining patrol end endpoint.
        speed: Linear translation speed in world units per simulated second (>= 0).
        current_time: Simulated time in seconds elapsed since scenario start.
        obstacle_type: Geometry type identifier ('circle').
    """

    id: int
    radius: float
    start_position: Tuple[float, float]
    end_position: Tuple[float, float]
    speed: float
    current_time: float
    obstacle_type: str

    def __init__(
        self,
        id: Optional[int] = None,
        radius: float = 10.0,
        start_position: Optional[Tuple[float, float]] = None,
        end_position: Optional[Tuple[float, float]] = None,
        speed: float = 20.0,
        current_time: float = 0.0,
        obstacle_type: str = "circle",
        *,
        obstacle_id: Optional[int] = None,
        point_a: Optional[Tuple[float, float]] = None,
        point_b: Optional[Tuple[float, float]] = None,
    ) -> None:
        actual_id = id if id is not None else (obstacle_id if obstacle_id is not None else 1)
        actual_start = start_position if start_position is not None else (point_a if point_a is not None else (0.0, 0.0))
        actual_end = end_position if end_position is not None else (point_b if point_b is not None else (0.0, 0.0))

        if not (isinstance(radius, (int, float)) and math.isfinite(radius) and radius > 0.0):
            raise ValueError(f"Radius must be a positive finite number, got {radius}")
        if not (isinstance(speed, (int, float)) and math.isfinite(speed) and speed >= 0.0):
            raise ValueError(f"Speed must be a non-negative finite number, got {speed}")

        self.id = int(actual_id)
        self.radius = float(radius)
        self.start_position = (float(actual_start[0]), float(actual_start[1]))
        self.end_position = (float(actual_end[0]), float(actual_end[1]))
        self.speed = float(speed)
        self.current_time = float(current_time)
        self.obstacle_type = str(obstacle_type)

    @property
    def point_a(self) -> Tuple[float, float]:
        """Alias for start_position."""
        return self.start_position

    @property
    def point_b(self) -> Tuple[float, float]:
        """Alias for end_position."""
        return self.end_position

    def is_point_colliding(self, px: float, py: float, query_time: Optional[float] = None) -> bool:
        """Check if query point falls within obstacle circular footprint."""
        return self.check_collision_circle(px, py, other_radius=0.0, query_time=query_time)

    def is_circle_colliding(
        self,
        cx: float,
        cy: float,
        other_radius: float,
        margin: float = 0.0,
        query_time: Optional[float] = None,
    ) -> bool:
        """Check if circular footprint collides with obstacle footprint."""
        return self.check_collision_circle(cx, cy, other_radius=other_radius, margin=margin, query_time=query_time)

    @property
    def current_position(self) -> Tuple[float, float]:
        """Return the obstacle center position at the current simulated time."""
        return self.position_at(self.current_time)

    def position_at(self, t: float) -> Tuple[float, float]:
        """
        Evaluate deterministic center position (x, y) at simulated time t.

        Uses ping-pong oscillatory linear interpolation along [start, end].
        If speed == 0 or endpoints are identical, remains stationary at start.

        Args:
            t: Simulated time in seconds (must be non-negative and finite).

        Returns:
            Tuple of (x, y) continuous world coordinates.
        """
        if not (isinstance(t, (int, float)) and math.isfinite(t) and t >= 0.0):
            raise ValueError(f"Simulation time t must be non-negative finite number, got {t}")

        x0, y0 = self.start_position
        x1, y1 = self.end_position
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-9 or self.speed <= 0.0:
            return (x0, y0)

        # One full back-and-forth cycle distance is 2 * seg_len
        cycle_dist = 2.0 * seg_len
        traveled = (self.speed * t) % cycle_dist

        if traveled <= seg_len:
            ratio = traveled / seg_len
        else:
            ratio = (cycle_dist - traveled) / seg_len

        x = x0 + ratio * dx
        y = y0 + ratio * dy
        return (x, y)

    def step(self, dt: float) -> Tuple[float, float]:
        """
        Advance simulated time by dt and return the new center position.

        Args:
            dt: Time increment in seconds (must be positive and finite).

        Returns:
            New (x, y) center position.
        """
        if not (isinstance(dt, (int, float)) and math.isfinite(dt) and dt > 0.0):
            raise ValueError(f"Timestep dt must be positive finite number, got {dt}")
        self.current_time += dt
        return self.current_position

    def reset(self) -> None:
        """Reset simulated time to zero."""
        self.current_time = 0.0

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        Return the strict spatial bounding envelope for this dynamic obstacle
        including its full circular radius throughout its motion:
        (min_x, min_y, max_x, max_y).
        """
        x0, y0 = self.start_position
        x1, y1 = self.end_position
        min_x = min(x0, x1) - self.radius
        max_x = max(x0, x1) + self.radius
        min_y = min(y0, y1) - self.radius
        max_y = max(y0, y1) + self.radius
        return (min_x, min_y, max_x, max_y)

    def check_collision_circle(
        self,
        cx: float,
        cy: float,
        other_radius: float,
        margin: float = 0.0,
        query_time: Optional[float] = None,
    ) -> bool:
        """
        Check if an external circle (cx, cy, other_radius) intersects or contacts
        this dynamic obstacle at query_time (defaults to current_time).

        Conservative policy: distance <= (radius + other_radius + margin) is True.
        """
        ox, oy = self.current_position if query_time is None else self.position_at(query_time)
        dist = math.hypot(cx - ox, cy - oy)
        return dist <= (self.radius + other_radius + margin)

    def distance_to_point(self, px: float, py: float, query_time: Optional[float] = None) -> float:
        """Calculate Euclidean surface distance from query point to circular obstacle."""
        ox, oy = self.current_position if query_time is None else self.position_at(query_time)
        return max(0.0, math.hypot(px - ox, py - oy) - self.radius)

    def distance_to_segment(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        query_time: Optional[float] = None,
    ) -> float:
        """Calculate Euclidean surface distance from a segment to circular obstacle."""
        ox, oy = self.current_position if query_time is None else self.position_at(query_time)
        center_dist = point_to_segment_distance(ox, oy, x1, y1, x2, y2)
        return max(0.0, center_dist - self.radius)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize dynamic obstacle to configuration dictionary."""
        return {
            "id": self.id,
            "type": self.obstacle_type,
            "radius": self.radius,
            "start_position": list(self.start_position),
            "end_position": list(self.end_position),
            "speed": self.speed,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> DynamicObstacle:
        """Construct DynamicObstacle from dictionary."""
        start = tuple(data.get("start_position", [0.0, 0.0]))
        end = tuple(data.get("end_position", start))
        return cls(
            id=int(data.get("id", 1)),
            radius=float(data.get("radius", 15.0)),
            start_position=start,
            end_position=end,
            speed=float(data.get("speed", 20.0)),
            obstacle_type=str(data.get("type", "circle")),
        )


def check_moving_obstacle_step_collision(
    robot_start: Tuple[float, float],
    robot_end: Tuple[float, float],
    robot_radius: float,
    obstacle: DynamicObstacle,
    t_start: float,
    t_end: float,
    margin: float = 0.0,
) -> Tuple[bool, float, str]:
    """
    Check if continuous robot motion along segment [robot_start, robot_end] during
    simulated time interval [t_start, t_end] collides with a moving dynamic obstacle.

    Evaluates synchronized relative continuous motion between the two bodies.
    For linear motion segments, relative position vector is:
        D(lambda) = (R_start - O_start) + lambda * ((R_end - R_start) - (O_end - O_start))
    The minimum continuous Euclidean distance between their circular centers is the distance
    from the origin (0, 0) to relative segment [P_start, P_end].

    If the obstacle undergoes a trajectory reversal/bounce within [t_start, t_end],
    the interval is partitioned at the reversal point(s) and the minimum distance across
    all sub-intervals is evaluated.

    Args:
        robot_start: Robot center position (x, y) at t_start.
        robot_end: Robot candidate center position (x, y) at t_end.
        robot_radius: Configured robot circular footprint radius.
        obstacle: DynamicObstacle instance.
        t_start: Step start simulation time.
        t_end: Step end simulation time (t_end >= t_start).
        margin: Optional additional clearance margin.

    Returns:
        Tuple of:
            - collided: bool (True if min_dist <= robot_radius + obstacle.radius + margin)
            - min_distance: float (minimum Euclidean distance between centers during step)
            - reason: str (diagnostic message)
    """
    if t_end < t_start:
        raise ValueError(f"t_end ({t_end}) must be >= t_start ({t_start})")

    r_total = robot_radius + obstacle.radius + margin

    if t_end == t_start:
        pos = obstacle.position_at(t_start)
        d = math.hypot(robot_start[0] - pos[0], robot_start[1] - pos[1])
        if d <= r_total:
            return (
                True,
                d,
                f"Collision with dynamic obstacle {obstacle.id} at ({pos[0]:.1f}, {pos[1]:.1f}) (dist={d:.2f} <= {r_total:.2f}).",
            )
        return (False, d, "Clear")

    dx = obstacle.end_position[0] - obstacle.start_position[0]
    dy = obstacle.end_position[1] - obstacle.start_position[1]
    seg_len = math.hypot(dx, dy)

    time_splits = [t_start]
    if seg_len > 1e-9 and obstacle.speed > 0.0:
        t_leg = seg_len / obstacle.speed
        k_min = math.floor(t_start / t_leg + 1e-9)
        k_max = math.ceil(t_end / t_leg - 1e-9)
        for k in range(int(k_min), int(k_max) + 1):
            t_turn = k * t_leg
            if t_start + 1e-9 < t_turn < t_end - 1e-9:
                time_splits.append(t_turn)
    time_splits.append(t_end)
    time_splits = sorted(list(set(time_splits)))

    overall_min_dist = float("inf")
    duration = t_end - t_start

    for i in range(len(time_splits) - 1):
        t0 = time_splits[i]
        t1 = time_splits[i + 1]

        o0 = obstacle.position_at(t0)
        o1 = obstacle.position_at(t1)

        s0 = (t0 - t_start) / duration
        s1 = (t1 - t_start) / duration
        r0 = (
            robot_start[0] + s0 * (robot_end[0] - robot_start[0]),
            robot_start[1] + s0 * (robot_end[1] - robot_start[1]),
        )
        r1 = (
            robot_start[0] + s1 * (robot_end[0] - robot_start[0]),
            robot_start[1] + s1 * (robot_end[1] - robot_start[1]),
        )

        p0 = (r0[0] - o0[0], r0[1] - o0[1])
        p1 = (r1[0] - o1[0], r1[1] - o1[1])

        dist_seg = point_to_segment_distance(0.0, 0.0, p0[0], p0[1], p1[0], p1[1])
        overall_min_dist = min(overall_min_dist, dist_seg)

        if overall_min_dist <= r_total:
            mid_obs = obstacle.position_at((t0 + t1) * 0.5)
            return (
                True,
                overall_min_dist,
                f"Swept footprint collides with moving dynamic obstacle {obstacle.id} around ({mid_obs[0]:.1f}, {mid_obs[1]:.1f}) "
                f"(dist={overall_min_dist:.2f} <= {r_total:.2f}).",
            )

    return (False, overall_min_dist, "Clear")


def check_route_conflict(
    route_world_points: Optional[Sequence[Tuple[float, float]]] = None,
    dynamic_obstacles: Optional[Sequence[DynamicObstacle]] = None,
    robot_radius: float = 10.0,
    safety_margin: float = 5.0,
    current_time: Optional[float] = None,
    *,
    route_points: Optional[Sequence[Tuple[float, float]]] = None,
) -> Tuple[bool, Optional[DynamicObstacle], Optional[int]]:
    """
    Check if a sequence of remaining path waypoints in world coordinates conflicts
    with any dynamic obstacle at current_time.

    Evaluates distance from each dynamic obstacle to each route segment.
    A conflict is flagged if:
        segment_to_center_distance <= (robot_radius + obstacle_radius + safety_margin)

    Args:
        route_world_points: Ordered list of continuous world coordinates [(wx, wy), ...].
        dynamic_obstacles: Sequence of DynamicObstacle instances.
        robot_radius: Configured robot circular footprint radius.
        safety_margin: Additional buffer distance in world units.
        current_time: Optional simulated time query.
        route_points: Optional alias for route_world_points.

    Returns:
        Tuple of:
            - is_in_conflict: bool
            - conflicting_obstacle: Optional[DynamicObstacle]
            - conflicting_segment_index: Optional[int] (index i of segment [i, i+1])
    """
    pts = route_world_points if route_world_points is not None else route_points
    if not pts or len(pts) < 2 or not dynamic_obstacles:
        return (False, None, None)

    for dyn in dynamic_obstacles:
        ox, oy = dyn.current_position if current_time is None else dyn.position_at(current_time)
        threshold = robot_radius + dyn.radius + safety_margin

        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]

            dist = point_to_segment_distance(ox, oy, x1, y1, x2, y2)
            if dist <= threshold:
                return (True, dyn, i)

    return (False, None, None)


def create_temporary_planning_world(
    base_world: World,
    dynamic_obstacles: Sequence[DynamicObstacle],
    safety_margin: float = 5.0,
    current_time: Optional[float] = None,
) -> World:
    """
    Create a temporary planning World instance where dynamic obstacles at their
    current positions are rasterized and inflated with the safety margin, without
    corrupting the base static World map.

    Args:
        base_world: The original static World environment.
        dynamic_obstacles: Dynamic obstacles to overlay.
        safety_margin: Additional clearance buffer around moving obstacles.
        current_time: Simulated time for obstacle position query.

    Returns:
        A cloned World instance containing static obstacles and dynamic obstacle overlays.
    """
    temp_world = World(
        width=base_world.width,
        height=base_world.height,
        grid_resolution=base_world.grid_resolution,
        robot_radius=base_world.robot_radius,
    )
    # Deep copy underlying static grid and clearance grid
    temp_world.grid = np.copy(base_world.grid)
    temp_world.clearance_grid = np.copy(base_world.clearance_grid)
    temp_world.obstacles = [
        Obstacle(
            id=obs.id,
            obstacle_type=obs.obstacle_type,
            position=obs.position,
            radius=obs.radius,
            width=obs.width,
            height=obs.height,
        )
        for obs in base_world.obstacles
    ]

    # Overlay each dynamic obstacle at its instantaneous continuous position
    for idx, dyn in enumerate(dynamic_obstacles):
        pos = dyn.current_position if current_time is None else dyn.position_at(current_time)
        effective_radius = dyn.radius + safety_margin
        temp_obs = Obstacle(
            id=90000 + idx,
            obstacle_type="circle",
            position=pos,
            radius=effective_radius,
        )
        temp_world.add_obstacle(temp_obs)

    return temp_world


def find_safe_start_grid(
    world: World,
    temp_world: World,
    rx: float,
    ry: float,
    search_radius: int = 3,
) -> Optional[Tuple[int, int]]:
    """
    Find the closest discrete grid cell to continuous position (rx, ry)
    that has valid circular footprint clearance in both base and temp world.

    Args:
        world: Base static World environment.
        temp_world: Temporary planning World environment.
        rx, ry: Robot center continuous world coordinates.
        search_radius: Maximum grid radius to search for a safe cell.

    Returns:
        (gx, gy) grid cell coordinates, or None if no safe cell exists within search_radius.
    """
    if not world.is_inside_world(rx, ry):
        return None

    try:
        gx, gy = world.world_to_grid(rx, ry)
    except ValueError:
        return None

    if (
        temp_world.is_inside_grid(gx, gy)
        and temp_world.is_footprint_safe_grid(gx, gy)
        and world.is_footprint_safe_grid(gx, gy)
    ):
        return (gx, gy)

    # Search outward in expanding rings
    best_cell = None
    best_dist = float("inf")

    for r in range(1, search_radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if (
                    temp_world.is_inside_grid(nx, ny)
                    and temp_world.is_footprint_safe_grid(nx, ny)
                    and world.is_footprint_safe_grid(nx, ny)
                ):
                    wx, wy = temp_world.grid_to_world(nx, ny)
                    dist = math.hypot(rx - wx, ry - wy)
                    if dist < best_dist:
                        best_dist = dist
                        best_cell = (nx, ny)
        if best_cell is not None:
            return best_cell

    return None
