"""
Pure Pursuit Path Tracking Controller and Simulation Runner.

Implements a Pure Pursuit path-following controller that steers a continuous
kinematic unicycle robot along a collision-checked smoothed trajectory, with
per-step obstacle and boundary collision checking.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Tuple

from autonomous_nav.world import World
from autonomous_nav.motion_model import RobotState, KinematicMotionModel, normalize_angle


class ControllerStatus(str, Enum):
    """Execution status returned by the path tracking controller."""

    IDLE = "IDLE"
    TRACKING = "TRACKING"
    REACHED_GOAL = "REACHED_GOAL"
    COLLISION = "COLLISION"
    FAILED = "FAILED"
    BLOCKED = "BLOCKED"


@dataclass
class ControllerOutput:
    """
    Output control commands and telemetry from a single controller update.

    Attributes:
        v_cmd: Commanded linear speed (units/s).
        omega_cmd: Commanded angular rate (rad/s).
        status: Current tracking status.
        lookahead_target: Current lookahead target coordinate in world frame (wx, wy).
        distance_to_goal: Euclidean distance from robot to final path waypoint.
        message: Informational diagnostics string.
    """

    v_cmd: float
    omega_cmd: float
    status: ControllerStatus
    lookahead_target: Optional[Tuple[float, float]] = None
    distance_to_goal: float = float("inf")
    message: str = ""


@dataclass
class SimulationMetrics:
    """
    Summary metrics returned by an end-to-end continuous simulation run.

    Attributes:
        success: True if the goal was reached safely without collisions.
        steps: Total number of discrete integration steps executed.
        elapsed_time: Total simulated time in seconds.
        initial_pose: Initial robot pose (x, y, theta).
        final_pose: Final robot pose (x, y, theta).
        goal_reached: Boolean indicator of whether goal tolerance was satisfied.
        collision_occurred: Boolean indicator of whether collision/OOB occurred.
        max_linear_speed: Maximum commanded linear velocity during run.
        max_angular_speed: Maximum commanded angular velocity magnitude during run.
        status: Final controller status.
        message: Diagnostic summary message.
        distance_traveled: Total Euclidean distance traversed by robot center in world units.
        final_distance_to_goal: Euclidean distance from final robot position to goal waypoint.
        replans_attempted: Number of A* replans initiated due to route conflicts.
        replans_succeeded: Number of successfully planned and validated replacement routes.
        dynamic_obstacles_count: Number of active dynamic obstacles in simulation.
    """

    success: bool
    steps: int
    elapsed_time: float
    initial_pose: Tuple[float, float, float]
    final_pose: Tuple[float, float, float]
    goal_reached: bool
    collision_occurred: bool
    max_linear_speed: float
    max_angular_speed: float
    status: ControllerStatus
    message: str
    distance_traveled: float = 0.0
    final_distance_to_goal: float = 0.0
    replans_attempted: int = 0
    replans_succeeded: int = 0
    dynamic_obstacles_count: int = 0


class PurePursuitController:
    """
    Geometric Pure Pursuit controller for 2D unicycle robots.

    Consumes discrete grid waypoints, transforms them into continuous world
    coordinates via the World model, and calculates velocity commands (v, omega)
    to follow the lookahead point along the path.
    """

    def __init__(
        self,
        world: World,
        lookahead_distance: float = 25.0,
        max_linear_speed: float = 40.0,
        max_angular_speed: float = 2.5,
        goal_tolerance: float = 5.0,
        slowdown_radius: float = 60.0,
        min_linear_speed: float = 2.0,
        check_footprint: bool = True,
        robot_radius: Optional[float] = None,
    ) -> None:
        """
        Initialize the Pure Pursuit controller.

        Args:
            world: Environment World model instance.
            lookahead_distance: Lookahead horizon L_d in world units (> 0).
            max_linear_speed: Upper bound on linear velocity (> 0).
            max_angular_speed: Upper bound on angular velocity magnitude (> 0).
            goal_tolerance: Radial distance to final waypoint to declare goal arrival (> 0).
            slowdown_radius: Radial distance to begin decelerating towards goal (> goal_tolerance).
            min_linear_speed: Minimum cruise speed while tracking (>= 0).
            check_footprint: If True, candidate simulation steps are checked with circular footprint.
            robot_radius: Optional override for robot radius. Defaults to world.robot_radius.

        Raises:
            ValueError: If parameters are non-positive or inconsistent.
        """
        if not math.isfinite(lookahead_distance) or lookahead_distance <= 0:
            raise ValueError(f"lookahead_distance must be positive, got: {lookahead_distance}")
        if not math.isfinite(max_linear_speed) or max_linear_speed <= 0:
            raise ValueError(f"max_linear_speed must be positive, got: {max_linear_speed}")
        if not math.isfinite(max_angular_speed) or max_angular_speed <= 0:
            raise ValueError(f"max_angular_speed must be positive, got: {max_angular_speed}")
        if not math.isfinite(goal_tolerance) or goal_tolerance <= 0:
            raise ValueError(f"goal_tolerance must be positive, got: {goal_tolerance}")
        if not math.isfinite(slowdown_radius) or slowdown_radius <= goal_tolerance:
            raise ValueError(
                f"slowdown_radius ({slowdown_radius}) must be greater than goal_tolerance ({goal_tolerance})"
            )
        if not math.isfinite(min_linear_speed) or min_linear_speed < 0:
            raise ValueError(f"min_linear_speed must be non-negative, got: {min_linear_speed}")

        self.world = world
        self.lookahead_distance = float(lookahead_distance)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.goal_tolerance = float(goal_tolerance)
        self.slowdown_radius = float(slowdown_radius)
        self.min_linear_speed = float(min_linear_speed)
        self.check_footprint = bool(check_footprint)
        self.robot_radius = float(robot_radius if robot_radius is not None else world.robot_radius)

        self.grid_path: List[Tuple[int, int]] = []
        self.world_path: List[Tuple[float, float]] = []
        self.current_segment_index: int = 0
        self.status: ControllerStatus = ControllerStatus.IDLE

    def set_path(self, path: Optional[List[Tuple[int, int]]]) -> None:
        """
        Set or update the target trajectory from discrete grid waypoints.

        Args:
            path: List of (gx, gy) grid waypoint tuples.
        """
        if not path:
            self.grid_path = []
            self.world_path = []
            self.current_segment_index = 0
            self.status = ControllerStatus.FAILED
            return

        self.grid_path = list(path)
        # Convert grid waypoints to continuous world coordinates (cell centers)
        self.world_path = [self.world.grid_to_world(gx, gy) for gx, gy in self.grid_path]
        self.current_segment_index = 0
        self.status = ControllerStatus.TRACKING

    def get_lookahead_target(
        self,
        rx: float,
        ry: float,
    ) -> Tuple[Tuple[float, float], int]:
        """
        Find the lookahead point along the path at lookahead_distance from (rx, ry).

        Searches forward segments starting from current_segment_index using
        circle-line segment intersections to prevent backward motion.

        Returns:
            Tuple of ((target_x, target_y), active_segment_index).
        """
        if not self.world_path:
            return ((rx, ry), 0)

        if len(self.world_path) == 1:
            return (self.world_path[0], 0)

        num_segments = len(self.world_path) - 1
        best_target: Optional[Tuple[float, float]] = None
        best_seg_idx = self.current_segment_index
        L_d = self.lookahead_distance

        for seg_idx in range(self.current_segment_index, num_segments):
            p1 = self.world_path[seg_idx]
            p2 = self.world_path[seg_idx + 1]

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            fx = p1[0] - rx
            fy = p1[1] - ry

            a = dx * dx + dy * dy
            if a < 1e-9:
                continue

            b = 2.0 * (fx * dx + fy * dy)
            c = (fx * fx + fy * fy) - L_d * L_d
            disc = b * b - 4.0 * a * c

            if disc >= 0.0:
                sqrt_disc = math.sqrt(disc)
                t2 = (-b + sqrt_disc) / (2.0 * a)
                t1 = (-b - sqrt_disc) / (2.0 * a)

                # Prioritize t2 (further forward along the segment)
                if 0.0 <= t2 <= 1.0:
                    best_target = (p1[0] + t2 * dx, p1[1] + t2 * dy)
                    best_seg_idx = seg_idx
                elif 0.0 <= t1 <= 1.0:
                    best_target = (p1[0] + t1 * dx, p1[1] + t1 * dy)
                    best_seg_idx = seg_idx

        if best_target is not None:
            self.current_segment_index = best_seg_idx
            return (best_target, best_seg_idx)

        # If no forward intersection is found, target the final goal
        goal_pt = self.world_path[-1]
        self.current_segment_index = max(self.current_segment_index, num_segments - 1)
        return (goal_pt, self.current_segment_index)

    def compute_command(self, state: RobotState) -> ControllerOutput:
        """
        Compute linear and angular velocity commands for current robot state.

        Args:
            state: Current RobotState (x, y, theta, v, omega).

        Returns:
            ControllerOutput containing v_cmd, omega_cmd, status, and telemetry.
        """
        if not self.world_path:
            self.status = ControllerStatus.FAILED
            return ControllerOutput(
                v_cmd=0.0,
                omega_cmd=0.0,
                status=ControllerStatus.FAILED,
                lookahead_target=None,
                distance_to_goal=float("inf"),
                message="No path provided to controller.",
            )

        goal_x, goal_y = self.world_path[-1]
        dist_to_goal = math.hypot(goal_x - state.x, goal_y - state.y)

        # Goal arrival check
        if dist_to_goal <= self.goal_tolerance:
            self.status = ControllerStatus.REACHED_GOAL
            return ControllerOutput(
                v_cmd=0.0,
                omega_cmd=0.0,
                status=ControllerStatus.REACHED_GOAL,
                lookahead_target=(goal_x, goal_y),
                distance_to_goal=dist_to_goal,
                message=f"Goal reached within tolerance ({dist_to_goal:.2f} <= {self.goal_tolerance}).",
            )

        # Determine lookahead target
        target, _ = self.get_lookahead_target(state.x, state.y)
        tx, ty = target

        # Pure Pursuit geometry: heading error alpha
        angle_to_target = math.atan2(ty - state.y, tx - state.x)
        alpha = normalize_angle(angle_to_target - state.theta)

        # Distance to lookahead target L
        L = math.hypot(tx - state.x, ty - state.y)
        if L < 1e-4:
            L = 1e-4

        # Curvature: kappa = 2 * sin(alpha) / L
        kappa = (2.0 * math.sin(alpha)) / L

        # Speed regulation:
        # 1. Smooth deceleration near final goal
        speed_factor = 1.0
        if dist_to_goal < self.slowdown_radius:
            speed_factor = max(
                self.min_linear_speed / self.max_linear_speed,
                dist_to_goal / self.slowdown_radius,
            )

        # 2. Reduce linear speed during sharp heading deviations
        turn_factor = max(0.25, math.cos(alpha))

        v_cmd = self.max_linear_speed * speed_factor * turn_factor
        v_cmd = max(self.min_linear_speed, min(self.max_linear_speed, v_cmd))

        # Commanded angular rate: omega = v * kappa
        omega_cmd = v_cmd * kappa
        omega_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, omega_cmd))

        self.status = ControllerStatus.TRACKING
        return ControllerOutput(
            v_cmd=v_cmd,
            omega_cmd=omega_cmd,
            status=ControllerStatus.TRACKING,
            lookahead_target=target,
            distance_to_goal=dist_to_goal,
            message="Tracking trajectory.",
        )

    def check_step_collision(
        self,
        candidate_x: float,
        candidate_y: float,
        current_x: Optional[float] = None,
        current_y: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Check if a candidate continuous position (candidate_x, candidate_y) or swept
        movement segment from (current_x, current_y) to (candidate_x, candidate_y) is collision-safe.

        When check_footprint is True:
        - If current_x and current_y are provided, verifies that the swept circular footprint
          along the entire segment from (current_x, current_y) to (candidate_x, candidate_y)
          is strictly collision-free from obstacles and world boundaries.
        - If current_x and current_y are omitted, checks the candidate position footprint.
        Otherwise checks only the center point.

        Args:
            candidate_x: Candidate continuous world X.
            candidate_y: Candidate continuous world Y.
            current_x: Optional current continuous world X.
            current_y: Optional current continuous world Y.

        Returns:
            Tuple of (is_safe: bool, reason: str).
        """
        if self.check_footprint:
            if current_x is not None and current_y is not None:
                return self.world.check_swept_footprint_collision(
                    current_x,
                    current_y,
                    candidate_x,
                    candidate_y,
                    radius=self.robot_radius,
                )
            return self.world.check_footprint_collision(
                candidate_x, candidate_y, radius=self.robot_radius
            )

        if not self.world.is_inside_world(candidate_x, candidate_y):
            return (
                False,
                f"Candidate position ({candidate_x:.2f}, {candidate_y:.2f}) is outside world bounds "
                f"[0, {self.world.width}) x [0, {self.world.height}).",
            )

        try:
            gx, gy = self.world.world_to_grid(candidate_x, candidate_y)
        except ValueError as err:
            return (False, str(err))

        if self.world.is_occupied(gx, gy):
            return (
                False,
                f"Candidate position ({candidate_x:.2f}, {candidate_y:.2f}) maps to occupied "
                f"obstacle cell ({gx}, {gy}).",
            )

        return (True, "Candidate position is clear.")


def step_simulation(
    robot_state: RobotState,
    controller: PurePursuitController,
    motion_model: KinematicMotionModel,
    dt: float,
    dynamic_obstacles: Optional[Sequence[Any]] = None,
    current_time: Optional[float] = None,
) -> Tuple[RobotState, ControllerOutput]:
    """
    Execute one complete simulation step: controller -> kinematics -> collision check.

    If candidate next state collides with an obstacle or leaves world bounds,
    the candidate step is rejected, robot velocity is set to 0, and COLLISION status
    is returned.

    Accounts for moving dynamic obstacles over the continuous timestep interval
    [current_time, current_time + dt].

    Args:
        robot_state: Current RobotState.
        controller: Initialized PurePursuitController.
        motion_model: Initialized KinematicMotionModel.
        dt: Time increment in seconds.
        dynamic_obstacles: Optional sequence of DynamicObstacle instances to verify against.
        current_time: Optional simulation start time for the step (defaults to obstacle current_time).

    Returns:
        Tuple of (new_robot_state, controller_output).
    """
    ctrl_out = controller.compute_command(robot_state)

    if ctrl_out.status == ControllerStatus.REACHED_GOAL:
        stopped_state = RobotState(
            x=robot_state.x,
            y=robot_state.y,
            theta=robot_state.theta,
            v=0.0,
            omega=0.0,
        )
        return (stopped_state, ctrl_out)

    if ctrl_out.status in (ControllerStatus.FAILED, ControllerStatus.COLLISION, ControllerStatus.BLOCKED):
        stopped_state = RobotState(
            x=robot_state.x,
            y=robot_state.y,
            theta=robot_state.theta,
            v=0.0,
            omega=0.0,
        )
        return (stopped_state, ctrl_out)

    candidate_state = motion_model.step(
        robot_state, ctrl_out.v_cmd, ctrl_out.omega_cmd, dt
    )

    is_safe, reason = controller.check_step_collision(
        candidate_state.x,
        candidate_state.y,
        current_x=robot_state.x,
        current_y=robot_state.y,
    )

    if not is_safe:
        stopped_state = RobotState(
            x=robot_state.x,
            y=robot_state.y,
            theta=robot_state.theta,
            v=0.0,
            omega=0.0,
        )
        collision_output = ControllerOutput(
            v_cmd=0.0,
            omega_cmd=0.0,
            status=ControllerStatus.COLLISION,
            lookahead_target=ctrl_out.lookahead_target,
            distance_to_goal=ctrl_out.distance_to_goal,
            message=f"Collision rejected: {reason}",
        )
        controller.status = ControllerStatus.COLLISION
        return (stopped_state, collision_output)

    # Dynamic obstacle continuous swept collision verification
    if dynamic_obstacles:
        from autonomous_nav.dynamic_obstacle import (
            check_moving_obstacle_step_collision,
            DynamicObstacle,
            point_to_segment_distance,
        )
        for dyn in dynamic_obstacles:
            t_s = current_time if current_time is not None else getattr(dyn, "current_time", 0.0)
            t_e = t_s + dt
            if isinstance(dyn, DynamicObstacle):
                collided, min_dist, msg = check_moving_obstacle_step_collision(
                    robot_start=(robot_state.x, robot_state.y),
                    robot_end=(candidate_state.x, candidate_state.y),
                    robot_radius=controller.robot_radius,
                    obstacle=dyn,
                    t_start=t_s,
                    t_end=t_e,
                )
            else:
                pos = dyn.current_position if hasattr(dyn, "current_position") else getattr(dyn, "position", (0.0, 0.0))
                dist = point_to_segment_distance(
                    pos[0], pos[1], robot_state.x, robot_state.y, candidate_state.x, candidate_state.y
                )
                r_obs = getattr(dyn, "radius", 0.0)
                collided = (dist <= controller.robot_radius + r_obs)
                msg = f"Swept footprint collides with dynamic obstacle {getattr(dyn, 'id', '?')} at ({pos[0]:.1f}, {pos[1]:.1f})."

            if collided:
                stopped_state = RobotState(
                    x=robot_state.x,
                    y=robot_state.y,
                    theta=robot_state.theta,
                    v=0.0,
                    omega=0.0,
                )
                dyn_collision_output = ControllerOutput(
                    v_cmd=0.0,
                    omega_cmd=0.0,
                    status=ControllerStatus.COLLISION,
                    lookahead_target=ctrl_out.lookahead_target,
                    distance_to_goal=ctrl_out.distance_to_goal,
                    message=f"Collision rejected: {msg}",
                )
                controller.status = ControllerStatus.COLLISION
                return (stopped_state, dyn_collision_output)

    return (candidate_state, ctrl_out)


def run_simulation(
    world: World,
    path: List[Tuple[int, int]],
    initial_pose: Optional[Tuple[float, float, float]] = None,
    max_steps: int = 2000,
    dt: float = 0.05,
    max_linear_speed: float = 40.0,
    max_angular_speed: float = 2.5,
    lookahead_distance: float = 25.0,
    goal_tolerance: float = 5.0,
    check_footprint: bool = True,
    robot_radius: Optional[float] = None,
    dynamic_obstacles: Optional[List[Any]] = None,
    enable_replanning: bool = True,
    safety_margin: float = 5.0,
    replan_cooldown: float = 0.2,
    max_replans: int = 20,
) -> SimulationMetrics:
    """
    Run an end-to-end continuous robot simulation tracking path until completion,
    with dynamic obstacle conflict detection and safe A* replanning.

    Args:
        world: World environment instance.
        path: Planned grid waypoints [(gx, gy), ...].
        initial_pose: Optional (x, y, theta). Defaults to start of path.
        max_steps: Maximum allowable discrete steps before termination.
        dt: Integration time step in seconds.
        max_linear_speed: Maximum allowable linear speed.
        max_angular_speed: Maximum allowable angular speed.
        lookahead_distance: Pure Pursuit lookahead distance.
        goal_tolerance: Radial tolerance to declare goal reached.
        check_footprint: If True, candidate steps enforce circular robot footprint clearance.
        robot_radius: Optional override for robot footprint radius. Defaults to world.robot_radius.
        dynamic_obstacles: Optional list of DynamicObstacle instances (defaults to world.dynamic_obstacles).
        enable_replanning: If True, automatically detects route conflicts and triggers A* replanning.
        safety_margin: Additional safety clearance buffer for dynamic obstacle conflict checking.
        replan_cooldown: Minimum elapsed simulation time (seconds) between successive replan attempts.
        max_replans: Maximum allowed replan attempts during run to avoid uncontrolled computation.

    Returns:
        SimulationMetrics with comprehensive run telemetry including replan counts.
    """
    if not path:
        return SimulationMetrics(
            success=False,
            steps=0,
            elapsed_time=0.0,
            initial_pose=(0.0, 0.0, 0.0),
            final_pose=(0.0, 0.0, 0.0),
            goal_reached=False,
            collision_occurred=False,
            max_linear_speed=0.0,
            max_angular_speed=0.0,
            status=ControllerStatus.FAILED,
            message="No path provided to simulation.",
        )

    # Resolve dynamic obstacles
    if dynamic_obstacles is None:
        dyn_obs_list = list(getattr(world, "dynamic_obstacles", []))
    else:
        dyn_obs_list = list(dynamic_obstacles)

    # Initial pose setup
    start_wx, start_wy = world.grid_to_world(*path[0])
    if initial_pose is not None:
        init_x, init_y, init_theta = initial_pose
    else:
        init_x, init_y = start_wx, start_wy
        if len(path) > 1:
            next_wx, next_wy = world.grid_to_world(*path[1])
            init_theta = math.atan2(next_wy - start_wy, next_wx - start_wx)
        else:
            init_theta = 0.0

    robot = RobotState(x=init_x, y=init_y, theta=init_theta, v=0.0, omega=0.0)
    start_pose = robot.pose

    controller = PurePursuitController(
        world=world,
        lookahead_distance=lookahead_distance,
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
        goal_tolerance=goal_tolerance,
        check_footprint=check_footprint,
        robot_radius=robot_radius,
    )
    controller.set_path(path)

    motion_model = KinematicMotionModel(
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
    )

    steps = 0
    max_v_seen = 0.0
    max_omega_seen = 0.0
    final_status = ControllerStatus.TRACKING
    collision_occurred = False
    goal_reached = False
    message = "Simulation ended."
    distance_traveled = 0.0

    replans_attempted = 0
    replans_succeeded = 0
    last_replan_step = -9999
    replan_cooldown_steps = max(1, int(replan_cooldown / dt))

    # Reset dynamic obstacle simulation clocks
    for dyn in dyn_obs_list:
        dyn.reset()

    while steps < max_steps:
        sim_time = steps * dt

        # Synchronize dynamic obstacle clocks with simulation time
        for dyn in dyn_obs_list:
            dyn.current_time = sim_time

        # Dynamic Obstacle Route Conflict Detection & Safe Replanning
        if enable_replanning and dyn_obs_list and len(controller.world_path) > 0:
            from autonomous_nav.dynamic_obstacle import (
                check_route_conflict,
                create_temporary_planning_world,
                find_safe_start_grid,
            )
            from autonomous_nav.planner import AStarPlanner
            from autonomous_nav.path_smoother import PathSmoother

            remaining_points = [(robot.x, robot.y)] + controller.world_path[controller.current_segment_index:]
            is_conflict, conf_dyn, _ = check_route_conflict(
                remaining_points,
                dyn_obs_list,
                robot_radius=controller.robot_radius,
                safety_margin=safety_margin,
                current_time=sim_time,
            )

            if is_conflict:
                if (steps - last_replan_step) >= replan_cooldown_steps:
                    if replans_attempted < max_replans:
                        replans_attempted += 1
                        temp_world = create_temporary_planning_world(
                            world,
                            dyn_obs_list,
                            safety_margin=safety_margin,
                            current_time=sim_time,
                        )
                        start_cell = find_safe_start_grid(world, temp_world, robot.x, robot.y)
                        goal_cell = controller.grid_path[-1]

                        if start_cell is not None:
                            planner = AStarPlanner(
                                temp_world,
                                connectivity=8,
                                strict_diagonal_clearance=True,
                                use_footprint=True,
                            )
                            plan_res = planner.plan(start_cell, goal_cell)

                            if plan_res.success and plan_res.path:
                                smoother = PathSmoother(temp_world, check_footprint=True)
                                smooth_res = smoother.smooth_result(plan_res)
                                new_path = smooth_res.smoothed_path if smooth_res.success else plan_res.path

                                # Verify continuous swept footprint along replacement route
                                path_safe = (
                                    all(
                                        temp_world.is_swept_footprint_safe(
                                            *temp_world.grid_to_world(*p1),
                                            *temp_world.grid_to_world(*p2),
                                        )
                                        for p1, p2 in zip(new_path[:-1], new_path[1:])
                                    )
                                    if len(new_path) >= 2
                                    else True
                                )

                                if path_safe:
                                    controller.set_path(new_path)
                                    replans_succeeded += 1
                                    last_replan_step = steps
                                else:
                                    # Safe stop: replacement path failed swept footprint verification
                                    robot = RobotState(x=robot.x, y=robot.y, theta=robot.theta, v=0.0, omega=0.0)
                                    final_status = ControllerStatus.BLOCKED
                                    message = "Replanning failed: Replacement path failed swept footprint check. Safe stop initiated."
                                    controller.status = ControllerStatus.BLOCKED
                                    break
                            else:
                                # Safe stop: A* found no collision-free path
                                robot = RobotState(x=robot.x, y=robot.y, theta=robot.theta, v=0.0, omega=0.0)
                                final_status = ControllerStatus.BLOCKED
                                obs_info = f"dynamic obstacle {conf_dyn.id}" if conf_dyn else "dynamic obstacle"
                                message = f"Replanning failed: No collision-free route around {obs_info}. Safe stop initiated."
                                controller.status = ControllerStatus.BLOCKED
                                break
                        else:
                            # Safe stop: current robot position lacks adequate clearance
                            robot = RobotState(x=robot.x, y=robot.y, theta=robot.theta, v=0.0, omega=0.0)
                            final_status = ControllerStatus.BLOCKED
                            message = "Replanning failed: Robot position lacks adequate footprint clearance. Safe stop initiated."
                            controller.status = ControllerStatus.BLOCKED
                            break
                    else:
                        # Safe stop: replan attempt budget exhausted while route in conflict
                        robot = RobotState(x=robot.x, y=robot.y, theta=robot.theta, v=0.0, omega=0.0)
                        final_status = ControllerStatus.BLOCKED
                        message = f"Replanning limit reached ({max_replans}) while route in conflict. Safe stop initiated."
                        controller.status = ControllerStatus.BLOCKED
                        break

        prev_x, prev_y = robot.x, robot.y
        robot, ctrl_out = step_simulation(
            robot,
            controller,
            motion_model,
            dt,
            dynamic_obstacles=dyn_obs_list,
            current_time=sim_time,
        )
        steps += 1
        distance_traveled += math.hypot(robot.x - prev_x, robot.y - prev_y)

        # Advance dynamic obstacle clocks by dt after completing simulation step
        for dyn in dyn_obs_list:
            dyn.current_time = steps * dt

        max_v_seen = max(max_v_seen, abs(ctrl_out.v_cmd))
        max_omega_seen = max(max_omega_seen, abs(ctrl_out.omega_cmd))
        final_status = ctrl_out.status

        if ctrl_out.status == ControllerStatus.REACHED_GOAL:
            goal_reached = True
            message = ctrl_out.message
            break

        if ctrl_out.status == ControllerStatus.COLLISION:
            collision_occurred = True
            message = ctrl_out.message
            break

        if ctrl_out.status in (ControllerStatus.FAILED, ControllerStatus.BLOCKED):
            message = ctrl_out.message
            break

    if steps >= max_steps and not goal_reached and not collision_occurred and final_status != ControllerStatus.BLOCKED:
        message = f"Simulation timed out after {max_steps} steps before reaching goal."

    success = goal_reached and not collision_occurred
    goal_wx, goal_wy = world.grid_to_world(*path[-1])
    final_dist = math.hypot(robot.x - goal_wx, robot.y - goal_wy)

    return SimulationMetrics(
        success=success,
        steps=steps,
        elapsed_time=steps * dt,
        initial_pose=start_pose,
        final_pose=robot.pose,
        goal_reached=goal_reached,
        collision_occurred=collision_occurred,
        max_linear_speed=max_v_seen,
        max_angular_speed=max_omega_seen,
        status=final_status,
        message=message,
        distance_traveled=distance_traveled,
        final_distance_to_goal=final_dist,
        replans_attempted=replans_attempted,
        replans_succeeded=replans_succeeded,
        dynamic_obstacles_count=len(dyn_obs_list),
    )
