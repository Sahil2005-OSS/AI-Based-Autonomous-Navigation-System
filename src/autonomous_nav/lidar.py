"""
2D Planar LiDAR Sensor Simulation Module.

Simulates physical time-of-flight laser rangefinder scanning for autonomous navigation:
- Configurable Field of View (FOV) and angular resolution.
- Raymarching against static discrete occupancy grids and dynamic obstacles.
- Configurable Gaussian range measurement noise and dropout probability.
- Local polar coordinates (range, bearing) to global Cartesian pointcloud transformation.
- Obstacle cluster segmentation for perception and proximity detection.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import List, Optional, Tuple
import numpy as np

from autonomous_nav.world import World, CELL_OCCUPIED


@dataclass(frozen=True)
class LidarConfig:
    """
    Configuration parameters for planar 2D LiDAR sensor.

    Attributes:
        num_beams: Total number of laser beams per scan rotation.
        fov_rad: Total field of view in radians (default: 2*pi for 360° surround).
        min_range: Minimum measurable distance in world units.
        max_range: Maximum laser detection range in world units.
        range_noise_std: Standard deviation of additive Gaussian noise (sigma).
        dropout_prob: Probability [0.0, 1.0] of a laser beam failing to register a return.
    """
    num_beams: int = 180
    fov_rad: float = 2.0 * math.pi
    min_range: float = 2.0
    max_range: float = 200.0
    range_noise_std: float = 0.5
    dropout_prob: float = 0.0

    def __post_init__(self) -> None:
        if self.num_beams < 2:
            raise ValueError(f"num_beams must be >= 2, got {self.num_beams}")
        if self.fov_rad <= 0 or self.fov_rad > 2 * math.pi + 1e-6:
            raise ValueError(f"fov_rad must be in (0, 2*pi], got {self.fov_rad}")
        if self.min_range < 0 or self.max_range <= self.min_range:
            raise ValueError(f"Invalid range bounds: [{self.min_range}, {self.max_range}]")
        if self.range_noise_std < 0:
            raise ValueError(f"range_noise_std must be >= 0, got {self.range_noise_std}")
        if not (0.0 <= self.dropout_prob <= 1.0):
            raise ValueError(f"dropout_prob must be in [0.0, 1.0], got {self.dropout_prob}")


@dataclass
class LidarScan:
    """
    Result of a single LiDAR sensor scan rotation.

    Attributes:
        timestamp: Simulation timestamp when scan was captured.
        pose: (x, y, theta) robot pose at scan instant.
        angles: Relative bearing angles for each beam in [-fov/2, +fov/2].
        ranges: Measured distances for each beam (clamped to max_range on miss).
        intensities: Return signal intensity indicator (0.0 = drop/miss, 1.0 = solid hit).
        pointcloud_world: Continuous (x, y) Cartesian coordinates of confirmed obstacle returns.
    """
    timestamp: float
    pose: Tuple[float, float, float]
    angles: np.ndarray
    ranges: np.ndarray
    intensities: np.ndarray
    pointcloud_world: List[Tuple[float, float]] = field(default_factory=list)


class LidarSensor:
    """
    Planar 2D LiDAR raymarching sensor simulator.
    """

    def __init__(self, world: World, config: Optional[LidarConfig] = None) -> None:
        self.world = world
        self.config = config if config is not None else LidarConfig()

        # Pre-compute relative beam angles centered along robot forward x-axis
        half_fov = self.config.fov_rad / 2.0
        self.beam_angles = np.linspace(-half_fov, half_fov, self.config.num_beams, endpoint=False)

    def scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        sim_time: float = 0.0,
        dynamic_obstacles: Optional[List[object]] = None,
        random_seed: Optional[int] = None,
    ) -> LidarScan:
        """
        Execute a full LiDAR scan rotation from the current robot pose.

        Args:
            robot_x: Robot world X position.
            robot_y: Robot world Y position.
            robot_theta: Robot heading angle in radians.
            sim_time: Active simulation timestamp.
            dynamic_obstacles: Optional collection of dynamic obstacle instances.
            random_seed: Optional seed for reproducible noise generation.

        Returns:
            LidarScan containing polar ranges, bearing angles, and world pointcloud.
        """
        if not (math.isfinite(robot_x) and math.isfinite(robot_y) and math.isfinite(robot_theta)):
            raise ValueError(f"Non-finite robot pose provided: ({robot_x}, {robot_y}, {robot_theta})")

        rng = random.Random(random_seed) if random_seed is not None else random.Random()

        ranges = np.zeros(self.config.num_beams, dtype=np.float64)
        intensities = np.zeros(self.config.num_beams, dtype=np.float64)
        points_world: List[Tuple[float, float]] = []

        max_r = self.config.max_range
        step_size = self.world.grid_resolution * 0.4  # Sub-cell resolution ray stepping

        # Extract active positions of dynamic obstacles
        dyn_circles: List[Tuple[float, float, float]] = []
        if dynamic_obstacles:
            for dyn in dynamic_obstacles:
                pos = getattr(dyn, "current_position", None)
                if pos is None and hasattr(dyn, "position_at"):
                    pos = dyn.position_at(sim_time)
                rad = getattr(dyn, "radius", 0.0)
                if pos is not None and rad > 0:
                    dyn_circles.append((float(pos[0]), float(pos[1]), float(rad)))

        for i, angle_rel in enumerate(self.beam_angles):
            # Check dropout probability
            if self.config.dropout_prob > 0.0 and rng.random() < self.config.dropout_prob:
                ranges[i] = max_r
                intensities[i] = 0.0
                continue

            angle_global = robot_theta + angle_rel
            cos_a = math.cos(angle_global)
            sin_a = math.sin(angle_global)

            # Raycast forward from min_range up to max_range
            hit_dist = max_r
            r_cur = self.config.min_range

            while r_cur < max_r:
                px = robot_x + r_cur * cos_a
                py = robot_y + r_cur * sin_a

                # 1. World boundary check
                if px <= 0.0 or px >= self.world.width or py <= 0.0 or py >= self.world.height:
                    hit_dist = r_cur
                    break

                # 2. Discrete static occupancy grid check
                gx = int(px / self.world.grid_resolution)
                gy = int(py / self.world.grid_resolution)
                if 0 <= gx < self.world.grid_width and 0 <= gy < self.world.grid_height:
                    if self.world.grid[gy, gx] == CELL_OCCUPIED:
                        hit_dist = r_cur
                        break

                # 3. Dynamic obstacle circle checks
                hit_dyn = False
                for cx, cy, rad in dyn_circles:
                    if (px - cx) ** 2 + (py - cy) ** 2 <= rad ** 2:
                        hit_dist = r_cur
                        hit_dyn = True
                        break
                if hit_dyn:
                    break

                r_cur += step_size

            # Apply Gaussian measurement noise if hit within max_range
            if hit_dist < max_r:
                if self.config.range_noise_std > 0.0:
                    noise = rng.gauss(0.0, self.config.range_noise_std)
                    hit_dist = max(self.config.min_range, min(max_r, hit_dist + noise))

                ranges[i] = hit_dist
                intensities[i] = 1.0

                hit_x = robot_x + hit_dist * cos_a
                hit_y = robot_y + hit_dist * sin_a
                points_world.append((hit_x, hit_y))
            else:
                ranges[i] = max_r
                intensities[i] = 0.0

        return LidarScan(
            timestamp=sim_time,
            pose=(robot_x, robot_y, robot_theta),
            angles=self.beam_angles.copy(),
            ranges=ranges,
            intensities=intensities,
            pointcloud_world=points_world,
        )

    def segment_clusters(
        self,
        scan: LidarScan,
        cluster_distance_threshold: float = 15.0,
        min_cluster_points: int = 2,
    ) -> List[List[Tuple[float, float]]]:
        """
        Segment the return pointcloud into discrete Euclidean spatial clusters.

        Args:
            scan: LidarScan output from scan().
            cluster_distance_threshold: Maximum distance between adjacent points in the same cluster.
            min_cluster_points: Minimum number of return points required to form a cluster.

        Returns:
            List of clusters, where each cluster is a list of (x, y) coordinates.
        """
        pts = scan.pointcloud_world
        if not pts:
            return []

        clusters: List[List[Tuple[float, float]]] = []
        current_cluster: List[Tuple[float, float]] = [pts[0]]

        for i in range(1, len(pts)):
            prev_pt = pts[i - 1]
            curr_pt = pts[i]
            d = math.hypot(curr_pt[0] - prev_pt[0], curr_pt[1] - prev_pt[1])

            if d <= cluster_distance_threshold:
                current_cluster.append(curr_pt)
            else:
                if len(current_cluster) >= min_cluster_points:
                    clusters.append(current_cluster)
                current_cluster = [curr_pt]

        if len(current_cluster) >= min_cluster_points:
            clusters.append(current_cluster)

        return clusters
