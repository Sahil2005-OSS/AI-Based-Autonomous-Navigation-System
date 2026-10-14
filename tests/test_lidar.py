"""
Unit tests for 2D LiDAR Range Sensor Simulation Module (src/autonomous_nav/lidar.py).
"""

import math
import unittest
import numpy as np

from autonomous_nav.world import World, Obstacle
from autonomous_nav.dynamic_obstacle import DynamicObstacle
from autonomous_nav.lidar import LidarConfig, LidarSensor, LidarScan


class TestLidarConfigValidation(unittest.TestCase):
    """Test LidarConfig parameters and strict boundary validations."""

    def test_default_config(self) -> None:
        cfg = LidarConfig()
        self.assertEqual(cfg.num_beams, 180)
        self.assertAlmostEqual(cfg.fov_rad, 2.0 * math.pi)
        self.assertEqual(cfg.min_range, 2.0)
        self.assertEqual(cfg.max_range, 200.0)
        self.assertEqual(cfg.range_noise_std, 0.5)

    def test_invalid_num_beams_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LidarConfig(num_beams=1)

    def test_invalid_fov_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LidarConfig(fov_rad=0.0)
        with self.assertRaises(ValueError):
            LidarConfig(fov_rad=7.0)

    def test_invalid_ranges_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LidarConfig(min_range=-1.0)
        with self.assertRaises(ValueError):
            LidarConfig(min_range=50.0, max_range=30.0)

    def test_invalid_noise_and_dropout_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LidarConfig(range_noise_std=-0.1)
        with self.assertRaises(ValueError):
            LidarConfig(dropout_prob=1.5)


class TestLidarScanningAndRaycasting(unittest.TestCase):
    """Test raycasting against boundaries, static obstacles, and dynamic obstacles."""

    def setUp(self) -> None:
        self.world = World(width=200.0, height=200.0, grid_resolution=10.0)
        # Place a 20x20 static obstacle at center (90 to 110 in both dims)
        for gx in range(9, 11):
            for gy in range(9, 11):
                self.world.add_obstacle_cell(gx, gy)

    def test_non_finite_pose_rejected(self) -> None:
        sensor = LidarSensor(self.world)
        with self.assertRaises(ValueError):
            sensor.scan(float("nan"), 50.0, 0.0)
        with self.assertRaises(ValueError):
            sensor.scan(50.0, float("inf"), 0.0)

    def test_scan_dimensions_and_beams(self) -> None:
        cfg = LidarConfig(num_beams=60, range_noise_std=0.0)
        sensor = LidarSensor(self.world, cfg)
        scan = sensor.scan(robot_x=50.0, robot_y=50.0, robot_theta=0.0)

        self.assertEqual(len(scan.angles), 60)
        self.assertEqual(len(scan.ranges), 60)
        self.assertEqual(len(scan.intensities), 60)
        self.assertTrue(len(scan.pointcloud_world) > 0)

    def test_static_obstacle_detection_ahead(self) -> None:
        """Robot positioned at (50, 100) pointing right (0 rad) towards obstacle at x=90."""
        cfg = LidarConfig(num_beams=36, range_noise_std=0.0, fov_rad=math.pi / 2)  # +/- 45 deg
        sensor = LidarSensor(self.world, cfg)
        scan = sensor.scan(robot_x=50.0, robot_y=100.0, robot_theta=0.0)

        # Beam pointing directly ahead should hit obstacle around x=90 (dist ~ 40)
        ahead_idx = len(scan.angles) // 2
        measured_range = scan.ranges[ahead_idx]
        self.assertTrue(35.0 <= measured_range <= 45.0, f"Expected range near 40.0, got {measured_range}")

    def test_dynamic_obstacle_detection(self) -> None:
        """Dynamic obstacle placed directly in path is detected by laser scan."""
        empty_world = World(width=300.0, height=300.0, grid_resolution=10.0)
        cfg = LidarConfig(num_beams=36, range_noise_std=0.0)
        sensor = LidarSensor(empty_world, cfg)

        dyn = DynamicObstacle(
            obstacle_id=1,
            start_position=(100.0, 50.0),
            end_position=(100.0, 50.0),
            radius=15.0,
            speed=0.0,
        )

        scan = sensor.scan(robot_x=50.0, robot_y=50.0, robot_theta=0.0, dynamic_obstacles=[dyn])
        # Beam directly ahead towards (100, 50) hits circle edge at 100 - 15 = 85 (dist ~ 35)
        # Beam 0 is relative -pi; beam directly ahead is at angle 0
        angles = scan.angles
        zero_idx = int(np.argmin(np.abs(angles)))
        measured_dist = scan.ranges[zero_idx]
        self.assertTrue(30.0 <= measured_dist <= 40.0, f"Expected range near 35.0, got {measured_dist}")

    def test_dropout_probability_full(self) -> None:
        """100% dropout should return all max_range and zero intensities."""
        cfg = LidarConfig(num_beams=30, dropout_prob=1.0)
        sensor = LidarSensor(self.world, cfg)
        scan = sensor.scan(50.0, 50.0, 0.0)

        self.assertTrue(np.all(scan.ranges == cfg.max_range))
        self.assertTrue(np.all(scan.intensities == 0.0))
        self.assertEqual(len(scan.pointcloud_world), 0)

    def test_pointcloud_clustering(self) -> None:
        """Pointcloud is segmented into distinct Euclidean clusters."""
        sensor = LidarSensor(self.world)
        fake_scan = LidarScan(
            timestamp=0.0,
            pose=(0.0, 0.0, 0.0),
            angles=np.array([]),
            ranges=np.array([]),
            intensities=np.array([]),
            pointcloud_world=[
                # Cluster 1 (near (10, 10))
                (10.0, 10.0), (10.5, 10.2), (11.0, 10.4),
                # Cluster 2 (near (80, 80))
                (80.0, 80.0), (81.0, 80.5)
            ]
        )

        clusters = sensor.segment_clusters(fake_scan, cluster_distance_threshold=5.0, min_cluster_points=2)
        self.assertEqual(len(clusters), 2)
        self.assertEqual(len(clusters[0]), 3)
        self.assertEqual(len(clusters[1]), 2)


if __name__ == "__main__":
    unittest.main()
