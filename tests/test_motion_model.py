"""
Unit tests for Kinematic Motion Model (src/autonomous_nav/motion_model.py).
"""

import math
import unittest

from autonomous_nav.motion_model import (
    KinematicMotionModel,
    RobotState,
    normalize_angle,
)


class TestAngleNormalization(unittest.TestCase):
    """Test angular wrapping into [-pi, pi)."""

    def test_angle_within_range_unmodified(self) -> None:
        """Angles already within [-pi, pi) must remain unchanged."""
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(1.0), 1.0)
        self.assertAlmostEqual(normalize_angle(-1.0), -1.0)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)

    def test_angle_wrapping_positive(self) -> None:
        """Angles greater than or equal to pi wrap to negative counterpart."""
        self.assertAlmostEqual(normalize_angle(math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(2.0 * math.pi), 0.0)
        self.assertAlmostEqual(normalize_angle(2.5 * math.pi), 0.5 * math.pi)

    def test_angle_wrapping_negative(self) -> None:
        """Angles less than -pi wrap into [-pi, pi)."""
        self.assertAlmostEqual(normalize_angle(-2.0 * math.pi), 0.0)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-2.5 * math.pi), -0.5 * math.pi)

    def test_non_finite_angle_raises_value_error(self) -> None:
        """NaN and infinite angles must raise ValueError."""
        with self.assertRaises(ValueError):
            normalize_angle(float("nan"))
        with self.assertRaises(ValueError):
            normalize_angle(float("inf"))
        with self.assertRaises(ValueError):
            normalize_angle(float("-inf"))


class TestRobotStateValidation(unittest.TestCase):
    """Test RobotState construction, validation, and properties."""

    def test_valid_robot_state_construction(self) -> None:
        """Valid numerical states initialize cleanly."""
        state = RobotState(x=10.0, y=20.0, theta=0.5, v=5.0, omega=-0.2)
        self.assertEqual(state.x, 10.0)
        self.assertEqual(state.y, 20.0)
        self.assertEqual(state.theta, 0.5)
        self.assertEqual(state.v, 5.0)
        self.assertEqual(state.omega, -0.2)
        self.assertEqual(state.pose, (10.0, 20.0, 0.5))
        self.assertEqual(state.as_tuple, (10.0, 20.0, 0.5, 5.0, -0.2))

    def test_heading_automatically_normalized_on_init(self) -> None:
        """Heading passed to RobotState is normalized into [-pi, pi)."""
        state = RobotState(x=0.0, y=0.0, theta=3.0 * math.pi)
        self.assertAlmostEqual(state.theta, -math.pi)

    def test_non_finite_state_fields_rejected(self) -> None:
        """Any NaN or infinite field raises ValueError."""
        with self.assertRaises(ValueError):
            RobotState(x=float("nan"), y=0.0, theta=0.0)
        with self.assertRaises(ValueError):
            RobotState(x=0.0, y=float("inf"), theta=0.0)
        with self.assertRaises(ValueError):
            RobotState(x=0.0, y=0.0, theta=float("nan"))
        with self.assertRaises(ValueError):
            RobotState(x=0.0, y=0.0, theta=0.0, v=float("-inf"))
        with self.assertRaises(ValueError):
            RobotState(x=0.0, y=0.0, theta=0.0, omega=float("nan"))


class TestKinematicMotionModel(unittest.TestCase):
    """Test unicycle kinematic equations, velocity limits, and stepping."""

    def setUp(self) -> None:
        self.model = KinematicMotionModel(
            max_linear_speed=20.0,
            max_angular_speed=math.pi,
            min_linear_speed=0.0,
        )

    def test_model_initialization_limits_validation(self) -> None:
        """Invalid limit configurations raise ValueError."""
        with self.assertRaises(ValueError):
            KinematicMotionModel(max_linear_speed=-5.0)
        with self.assertRaises(ValueError):
            KinematicMotionModel(max_angular_speed=0.0)
        with self.assertRaises(ValueError):
            KinematicMotionModel(min_linear_speed=-1.0)
        with self.assertRaises(ValueError):
            KinematicMotionModel(max_linear_speed=10.0, min_linear_speed=20.0)

    def test_straight_line_motion(self) -> None:
        """Moving straight ahead (omega=0) advances x along heading."""
        # Facing East (theta=0)
        s0 = RobotState(x=0.0, y=0.0, theta=0.0)
        s1 = self.model.step(s0, v_cmd=10.0, omega_cmd=0.0, dt=1.0)

        self.assertAlmostEqual(s1.x, 10.0)
        self.assertAlmostEqual(s1.y, 0.0)
        self.assertAlmostEqual(s1.theta, 0.0)
        self.assertEqual(s1.v, 10.0)
        self.assertEqual(s1.omega, 0.0)

        # Facing North (theta=pi/2)
        s_north = RobotState(x=5.0, y=5.0, theta=math.pi / 2.0)
        s_next = self.model.step(s_north, v_cmd=8.0, omega_cmd=0.0, dt=0.5)
        self.assertAlmostEqual(s_next.x, 5.0)
        self.assertAlmostEqual(s_next.y, 9.0)  # 5.0 + 8.0 * 0.5
        self.assertAlmostEqual(s_next.theta, math.pi / 2.0)

    def test_pure_rotation_in_place(self) -> None:
        """Pure rotation (v=0) alters heading without translating."""
        s0 = RobotState(x=10.0, y=10.0, theta=0.0)
        s1 = self.model.step(s0, v_cmd=0.0, omega_cmd=math.pi / 4.0, dt=2.0)

        self.assertAlmostEqual(s1.x, 10.0)
        self.assertAlmostEqual(s1.y, 10.0)
        self.assertAlmostEqual(s1.theta, math.pi / 2.0)

    def test_velocity_clamping_to_configured_limits(self) -> None:
        """Commands exceeding limits are clamped."""
        s0 = RobotState(x=0.0, y=0.0, theta=0.0)
        s1 = self.model.step(s0, v_cmd=100.0, omega_cmd=10.0, dt=1.0)

        self.assertEqual(s1.v, 20.0)  # Clamped to max_linear_speed (20.0)
        self.assertEqual(s1.omega, math.pi)  # Clamped to max_angular_speed (pi)

        # Negative angular velocity clamping
        s2 = self.model.step(s0, v_cmd=10.0, omega_cmd=-10.0, dt=1.0)
        self.assertEqual(s2.omega, -math.pi)

    def test_invalid_timestep_rejected(self) -> None:
        """dt <= 0, NaN, or inf raises ValueError."""
        s0 = RobotState(x=0.0, y=0.0, theta=0.0)
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=5.0, omega_cmd=0.0, dt=0.0)
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=5.0, omega_cmd=0.0, dt=-0.1)
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=5.0, omega_cmd=0.0, dt=float("nan"))
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=5.0, omega_cmd=0.0, dt=float("inf"))

    def test_non_finite_commands_rejected(self) -> None:
        """NaN or inf commands raise ValueError."""
        s0 = RobotState(x=0.0, y=0.0, theta=0.0)
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=float("nan"), omega_cmd=0.0, dt=0.1)
        with self.assertRaises(ValueError):
            self.model.step(s0, v_cmd=5.0, omega_cmd=float("inf"), dt=0.1)

    def test_deterministic_stepping(self) -> None:
        """Identical inputs produce identical outputs across repeated steps."""
        s0 = RobotState(x=12.5, y=34.5, theta=0.7)
        res1 = self.model.step(s0, v_cmd=15.0, omega_cmd=0.5, dt=0.05)
        res2 = self.model.step(s0, v_cmd=15.0, omega_cmd=0.5, dt=0.05)

        self.assertEqual(res1.as_tuple, res2.as_tuple)


if __name__ == "__main__":
    unittest.main()
