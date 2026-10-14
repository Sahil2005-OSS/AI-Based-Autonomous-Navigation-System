"""
Kinematic Robot Motion Model for Autonomous Navigation.

Implements a unicycle-style kinematic vehicle model in continuous 2D world
coordinates using discrete-time forward integration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """
    Normalize an angular value into the half-open interval [-pi, pi).

    Args:
        angle: Angle in radians.

    Returns:
        Equivalent angle in radians strictly within [-pi, pi).

    Raises:
        ValueError: If angle is not finite (NaN or infinite).
    """
    if not math.isfinite(angle):
        raise ValueError(f"Angle must be finite, got: {angle}")
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    # Handle floating point edge case where wrapped rounds to pi
    if wrapped >= math.pi:
        wrapped -= 2.0 * math.pi
    return float(wrapped)


@dataclass
class RobotState:
    """
    Continuous state representation of the kinematic robot.

    Attributes:
        x: Continuous world X coordinate (length units).
        y: Continuous world Y coordinate (length units).
        theta: Heading orientation in radians, normalized to [-pi, pi).
        v: Current linear forward velocity (length units / second).
        omega: Current angular velocity in radians / second (positive counter-clockwise).
    """

    x: float
    y: float
    theta: float
    v: float = 0.0
    omega: float = 0.0

    def __post_init__(self) -> None:
        """Validate numerical integrity and normalize orientation."""
        for name, val in [
            ("x", self.x),
            ("y", self.y),
            ("theta", self.theta),
            ("v", self.v),
            ("omega", self.omega),
        ]:
            if not isinstance(val, (int, float)) or not math.isfinite(val):
                raise ValueError(
                    f"RobotState parameter '{name}' must be a finite real number, got: {val}"
                )

        self.x = float(self.x)
        self.y = float(self.y)
        self.theta = normalize_angle(float(self.theta))
        self.v = float(self.v)
        self.omega = float(self.omega)

    @property
    def pose(self) -> Tuple[float, float, float]:
        """Return pose 3-tuple (x, y, theta)."""
        return (self.x, self.y, self.theta)

    @property
    def as_tuple(self) -> Tuple[float, float, float, float, float]:
        """Return complete state 5-tuple (x, y, theta, v, omega)."""
        return (self.x, self.y, self.theta, self.v, self.omega)


class KinematicMotionModel:
    """
    Unicycle-style discrete-time kinematic motion model.

    Equations of motion:
        x_next     = x + v * cos(theta) * dt
        y_next     = y + v * sin(theta) * dt
        theta_next = normalize(theta + omega * dt)

    Commands are clamped to configured limits before integration.
    """

    def __init__(
        self,
        max_linear_speed: float = 40.0,
        max_angular_speed: float = math.pi,
        min_linear_speed: float = 0.0,
    ) -> None:
        """
        Initialize the kinematic motion model with velocity limits.

        Args:
            max_linear_speed: Maximum permissible linear speed (> 0).
            max_angular_speed: Maximum permissible angular speed magnitude (> 0).
            min_linear_speed: Minimum permissible linear speed (>= 0).

        Raises:
            ValueError: If limits are non-positive or non-finite.
        """
        if not math.isfinite(max_linear_speed) or max_linear_speed <= 0:
            raise ValueError(
                f"max_linear_speed must be positive and finite, got: {max_linear_speed}"
            )
        if not math.isfinite(max_angular_speed) or max_angular_speed <= 0:
            raise ValueError(
                f"max_angular_speed must be positive and finite, got: {max_angular_speed}"
            )
        if not math.isfinite(min_linear_speed) or min_linear_speed < 0:
            raise ValueError(
                f"min_linear_speed must be non-negative and finite, got: {min_linear_speed}"
            )
        if min_linear_speed > max_linear_speed:
            raise ValueError(
                f"min_linear_speed ({min_linear_speed}) cannot exceed max_linear_speed ({max_linear_speed})"
            )

        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed = float(max_angular_speed)
        self.min_linear_speed = float(min_linear_speed)

    def clamp_commands(self, v_cmd: float, omega_cmd: float) -> Tuple[float, float]:
        """
        Clamp linear and angular velocity commands to configured bounds.

        Args:
            v_cmd: Commanded linear speed.
            omega_cmd: Commanded angular rate.

        Returns:
            Tuple of (clamped_v, clamped_omega).

        Raises:
            ValueError: If either command is non-finite.
        """
        if not math.isfinite(v_cmd):
            raise ValueError(f"v_cmd must be finite, got: {v_cmd}")
        if not math.isfinite(omega_cmd):
            raise ValueError(f"omega_cmd must be finite, got: {omega_cmd}")

        clamped_v = max(self.min_linear_speed, min(self.max_linear_speed, float(v_cmd)))
        clamped_omega = max(
            -self.max_angular_speed, min(self.max_angular_speed, float(omega_cmd))
        )
        return (clamped_v, clamped_omega)

    def step(
        self,
        state: RobotState,
        v_cmd: float,
        omega_cmd: float,
        dt: float,
    ) -> RobotState:
        """
        Advance robot state by timestep dt using unicycle kinematics.

        Args:
            state: Current RobotState.
            v_cmd: Commanded linear velocity.
            omega_cmd: Commanded angular velocity.
            dt: Discrete integration time step in seconds (> 0).

        Returns:
            Updated RobotState representing pose and velocities at t + dt.

        Raises:
            ValueError: If dt is non-positive or non-finite.
        """
        if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"Time step dt must be positive and finite, got: {dt}")

        v_clamped, omega_clamped = self.clamp_commands(v_cmd, omega_cmd)

        x_next = state.x + v_clamped * math.cos(state.theta) * dt
        y_next = state.y + v_clamped * math.sin(state.theta) * dt
        theta_next = normalize_angle(state.theta + omega_clamped * dt)

        return RobotState(
            x=x_next,
            y=y_next,
            theta=theta_next,
            v=v_clamped,
            omega=omega_clamped,
        )
