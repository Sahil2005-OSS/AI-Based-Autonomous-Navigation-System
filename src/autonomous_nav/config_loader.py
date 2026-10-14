"""
Configuration loader and validator for Autonomous Navigation System scenarios.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict


def validate_scenario_config(data: Dict[str, Any]) -> None:
    """
    Validate that a parsed scenario dictionary conforms to expected schema requirements.

    Raises:
        ValueError: If required sections or parameters are missing or invalid.
    """
    if not isinstance(data, dict):
        raise ValueError(f"Configuration root must be a JSON object (dict), got {type(data).__name__}")

    if "world" not in data:
        raise ValueError("Configuration missing required 'world' block.")

    world_cfg = data["world"]
    if not isinstance(world_cfg, dict):
        raise ValueError(f"'world' block must be a JSON object, got {type(world_cfg).__name__}")

    for dim in ("width", "height", "grid_resolution"):
        if dim not in world_cfg:
            raise ValueError(f"'world' configuration missing required '{dim}' field.")
        val = world_cfg[dim]
        if not isinstance(val, (int, float)) or val <= 0:
            raise ValueError(f"'world.{dim}' must be a strictly positive number, got {val}")

    if "robot" in data:
        robot_cfg = data["robot"]
        if not isinstance(robot_cfg, dict):
            raise ValueError(f"'robot' block must be a JSON object, got {type(robot_cfg).__name__}")
        for pos_key in ("start_position", "target_position"):
            if pos_key in robot_cfg:
                pos = robot_cfg[pos_key]
                if not (isinstance(pos, (list, tuple)) and len(pos) == 2 and all(isinstance(x, (int, float)) for x in pos)):
                    raise ValueError(f"'robot.{pos_key}' must be a 2-element numeric coordinate list [x, y], got {pos}")

    if "obstacles" in data:
        obs_list = data["obstacles"]
        if not isinstance(obs_list, list):
            raise ValueError(f"'obstacles' must be a list, got {type(obs_list).__name__}")
        for idx, obs in enumerate(obs_list):
            if not isinstance(obs, dict):
                raise ValueError(f"Obstacle at index {idx} must be a JSON object, got {type(obs).__name__}")
            if "type" not in obs:
                raise ValueError(f"Obstacle at index {idx} missing required 'type' field.")
            if "position" not in obs:
                raise ValueError(f"Obstacle at index {idx} missing required 'position' field.")
            pos = obs["position"]
            if not (isinstance(pos, (list, tuple)) and len(pos) == 2 and all(isinstance(x, (int, float)) for x in pos)):
                raise ValueError(f"Obstacle at index {idx} position must be a 2-element coordinate list [x, y], got {pos}")

    if "dynamic_obstacles" in data:
        dyn_list = data["dynamic_obstacles"]
        if not isinstance(dyn_list, list):
            raise ValueError(f"'dynamic_obstacles' must be a list, got {type(dyn_list).__name__}")
        for idx, dyn in enumerate(dyn_list):
            if not isinstance(dyn, dict):
                raise ValueError(f"Dynamic obstacle at index {idx} must be a JSON object, got {type(dyn).__name__}")
            for pos_key in ("start_position", "end_position"):
                if pos_key in dyn:
                    pos = dyn[pos_key]
                    if not (isinstance(pos, (list, tuple)) and len(pos) == 2 and all(isinstance(x, (int, float)) for x in pos)):
                        raise ValueError(f"Dynamic obstacle at index {idx} '{pos_key}' must be a 2-element numeric coordinate list [x, y], got {pos}")
            if "radius" in dyn:
                r = dyn["radius"]
                if not (isinstance(r, (int, float)) and math.isfinite(r) and r > 0):
                    raise ValueError(f"Dynamic obstacle at index {idx} radius must be a positive number, got {r}")


def load_config(config_path: str | Path, validate: bool = True) -> Dict[str, Any]:
    """
    Load, parse, and validate a JSON scenario configuration file.

    Args:
        config_path: Path to the JSON configuration file.
        validate: Whether to validate the configuration schema (default True).

    Returns:
        Dictionary containing parsed configuration data.

    Raises:
        FileNotFoundError: If the configuration file does not exist.
        ValueError: If the file is not valid JSON or fails schema validation.
    """
    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file not found at: {path}")

    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as err:
        raise ValueError(f"Invalid JSON in configuration file {path}: {err}") from err

    if validate:
        validate_scenario_config(data)

    return data
