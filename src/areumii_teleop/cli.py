# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Command-line configuration without importing XR or simulator libraries."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path

import tomllib


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments and validate the robot model and configuration.

    Args:
        argv: Arguments to parse; defaults to the process arguments.

    Returns:
        Resolved configuration. Angles use [rad], distances [m], and times [s].
    """
    parser = argparse.ArgumentParser(description="AReuMii bimanual XR teleoperation over UDP.")
    parser.add_argument("--config", type=Path, help="TOML overrides; relative paths resolve beside this file.")
    parser.add_argument("--urdf", help="URDF override; defaults to the bundled kinematics model.")
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--cmd_port", "--cmd-port", type=int)
    parser.add_argument("--feedback_port", "--feedback-port", type=int)
    parser.add_argument("--cloudxr_env", "--cloudxr-env")
    parser.add_argument("--fps", type=float, default=30.0, help="Control frequency [Hz].")
    parser.add_argument("--clutch_threshold", "--clutch-threshold", type=float, default=0.5)
    parser.add_argument("--orientation_weight", "--orientation-weight", type=float, default=0.01)
    parser.add_argument("--max_ee_step_m", "--max-ee-step-m", type=float, default=0.005)
    parser.add_argument("--max_joint_step_rad", "--max-joint-step-rad", type=float, default=0.02)
    parser.add_argument("--max_gripper_step_rad", "--max-gripper-step-rad", type=float, default=0.05)
    parser.add_argument("--feedback_timeout_s", "--feedback-timeout-s", type=float, default=0.2)
    parser.add_argument(
        "--enable_command", "--enable-command", action="store_true", help="Enable command transmission."
    )
    parser.add_argument(
        "--print_config", action="store_true", help="Print validated configuration and exit without XR."
    )
    # Parse once before loading files so --help always works.
    initial = parser.parse_args(argv)
    try:
        assets = files("areumii_teleop").joinpath("assets")
        config = tomllib.loads(assets.joinpath("default.toml").read_text())
        config["teleop"].update(urdf=None, cloudxr_env=None, cmd_port=None, feedback_port=None)
        if initial.config is not None:
            config_path = initial.config.expanduser().resolve()
            override = tomllib.loads(config_path.read_text())
            _merge_config(config, override)
            for key in ("urdf", "cloudxr_env"):
                value = config["teleop"][key]
                if value is not None:
                    path = Path(value).expanduser()
                    config["teleop"][key] = str(path if path.is_absolute() else config_path.parent / path)
        parser.set_defaults(**config["teleop"])
        args = parser.parse_args(argv)
        args.robot = config["robot"]
        args.urdf = (
            str(Path(args.urdf).expanduser().resolve())
            if args.urdf
            else str(assets.joinpath("areumii_c1_kinematics.urdf"))
        )
        if args.cloudxr_env is not None:
            args.cloudxr_env = str(Path(args.cloudxr_env).expanduser().resolve())
            if not Path(args.cloudxr_env).is_file():
                raise ValueError(f"CloudXR profile not found: {args.cloudxr_env}")
        _validate_config(args)
    except (OSError, ValueError, TypeError, ET.ParseError) as error:
        parser.error(str(error))
    return args


def main() -> None:
    """Run teleoperation or print configuration without starting any devices."""
    args = parse_args()
    if args.print_config:
        print(json.dumps(vars(args), indent=2, default=str))
        return
    # Defer hardware dependencies until argument validation has completed.
    importlib.import_module("areumii_teleop.teleop").main(args)


def _merge_config(base: dict, override: dict) -> None:
    for key, value in override.items():
        if key not in base:
            raise ValueError(f"Unknown configuration key: {key}")
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"Expected a table for {key}")
            _merge_config(base[key], value)
        else:
            base[key] = value


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _validate_config(args: argparse.Namespace) -> None:
    if args.mode not in ("sim", "real"):
        raise ValueError("mode must be sim or real")
    for name in ("fps", "max_ee_step_m", "max_joint_step_rad", "max_gripper_step_rad", "feedback_timeout_s"):
        if _finite(getattr(args, name), name) <= 0:
            raise ValueError(f"{name} must be positive")
    if not 0 <= _finite(args.clutch_threshold, "clutch_threshold") <= 1:
        raise ValueError("clutch_threshold must be in [0, 1]")
    if _finite(args.orientation_weight, "orientation_weight") < 0:
        raise ValueError("orientation_weight must be nonnegative")
    for name in ("cmd_port", "feedback_port"):
        value = getattr(args, name)
        if value is not None and (type(value) is not int or not 1 <= value <= 65535):
            raise ValueError(f"{name} must be an integer in [1, 65535]")
    robot = args.robot
    margin = _finite(robot["joint_margin"], "joint_margin")
    if margin < 0:
        raise ValueError("joint_margin must be nonnegative")
    lower, upper = robot["joint_lower"], robot["joint_upper"]
    if not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != 7 or len(upper) != 7:
        raise ValueError("Joint limits must each contain seven angles [rad]")
    for lo, hi in zip(lower, upper, strict=True):
        if _finite(lo, "joint_lower") + margin >= _finite(hi, "joint_upper") - margin:
            raise ValueError("Joint lower limit plus margin must be below upper limit minus margin")
    model = ET.parse(args.urdf).getroot()
    joints = {joint.get("name") for joint in model.findall("joint")}
    links = {link.get("name") for link in model.findall("link")}
    for side in ("left", "right"):
        names = robot["joint_names"][side]
        if not isinstance(names, list) or len(names) != 7 or not all(isinstance(n, str) for n in names):
            raise ValueError(f"{side}: expected seven joint names")
        if len(set(names)) != 7 or not set(names).issubset(joints):
            raise ValueError(f"{side}: joint names must be unique and present in the URDF")
        if robot["target_frame"][side] not in links:
            raise ValueError(f"{side}: target frame is missing from the URDF")
        for key in ("gripper_open", "gripper_close"):
            _finite(robot[key][side], f"{key}.{side}")


if __name__ == "__main__":
    main()
