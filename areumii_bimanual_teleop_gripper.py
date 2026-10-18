# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compatibility entry point for the standalone teleoperation package."""

import importlib
import sys
from pathlib import Path

# Allow the historical direct-script command before installing the package.
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from areumii_teleop.cli import main, parse_args  # noqa: E402, F401


def __getattr__(name: str):
    return getattr(importlib.import_module("areumii_teleop.teleop"), name)


if __name__ == "__main__":
    main()
