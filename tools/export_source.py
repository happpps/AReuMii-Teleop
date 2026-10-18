# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Export a standalone repository tree without local secrets or build artifacts."""

import tarfile
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_INCLUDE = (
    "pyproject.toml",
    "uv.lock",
    "README.md",
    "README_QUICKSTART.md",
    "LICENSE",
    "NOTICE",
    "MANIFEST.in",
    ".gitignore",
    ".dockerignore",
    "src",
    "isaac_teleop",
    "can_bridge",
    "configs",
    "docker",
    "licenses",
    "tests",
    "tools",
    "areumii_bimanual_teleop_gripper.py",
    "areumii_bimanual_teleop.py",
    "areumii_real_teleop.py",
    "areumii_c1_kinematics.urdf",
    "test_real_teleop_in_sim.py",
)


def _filter(member: tarfile.TarInfo) -> tarfile.TarInfo | None:
    path = Path(member.name)
    if any(part in {"__pycache__", ".venv", "build", "dist", ".pytest_cache", ".ruff_cache"} for part in path.parts):
        return None
    if path.name.startswith(".env") or path.suffix in {".env", ".zip", ".pyc", ".so"} or ".egg-info" in member.name:
        return None
    if member.isfile() and path.suffix == "" and path.parent.name == "can_bridge":
        return None
    # Keep only reviewed example configurations; personal TOML files are excluded.
    if path.parent.name == "configs" and path.name not in {"default.toml", "local.example.toml"}:
        return None
    return member


def main() -> None:
    """Write an uploadable source archive under the local dist directory."""
    destination = _ROOT / "dist/areumii-teleop-source.tar.gz"
    destination.parent.mkdir(exist_ok=True)
    with tarfile.open(destination, "w:gz") as archive:
        for name in _INCLUDE:
            archive.add(_ROOT / name, arcname=f"areumii-teleop/{name}", filter=_filter)
    print(destination)


if __name__ == "__main__":
    main()
