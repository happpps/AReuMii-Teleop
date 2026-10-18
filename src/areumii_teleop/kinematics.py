# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""URDF forward and inverse kinematics adapters."""

import numpy as np
from lerobot.model.kinematics import RobotKinematics


class Kinematics:
    """Adapt LeRobot's degree-based solver to joint angles [rad]."""

    def __init__(self, urdf: str, frame: str, names: list[str]):
        self.kin = RobotKinematics(
            urdf_path=urdf,
            target_frame_name=frame,
            joint_names=names,
        )

    def fk(self, q: np.ndarray) -> np.ndarray:
        """Return the end-effector transform, with translation [m], from angles [rad]."""
        return self.kin.forward_kinematics(np.rad2deg(q))

    def ik(self, q_seed: np.ndarray, T: np.ndarray, orientation_weight: float) -> np.ndarray:
        """Return angles [rad] using a seed [rad] and a transform with translation [m]."""
        q_deg = self.kin.inverse_kinematics(
            np.rad2deg(q_seed),
            T,
            orientation_weight=orientation_weight,
        )
        return np.deg2rad(np.asarray(q_deg, dtype=float))
