# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Bimanual controller output for Isaac Teleop."""

import numpy as np
from isaac_teleop.teleop_xr_controller import XRController
from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
from isaacteleop.retargeting_engine.interface import OutputCombiner, ValueInput
from isaacteleop.retargeting_engine.tensor_types import TransformMatrix
from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex


class BimanualXRController(XRController):
    """기존 XRController에서 controller output만 좌/우 둘 다 꺼냄."""

    def _build_pipeline(self):
        controllers = ControllersSource(name="controllers")
        xform = ValueInput("base_T_anchor", TransformMatrix())
        transformed = controllers.transformed(xform.output("value"))

        return OutputCombiner(
            {
                "left": transformed.output("controller_left"),
                "right": transformed.output("controller_right"),
            }
        )

    def get_action(self):
        result = self._step(
            execution_events=self._running_events(),
            external_inputs=self._external_inputs,
        )

        actions = {}
        for side in ("left", "right"):
            c = result[side]

            if getattr(c, "is_none", False):
                actions[side] = None
                continue

            try:
                actions[side] = {
                    "grip_pos": np.asarray(
                        c[ControllerInputIndex.GRIP_POSITION],
                        dtype=np.float32,
                    ),
                    "grip_quat": np.asarray(
                        c[ControllerInputIndex.GRIP_ORIENTATION],
                        dtype=np.float32,
                    ),
                    "squeeze": float(c[ControllerInputIndex.SQUEEZE_VALUE]),
                    "trigger": float(c[ControllerInputIndex.TRIGGER_VALUE]),
                }
            except (IndexError, KeyError, TypeError, ValueError):
                actions[side] = None

        return actions
