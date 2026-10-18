# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from isaac_teleop.clutch import Clutch
from isaac_teleop.config_isaac_teleop import XRControllerConfig
from scipy.spatial.transform import Rotation

from .cli import parse_args
from .kinematics import Kinematics
from .transport import UdpBridge
from .xr import BimanualXRController

JOINT_NAMES = {
    "left": [
        "shoulder_pitch_joint_L",
        "shoulder_roll_joint_L",
        "shouler_yaw_joint_L",
        "elbow_joint_L",
        "wrist_roll_joint_L",
        "wrist_yaw_joint_L",
        "wrist_pitch_joint_L",
    ],
    "right": [
        "shoulder_pitch_joint_R",
        "shoulder_roll_joint_R",
        "shoulder_yaw_joint_R",
        "elbow_joint_R",
        "wrist_roll_joint_R",
        "wrist_yaw_joint_R",
        "wrist_pitch_joint_R",
    ],
}

TARGET_FRAME = {
    "left": "gripper_hand_link_L_1",
    "right": "gripper_hand_link_R_1",
}

JOINT_LOWER = np.array([-3.142, -0.262, -1.396, -0.873, -1.745, -0.524, -1.571])
JOINT_UPPER = np.array([3.142, 2.967, 1.396, 2.094, 1.745, 0.524, 1.571])
JOINT_MARGIN = 0.01

# Actual gripper motor angles measured on the real robot [rad].
# Trigger 0.0 -> OPEN, Trigger 1.0 -> CLOSE.
GRIPPER_OPEN = {
    "left": -1.1,
    "right": -1.1,
}
GRIPPER_CLOSE = {
    "left": 0.7,
    "right": 0.7,
}

MODE_PORTS = {
    "sim": (5010, 5011),
    "real": (5005, 5006),
}


def clamp_joint_step(current, target, max_step):
    return current + np.clip(target - current, -max_step, max_step)


def clamp_ee_step(current, target, max_step):
    d = target - current
    n = np.linalg.norm(d)

    if n <= max_step or n == 0:
        return target

    return current + d * (max_step / n)


def warn_throttled(st, message, period=0.25):
    now = time.monotonic()
    if now - st["last_warn"] >= period:
        print(message)
        st["last_warn"] = now


def main(args=None):
    if args is None:
        args = parse_args()
    robot = args.robot
    joint_names = robot["joint_names"]
    target_frame = robot["target_frame"]
    gripper_open = robot["gripper_open"]
    gripper_close = robot["gripper_close"]

    if not Path(args.urdf).exists():
        raise FileNotFoundError(args.urdf)

    cmd_default, fb_default = MODE_PORTS[args.mode]
    cmd_port = args.cmd_port or cmd_default
    fb_port = args.feedback_port or fb_default

    bridge = UdpBridge(cmd_port, fb_port)
    q14 = bridge.wait()

    q_measured = {
        "left": q14[:7].copy(),
        "right": q14[7:14].copy(),
    }
    q_control = {
        "left": q14[14:21].copy(),
        "right": q14[21:28].copy(),
    }

    # A gripper-aware bridge appends:
    # [28] measured left gripper
    # [29] measured right gripper
    # [30] control left gripper
    # [31] control right gripper
    gripper_feedback_available = len(q14) == 32
    q_gripper_measured = {
        "left": float(q14[28]) if gripper_feedback_available else None,
        "right": float(q14[29]) if gripper_feedback_available else None,
    }
    q_gripper_control = {
        "left": float(q14[30]) if gripper_feedback_available else None,
        "right": float(q14[31]) if gripper_feedback_available else None,
    }

    lo = np.asarray(robot["joint_lower"]) + robot["joint_margin"]
    hi = np.asarray(robot["joint_upper"]) - robot["joint_margin"]

    arms = {}

    for side in ("left", "right"):
        kin = Kinematics(
            args.urdf,
            target_frame[side],
            joint_names[side],
        )

        T = kin.fk(q_measured[side])

        arms[side] = {
            "kin": kin,
            "clutch": Clutch(T),
            # IK state follows actual/measured robot geometry.
            "q_seed": q_measured[side].copy(),
            # Motor command state follows the CURRENT CAN/SHM control target.
            "q_cmd": q_control[side].copy(),
            "last_ee": T[:3, 3].copy(),
            "enabled": False,
            "last_warn": 0.0,
            "last_gripper_print": 0.0,
        }

    xr = BimanualXRController(
        XRControllerConfig(
            app_name="AreumiiBimanualTeleop",
            hand_side="right",  # subclass에서는 실제 side 선택에 사용 안 함
            clutch_threshold=args.clutch_threshold,
            cloudxr_env_file=args.cloudxr_env,
            auto_launch_cloudxr=True,
        )
    )

    print("[AREUMII] Connecting XR...")
    xr.connect()
    print("[AREUMII] LEFT squeeze = left arm / RIGHT squeeze = right arm")
    print("[AREUMII] LEFT/RIGHT trigger = corresponding gripper")
    print(
        "[GRIPPER] endpoints [rad]: "
        f"L open={gripper_open['left']:.2f} close={gripper_close['left']:.2f} | "
        f"R open={gripper_open['right']:.2f} close={gripper_close['right']:.2f}"
    )
    if not gripper_feedback_available:
        print(
            "[GRIPPER] Current bridge feedback is 28D (arm only). "
            "Gripper commands are disabled until a 32D gripper-aware bridge is used."
        )

    period = 1.0 / args.fps

    try:
        while True:
            t0 = time.perf_counter()

            q_live = bridge.poll()
            if q_live is not None:
                q_measured["left"] = q_live[:7]
                q_measured["right"] = q_live[7:14]
                q_control["left"] = q_live[14:21]
                q_control["right"] = q_live[21:28]

                if len(q_live) == 32:
                    gripper_feedback_available = True
                    q_gripper_measured["left"] = float(q_live[28])
                    q_gripper_measured["right"] = float(q_live[29])
                    q_gripper_control["left"] = float(q_live[30])
                    q_gripper_control["right"] = float(q_live[31])

            actions = xr.get_action()

            for side in ("left", "right"):
                st = arms[side]
                a = actions[side]

                pose_valid = False

                if a is not None:
                    pos = np.asarray(a["grip_pos"], dtype=float)
                    quat = np.asarray(a["grip_quat"], dtype=float)
                    quat_norm = np.linalg.norm(quat)

                    pose_valid = np.all(np.isfinite(pos)) and np.all(np.isfinite(quat)) and quat_norm > 1e-6

                    if pose_valid:
                        quat = quat / quat_norm

                # Gripper is independent of the squeeze/clutch.
                # The trigger directly interpolates the measured motor-angle
                # endpoints: 0=open, 1=close.
                if pose_valid and gripper_feedback_available and q_gripper_control[side] is not None:
                    trigger = float(np.clip(a["trigger"], 0.0, 1.0))
                    gripper_target = gripper_open[side] + trigger * (gripper_close[side] - gripper_open[side])

                    # Rate-limit relative to the CURRENT SHM control target so a
                    # trigger jump cannot instantly request the full motor range.
                    gripper_cmd = float(
                        q_gripper_control[side]
                        + np.clip(
                            gripper_target - q_gripper_control[side],
                            -args.max_gripper_step_rad,
                            args.max_gripper_step_rad,
                        )
                    )

                    if args.enable_command and bridge.fresh(args.feedback_timeout_s):
                        bridge.send_gripper(side, gripper_cmd)

                    now_grip = time.monotonic()
                    if now_grip - st["last_gripper_print"] >= 0.25:
                        # print(
                        #     f"[{side.upper()} GRIP] "
                        #     f"trigger={trigger:.2f} "
                        #     f"target={gripper_target:.3f} "
                        #     f"cmd={gripper_cmd:.3f} "
                        #     f"measured={q_gripper_measured[side]:.3f}"
                        # )
                        st["last_gripper_print"] = now_grip

                enabled = pose_valid and a["squeeze"] > args.clutch_threshold

                if enabled and not st["enabled"]:
                    if args.enable_command and not bridge.fresh(args.feedback_timeout_s):
                        warn_throttled(st, f"[{side.upper()} STOP] feedback stale on clutch engage")
                        enabled = False
                    else:
                        T = st["kin"].fk(q_measured[side])

                        st["clutch"].engage(
                            pos,
                            quat,
                            measured_base_T_ee=T,
                        )

                        # IMPORTANT handoff:
                        #   measured q -> FK / IK geometry
                        #   control q  -> motor command continuity
                        # Do not overwrite the current hold command with measured q.
                        st["q_seed"] = q_measured[side].copy()
                        st["q_cmd"] = q_control[side].copy()
                        st["last_ee"] = T[:3, 3].copy()

                        print(
                            f"[{side.upper()}] clutch ENGAGED "
                            f"measured={np.round(q_measured[side], 3)} "
                            f"control={np.round(q_control[side], 3)}"
                        )

                if enabled:
                    if args.enable_command and not bridge.fresh(args.feedback_timeout_s):
                        warn_throttled(st, f"[{side.upper()} STOP] feedback stale")
                        enabled = False
                    else:
                        ee_pos, ee_quat = st["clutch"].rebase(pos, quat)

                        ee_pos = clamp_ee_step(
                            st["last_ee"],
                            np.asarray(ee_pos, dtype=float),
                            args.max_ee_step_m,
                        )
                        st["last_ee"] = ee_pos.copy()

                        T = np.eye(4)
                        T[:3, :3] = Rotation.from_quat(ee_quat).as_matrix()
                        T[:3, 3] = ee_pos

                        q_ik = st["kin"].ik(
                            st["q_seed"],
                            T,
                            args.orientation_weight,
                        )

                        # T_meas = st["kin"].fk(q_measured[side])
                        # ee_meas = T_meas[:3, 3]

                        # now = time.monotonic()
                        # if now - st.get("last_debug", 0.0) >= 0.2:
                        #     err_mm = (ee_pos - ee_meas) * 1000.0

                        #     print(
                        #         f"[{side.upper()} DEBUG] "
                        #         f"target={np.round(ee_pos, 3)} "
                        #         f"actual={np.round(ee_meas, 3)} "
                        #         f"err_mm={np.round(err_mm, 1)} "
                        #         f"q_ik={np.round(q_ik, 2)}"
                        #     )

                        #     st["last_debug"] = now

                        if not np.all(np.isfinite(q_ik)):
                            warn_throttled(st, f"[{side.upper()} STOP] IK non-finite: {np.round(q_ik, 3)}")

                        else:
                            # Instead of dropping the entire 7-DOF command when
                            # one joint reaches the safety margin, clamp only the
                            # offending joint(s) to the soft limit.
                            q_safe = np.clip(q_ik, lo, hi)

                            clipped = np.where(np.abs(q_safe - q_ik) > 1e-9)[0]
                            if len(clipped):
                                warn_throttled(
                                    st,
                                    f"[{side.upper()} LIMIT CLIP] "
                                    f"joints={clipped.tolist()} "
                                    f"ik={np.round(q_ik, 3)} "
                                    f"safe={np.round(q_safe, 3)}",
                                )

                            # Keep the original command structure:
                            # apply only the IK joint delta to the existing motor
                            # command, with the same per-frame joint-step limit.
                            ik_delta = q_safe - st["q_seed"]
                            ik_delta = np.clip(
                                ik_delta,
                                -args.max_joint_step_rad,
                                args.max_joint_step_rad,
                            )
                            q_cmd = st["q_cmd"] + ik_delta

                            st["q_seed"] = q_safe.copy()
                            st["q_cmd"] = q_cmd.copy()

                            if args.enable_command:
                                bridge.send(side, q_cmd)

                if st["enabled"] and not enabled:
                    if not pose_valid:
                        print(f"[{side.upper()} STOP] XR pose/tracking invalid")
                    else:
                        print(f"[{side.upper()}] clutch RELEASED")

                st["enabled"] = enabled

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)

    except KeyboardInterrupt:
        print("\n[AREUMII] stop")

    finally:
        xr.disconnect()


if __name__ == "__main__":
    main()
