# AReuMii XR teleoperation quickstart

For standalone installation, bundled URDF resolution, configuration, and Docker,
see [README.md](README.md). The existing direct-script commands below remain supported.

The current entry point controls both seven-joint arms and both grippers:
`areumii_bimanual_teleop_gripper.py`. It sends joint targets [rad] over UDP
and reads feedback from the bridge. Squeezing engages each arm's clutch;
releasing it disengages that arm. The trigger controls the gripper.

```text
Quest / CloudXR -> Python XR controller -> URDF IK -> UDP
    -> C++ bridge -> Areum2_can shared memory -> SocketCAN -> motors
```

## Prerequisites

Run the Python process in an environment with LeRobot kinematics, Isaac Teleop,
NumPy, and SciPy installed. The local `real_teleop/isaac_teleop/` helpers are
loaded from the script directory. Use a URDF whose joint names and frames match
`JOINT_NAMES` and `TARGET_FRAME` in the script.

The real bridge uses the existing `Areum2_can` shared-memory layout. A snapshot
of `inc/Sharemem.hpp` is bundled locally; verify its ABI matches the CAN program.
Build from this repository root:

```bash
cmake -S real_teleop/can_bridge -B real_teleop/build/bridge -DCMAKE_BUILD_TYPE=Release
cmake --build real_teleop/build/bridge --parallel
```

The alternative `real_teleop/can_bridge/areumii_udp_shm_bridge.cpp` also sends
recorder feedback to UDP port `5007`. Both variants are built by the CMake command above.

## Simulation and dry run

The script defaults to `--mode sim` and does not send commands until
`--enable-command` is supplied. Start a compatible simulation bridge to provide
feedback, then run:

```bash
uv run python real_teleop/areumii_bimanual_teleop_gripper.py \
  --urdf real_teleop/areumii_c1_kinematics.urdf \
  --mode sim
```

Use `--cloudxr-env /path/to/profile.env` when a CloudXR profile is needed.
Once feedback and controller mapping are verified, add `--enable-command`
to send commands to the simulation bridge.

## Real robot

Start the existing CAN program in `Areum2_can`, then start the compiled bridge.
Select the real feedback ports with:

```bash
uv run python real_teleop/areumii_bimanual_teleop_gripper.py \
  --urdf real_teleop/areumii_c1_kinematics.urdf \
  --mode real
```

Verify feedback, joint signs, URDF frames, and gripper calibration before adding
`--enable-command` to activate motor commands. Keep the hardware emergency stop
accessible during physical tests.

| Mode | Command port | Feedback port |
| --- | --- | --- |
| `sim` | 5010 | 5011 |
| `real` | 5005 | 5006 |

Current Python defaults:

- Feedback timeout: `0.2 s`.
- Maximum end-effector translation step: `0.005 m/frame`.
- Maximum arm joint step: `0.02 rad/frame`.
- Maximum gripper motor step: `0.05 rad/frame`.
- IK orientation weight: `0.01`.
- Arm joint safety margin: `0.01 rad` inside the configured limits.
- Gripper target endpoints: open `-1.1 rad`, close `0.7 rad`.

Joint limits are applied to the IK solution before the per-frame command delta.
The clutch uses measured feedback when engaging and refuses stale feedback.
