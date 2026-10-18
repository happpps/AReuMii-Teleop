# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""UDP transport shared by the real robot and simulation."""

import socket
import time

import numpy as np


class UdpBridge:
    """Exchange arm and gripper targets [rad] with a local UDP bridge."""

    def __init__(self, cmd_port: int, fb_port: int):
        self.cmd_addr = ("127.0.0.1", cmd_port)

        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("127.0.0.1", fb_port))
        self.rx.setblocking(False)

        self.latest_q = None
        self.latest_time = 0.0

    def poll(self) -> np.ndarray | None:
        """Return the latest 28- or 32-value feedback packet [rad], if available."""
        while True:
            try:
                data, _ = self.rx.recvfrom(4096)
            except BlockingIOError:
                break

            try:
                q = np.asarray(
                    [float(x) for x in data.decode().strip().split(",")],
                    dtype=float,
                )
            except Exception:
                continue

            # Existing bridge: 28D
            #   measured L7,R7 + control L7,R7
            # Gripper-aware bridge: 32D
            #   above 28 + measured gripper L,R + control gripper L,R
            if len(q) in (28, 32) and np.all(np.isfinite(q)):
                self.latest_q = q
                self.latest_time = time.monotonic()

        return None if self.latest_q is None else self.latest_q.copy()

    def wait(self, timeout: float = 5.0) -> np.ndarray:
        """Wait up to timeout [s] and return feedback joint and gripper angles [rad]."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            q = self.poll()
            if q is not None:
                return q
            time.sleep(0.01)
        raise TimeoutError("No feedback: expected 28D arm packet or 32D arm+gripper packet")

    def fresh(self, max_age: float) -> bool:
        """Return whether feedback is newer than max_age [s]."""
        self.poll()
        return self.latest_q is not None and time.monotonic() - self.latest_time <= max_age

    def send(self, side: str, q: np.ndarray) -> None:
        """Send seven joint targets [rad] for the selected arm."""
        tag = "L" if side == "left" else "R"
        msg = tag + "," + ",".join(f"{v:.12g}" for v in q)
        self.tx.sendto(msg.encode(), self.cmd_addr)

    def send_gripper(self, side: str, motor_rad: float) -> None:
        """Send a gripper motor target [rad] for the selected side."""
        # Gripper-aware bridge protocol:
        #   GL,<motor_rad>  = left gripper
        #   GR,<motor_rad>  = right gripper
        tag = "GL" if side == "left" else "GR"
        msg = f"{tag},{float(motor_rad):.12g}"
        self.tx.sendto(msg.encode(), self.cmd_addr)
