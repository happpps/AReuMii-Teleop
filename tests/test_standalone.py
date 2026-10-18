# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Check portable configuration and the existing bridge wire protocol without XR."""

import contextlib
import io
import os
import socket
import tempfile
import unittest
from pathlib import Path

import numpy as np
from areumii_teleop.cli import parse_args
from areumii_teleop.transport import UdpBridge


class TestConfiguration(unittest.TestCase):
    def test_bundled_model_resolves_from_another_directory(self):
        previous = Path.cwd()
        with tempfile.TemporaryDirectory() as directory:
            try:
                os.chdir(directory)
                args = parse_args([])
            finally:
                os.chdir(previous)
        self.assertTrue(Path(args.urdf).is_file())
        self.assertEqual(args.mode, "sim")
        self.assertFalse(args.enable_command)

    def test_config_paths_and_cli_precedence(self):
        model = Path(parse_args([]).urdf).read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "robot.urdf").write_text(model)
            config = path / "custom.toml"
            config.write_text('[teleop]\nurdf="robot.urdf"\nfps=20.0\nmode="real"\n[robot.gripper_open]\nleft=-0.9\n')
            args = parse_args(["--config", str(config), "--fps", "25"])
        self.assertEqual(args.fps, 25)
        self.assertEqual(args.mode, "real")
        self.assertEqual(args.urdf, str(path / "robot.urdf"))
        self.assertEqual(args.robot["gripper_open"], {"left": -0.9, "right": -1.1})
        self.assertFalse(args.enable_command)

    def test_legacy_options_remain_accepted(self):
        args = parse_args(["--cmd-port", "5050", "--enable-command", "--max-joint-step-rad", "0.01"])
        self.assertEqual(args.cmd_port, 5050)
        self.assertEqual(args.max_joint_step_rad, 0.01)
        self.assertTrue(args.enable_command)

    def test_invalid_runtime_options_fail_before_startup(self):
        for option, value in (("--fps", "0"), ("--fps", "nan"), ("--cmd_port", "0"), ("--max_ee_step_m", "-1")):
            with self.subTest(option=option, value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args([option, value])

    def test_invalid_config_is_rejected(self):
        invalid = (
            "[teleop]\nenable_command=true\n",
            '[teleop]\nmode="typo"\n',
            "[teleop]\nfps=true\n",
            '[robot.joint_names]\nleft=["missing"]\n',
            '[robot.target_frame]\nright="missing"\n',
            "[robot]\njoint_margin=100.0\n",
        )
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "invalid.toml"
            for content in invalid:
                config.write_text(content)
                with self.subTest(config=content), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        parse_args(["--config", str(config)])


class TestTransport(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(1)
        self.bridge = UdpBridge(self.receiver.getsockname()[1], 0)
        self.addCleanup(self.receiver.close)
        self.addCleanup(self.bridge.rx.close)
        self.addCleanup(self.bridge.tx.close)

    def test_arm_and_gripper_command_packets(self):
        self.bridge.send("left", np.arange(7))
        self.assertEqual(self.receiver.recv(4096), b"L,0,1,2,3,4,5,6")
        self.bridge.send("right", np.zeros(7))
        self.assertEqual(self.receiver.recv(4096), b"R,0,0,0,0,0,0,0")
        self.bridge.send_gripper("left", -1.1)
        self.assertEqual(self.receiver.recv(4096), b"GL,-1.1")
        self.bridge.send_gripper("right", 0.7)
        self.assertEqual(self.receiver.recv(4096), b"GR,0.7")

    def test_feedback_packet_shapes_and_freshness(self):
        for length in (28, 32):
            packet = ",".join(str(i) for i in range(length)).encode()
            self.receiver.sendto(packet, self.bridge.rx.getsockname())
            np.testing.assert_array_equal(self.bridge.wait(), np.arange(length))
        self.assertTrue(self.bridge.fresh(1))
        self.bridge.latest_time -= 2
        self.assertFalse(self.bridge.fresh(1))

    def test_invalid_feedback_cannot_refresh_timestamp(self):
        for packet in (b"bad", b"1,2", b"nan," + b"0," * 30 + b"0"):
            self.receiver.sendto(packet, self.bridge.rx.getsockname())
        self.assertIsNone(self.bridge.poll())
        self.assertEqual(self.bridge.latest_time, 0)


if __name__ == "__main__":
    unittest.main()
