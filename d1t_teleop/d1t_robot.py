"""GELLO `Robot` implementation for the Unitree D1-T.

Joint state convention (same as other GELLO robots):
    q[0:6] = arm joints [rad], q[6] = gripper (0 = open, 1 = closed)
The D1 side uses degrees.
"""

import json
import threading
import time
from typing import Dict, Optional

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from d1t_teleop.config import D1TConfig
from d1t_teleop.msg import ArmString_, PubServoInfo_

NUM_ARM_JOINTS = 6


class D1TRobot:
    def __init__(self, config: D1TConfig, dry_run: bool = True):
        self._cfg = config
        self._dry_run = dry_run

        self._lock = threading.Lock()
        self._state_deg: Optional[np.ndarray] = None
        self._state_time = 0.0
        self._last_cmd: Optional[np.ndarray] = None
        self._seq = 0

        ChannelFactoryInitialize(config.domain_id, config.nic)
        self._sub = ChannelSubscriber(config.state_topic, PubServoInfo_)
        self._sub.Init(self._on_state, 10)
        self._pub = None
        if not dry_run:
            self._pub = ChannelPublisher(config.cmd_topic, ArmString_)
            self._pub.Init()

    # --- Robot protocol -----------------------------------------------------
    def num_dofs(self) -> int:
        return NUM_ARM_JOINTS + 1

    def get_joint_state(self) -> np.ndarray:
        with self._lock:
            if self._state_deg is None:
                raise RuntimeError(
                    f"No data on '{self._cfg.state_topic}' yet. "
                    "Check the NIC, the cable, and scripts/probe_d1.py."
                )
            deg = self._state_deg.copy()
        q = np.zeros(self.num_dofs())
        q[:NUM_ARM_JOINTS] = np.deg2rad(deg[:NUM_ARM_JOINTS])
        q[-1] = self._gripper_deg_to_normalized(deg[NUM_ARM_JOINTS])
        return q

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self.num_dofs(), joint_state
        target = np.asarray(joint_state, dtype=float).copy()

        target[:NUM_ARM_JOINTS] = np.clip(
            target[:NUM_ARM_JOINTS], self._cfg.joint_lower, self._cfg.joint_upper
        )
        target[-1] = np.clip(target[-1], 0.0, 1.0)

        if self._last_cmd is not None:
            step = target[:NUM_ARM_JOINTS] - self._last_cmd[:NUM_ARM_JOINTS]
            step = np.clip(step, -self._cfg.max_step_rad, self._cfg.max_step_rad)
            target[:NUM_ARM_JOINTS] = self._last_cmd[:NUM_ARM_JOINTS] + step
        self._last_cmd = target

        deg = np.zeros(self.num_dofs())
        deg[:NUM_ARM_JOINTS] = np.rad2deg(target[:NUM_ARM_JOINTS])
        deg[-1] = self._gripper_normalized_to_deg(target[-1])
        msg = self._build_command(deg)

        if self._dry_run:
            print(f"[dry-run] {msg}")
        else:
            self._pub.Write(ArmString_(data=msg))

    def get_observations(self) -> Dict[str, np.ndarray]:
        q = self.get_joint_state()
        return {
            "joint_positions": q,
            "joint_velocities": np.zeros_like(q),  # not provided by D1
            "ee_pos_quat": np.zeros(7),  # TODO: FK if needed
            "gripper_position": np.array([q[-1]]),
        }

    # --- extras -------------------------------------------------------------
    def has_state(self) -> bool:
        with self._lock:
            return self._state_deg is not None

    def state_age(self) -> float:
        """Seconds since the last state message."""
        with self._lock:
            return time.time() - self._state_time

    def reset_command_history(self) -> None:
        """Forget the last command so the next one is not rate-limited against it."""
        self._last_cmd = None

    # --- internals ----------------------------------------------------------
    def _on_state(self, msg: PubServoInfo_) -> None:
        with self._lock:
            self._state_deg = np.array(msg.as_list(), dtype=float)
            self._state_time = time.time()

    def _build_command(self, deg: np.ndarray) -> str:
        # TODO(verify): multi-joint command format. Check funcode / mode / field
        # names against the D1 SDK docs before running live. Everything that
        # depends on the command format is contained in this method.
        self._seq += 1
        data = {"mode": 1}
        for i, a in enumerate(deg):
            data[f"angle{i}"] = round(float(a), 2)
        return json.dumps(
            {"seq": self._seq, "address": 1, "funcode": 2, "data": data},
            separators=(",", ":"),
        )

    def _gripper_deg_to_normalized(self, deg: float) -> float:
        o, c = self._cfg.gripper_open_deg, self._cfg.gripper_closed_deg
        if o == c:
            return 0.0
        return float(np.clip((deg - o) / (c - o), 0.0, 1.0))

    def _gripper_normalized_to_deg(self, g: float) -> float:
        o, c = self._cfg.gripper_open_deg, self._cfg.gripper_closed_deg
        return o + g * (c - o)
