"""GELLO `Robot` implementation for the Unitree D1-T.

Joint state convention (same as other GELLO robots):
    q[0:6] = arm joints [rad], q[6] = gripper (0 = open, 1 = closed)
The D1 side uses degrees.

Commands go straight to marm_controller_node as SetServoAngle_ (one per joint),
and must be sent right after a feedback message: call `wait_for_state()` before
each `command_joint_state()`. marm_controller_node reads the servos (~11 ms) and
then publishes current_servo_angle, so the serial bus is idle for ~22 ms after a
feedback arrives. Its reads and writes share /dev/ttyS4 without a lock; writes
that land during a read make the reads time out and stall the feedback for
~330 ms.
"""

import threading
import time
from typing import Callable, Dict, Optional

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from d1t_teleop.config import D1TConfig
from d1t_teleop.msg import PubServoInfo_, SetServoAngle_, SetServoDumping_

NUM_ARM_JOINTS = 6


class D1TRobot:
    def __init__(
        self,
        config: D1TConfig,
        dry_run: bool = True,
        on_state: Optional[Callable[[float, np.ndarray], None]] = None,
    ):
        """on_state(perf_counter_time, angles_deg) is called for every feedback message."""
        self._cfg = config
        self._dry_run = dry_run
        self._on_state_hook = on_state

        self._lock = threading.Lock()
        self._state_event = threading.Event()
        self._state_deg: Optional[np.ndarray] = None
        self._state_time = 0.0
        self._last_cmd: Optional[np.ndarray] = None
        self._seq = 0

        self._lower_deg = np.array([*config.joint_lower_deg, config.gripper_lower_deg])
        self._upper_deg = np.array([*config.joint_upper_deg, config.gripper_upper_deg])

        ChannelFactoryInitialize(config.domain_id, config.nic)
        self._sub = ChannelSubscriber(config.state_topic, PubServoInfo_)
        self._sub.Init(self._on_state, 10)
        self._pub = None
        self._damping_pub = None
        if not dry_run:
            self._pub = ChannelPublisher(config.cmd_topic, SetServoAngle_)
            self._pub.Init()
            self._damping_pub = ChannelPublisher(config.damping_topic, SetServoDumping_)
            self._damping_pub.Init()

    # --- Robot protocol -----------------------------------------------------
    def num_dofs(self) -> int:
        return NUM_ARM_JOINTS + 1

    def get_joint_state(self) -> np.ndarray:
        deg = self.get_state_deg()
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
        self.send_deg(deg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        q = self.get_joint_state()
        return {
            "joint_positions": q,
            "joint_velocities": np.zeros_like(q),  # not provided by D1
            "ee_pos_quat": np.zeros(7),  # TODO: FK if needed
            "gripper_position": np.array([q[-1]]),
        }

    # --- extras -------------------------------------------------------------
    def wait_for_state(self, timeout: float = 0.1) -> bool:
        """Block until a feedback message newer than the previous call arrives.

        Send the next command right after this returns. Returns False on
        timeout (feedback stalled); sending anyway keeps the motion going.
        """
        got = self._state_event.wait(timeout)
        self._state_event.clear()
        return got

    def send_deg(self, deg: np.ndarray) -> None:
        """Send J0..J6 [deg] as-is (only clipped to the D1 limits). No rate limit."""
        deg = np.clip(np.asarray(deg, dtype=float), self._lower_deg, self._upper_deg)
        self._seq += 1
        if self._dry_run:
            print(f"[dry-run] seq={self._seq} deg={np.round(deg, 2).tolist()}")
            return
        for i, a in enumerate(deg):
            self._pub.Write(
                SetServoAngle_(seq=self._seq, id=i, angle=float(a), delay_ms=self._cfg.delay_ms)
            )

    def set_damping(self, power: int) -> None:
        """Put all 7 servos in damping mode (limp). power 0 = free; larger = stiffer
        [mW, FashionStar]. The next angle command re-enables position control.
        Call right after wait_for_state(), like send_deg()."""
        self._seq += 1
        if self._dry_run:
            print(f"[dry-run] seq={self._seq} damping power={power}")
            return
        for i in range(self.num_dofs()):
            self._damping_pub.Write(SetServoDumping_(seq=self._seq, id=i, power=int(power)))

    def get_state_deg(self) -> np.ndarray:
        """Latest J0..J6 [deg]."""
        with self._lock:
            if self._state_deg is None:
                raise RuntimeError(
                    f"No data on '{self._cfg.state_topic}' yet. "
                    "Check the NIC, the cable, and scripts/probe_d1.py."
                )
            return self._state_deg.copy()

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
        now = time.perf_counter()
        deg = np.array(msg.as_list(), dtype=float)
        with self._lock:
            self._state_deg = deg
            self._state_time = time.time()
        if self._on_state_hook is not None:
            self._on_state_hook(now, deg)
        self._state_event.set()

    def _gripper_deg_to_normalized(self, deg: float) -> float:
        o, c = self._cfg.gripper_open_deg, self._cfg.gripper_closed_deg
        if o == c:
            return 0.0
        return float(np.clip((deg - o) / (c - o), 0.0, 1.0))

    def _gripper_normalized_to_deg(self, g: float) -> float:
        o, c = self._cfg.gripper_open_deg, self._cfg.gripper_closed_deg
        return o + g * (c - o)
