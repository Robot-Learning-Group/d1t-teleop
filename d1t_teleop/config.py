"""Configuration for the GELLO leader and the D1-T follower.

Values marked TODO are placeholders and must be filled in before running live.
"""

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np
from gello.agents.gello_agent import DynamixelRobotConfig

# --- GELLO leader -----------------------------------------------------------
# TODO: fill in with the output of third_party/gello_software/scripts/gello_get_offset.py
LEADER_CONFIG = DynamixelRobotConfig(
    joint_ids=(1, 2, 3, 4, 5, 6),
    joint_offsets=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    joint_signs=(1, 1, 1, 1, 1, 1),
    # (gripper dynamixel id, open [deg], closed [deg])
    gripper_config=(7, 0, 0),
)


# --- D1-T follower ----------------------------------------------------------
@dataclass
class D1TConfig:
    nic: str = "eth0"
    """PC network interface connected to the D1-T (check with `ip a`)."""

    domain_id: int = 0

    cmd_topic: str = "rt/arm_Command"
    state_topic: str = "current_servo_angle"

    # TODO(verify): joint limits [deg] for J0..J5 from the D1-T manual.
    joint_lower_deg: Tuple[float, ...] = (-135.0, -90.0, -90.0, -135.0, -90.0, -135.0)
    joint_upper_deg: Tuple[float, ...] = (135.0, 90.0, 90.0, 135.0, 90.0, 135.0)

    # TODO(verify): J6 (gripper) values for fully open / fully closed.
    gripper_open_deg: float = 0.0
    gripper_closed_deg: float = 0.0

    max_step_rad: float = np.deg2rad(3.0)
    """Max joint change per command. At 100 Hz, 3 deg/step = 300 deg/s."""

    move_duration_ms: int = 100
    """Duration passed to the arm for each command (arm-side interpolation)."""

    joint_lower: np.ndarray = field(init=False)
    joint_upper: np.ndarray = field(init=False)

    def __post_init__(self):
        self.joint_lower = np.deg2rad(self.joint_lower_deg)
        self.joint_upper = np.deg2rad(self.joint_upper_deg)
