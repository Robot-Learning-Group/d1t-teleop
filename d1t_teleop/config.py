"""Configuration for the GELLO leader and the D1-T follower.

Values marked TODO are placeholders and must be filled in before running live.
"""

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np
from gello.agents.gello_agent import DynamixelRobotConfig

# --- GELLO leader -----------------------------------------------------------
# Our leader's Dynamixel IDs start at 0 (J0..J5 = 0..5, gripper = 6).
# Offsets from scripts/leader_offset.py in the D1 default pose (0, -90, 90, 0, 0, 0),
# residual <= 2.4 deg (2026-09-30).
LEADER_CONFIG = DynamixelRobotConfig(
    joint_ids=(0, 1, 2, 3, 4, 5),
    joint_offsets=(0 * np.pi / 2, -1 * np.pi / 2, 1 * np.pi / 2, 4 * np.pi / 2, 4 * np.pi / 2, 4 * np.pi / 2),
    joint_signs=(-1, -1, -1, -1, -1, -1),  # checked with scripts/compare_live.py (gripper: +1)
    # (gripper dynamixel id, leader raw open [deg], leader raw closed [deg])
    gripper_config=(6, 28.0, -33.8),
)


# --- D1-T follower ----------------------------------------------------------
@dataclass
class D1TConfig:
    nic: str = "eth0"
    """PC network interface connected to the D1-T (check with `ip a`)."""

    domain_id: int = 0

    state_topic: str = "current_servo_angle"
    cmd_topic: str = "set_servo_angle"
    """SetServoAngle_ straight to marm_controller_node (bypasses the JSON layer)."""
    damping_topic: str = "set_servo_dumping"

    # Same limits marm_controller_node clips to.
    joint_lower_deg: Tuple[float, ...] = (-135.0, -90.0, -90.0, -135.0, -90.0, -135.0)
    joint_upper_deg: Tuple[float, ...] = (135.0, 90.0, 90.0, 135.0, 90.0, 135.0)
    gripper_lower_deg: float = -20.0
    gripper_upper_deg: float = 50.0

    # J6 (gripper). By hand it goes from 70 (open) to -23.8 (closed), but
    # marm_controller_node clips commands to [-20, 50], so use the reachable range.
    gripper_open_deg: float = 50.0
    gripper_closed_deg: float = -20.0

    feedback_hz: float = 30.0
    """current_servo_angle rate (marm_controller_node patched to 30 Hz). One command is sent per feedback."""

    delay_ms: int = 33
    """delay_ms in SetServoAngle_ (marm_controller_node derives the servo move time from it)."""

    max_joint_speed_deg: float = 100.0
    """Max joint speed [deg/s] of the commands. marm_controller_node caps at ~115 deg/s anyway."""

    joint_lower: np.ndarray = field(init=False)
    joint_upper: np.ndarray = field(init=False)
    max_step_rad: float = field(init=False)
    """Max joint change per command, derived from max_joint_speed_deg / feedback_hz."""

    def __post_init__(self):
        self.joint_lower = np.deg2rad(self.joint_lower_deg)
        self.joint_upper = np.deg2rad(self.joint_upper_deg)
        self.max_step_rad = np.deg2rad(self.max_joint_speed_deg / self.feedback_hz)
