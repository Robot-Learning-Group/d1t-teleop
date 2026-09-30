"""Compute the GELLO leader joint offsets (replaces gello's gello_get_offset.py,
which hard-codes Dynamixel IDs 1..7; ours start at 0).

Put the leader in the same pose as the D1 (roughly: within +/-45 deg per joint),
pass that pose and the joint signs, and copy the result into LEADER_CONFIG.

    uv run python scripts/leader_offset.py --port /dev/cu.usbserial-XXXX \\
        --start-deg 0 0 0 0 0 0 --joint-signs 1 1 1 1 1 1

Same method as gello: each offset is the multiple of pi/2 (within +/-8 pi) that
brings sign * (raw - offset) closest to the start angle. The residual error is
printed; it should be well under 45 deg, otherwise the pose or the sign is off.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tyro
from gello.dynamixel.driver import DynamixelDriver

from d1t_teleop.config import LEADER_CONFIG


@dataclass
class Args:
    port: str
    start_deg: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    """D1 joint angles J0..J5 [deg] of the pose the leader is placed in."""
    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1)
    """+1 if the leader raw angle increases when the D1 joint angle increases, else -1."""
    baudrate: int = 57600


def main(args: Args) -> None:
    ids = list(LEADER_CONFIG.joint_ids)
    gripper_id = LEADER_CONFIG.gripper_config[0]
    assert len(args.start_deg) == len(ids) == len(args.joint_signs)
    assert all(s in (1, -1) for s in args.joint_signs), args.joint_signs

    driver = DynamixelDriver(
        ids + [gripper_id], port=args.port, baudrate=args.baudrate, use_fake_fallback=False
    )
    try:
        for _ in range(10):
            driver.get_joints()  # warmup
        raw = driver.get_joints()
    finally:
        driver.close()

    start = np.deg2rad(args.start_deg)
    candidates = np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1)  # multiples of pi/2
    offsets, errors = [], []
    for i, sign in enumerate(args.joint_signs):
        err = np.abs(sign * (raw[i] - candidates) - start[i])
        k = int(np.argmin(err))
        offsets.append(candidates[k])
        errors.append(np.rad2deg(err[k]))

    print("raw [deg]:        ", np.round(np.rad2deg(raw), 1).tolist())
    print("residual [deg]:   ", np.round(errors, 1).tolist())
    if max(errors) > 30:
        print("[warn] residual > 30 deg: check the pose and the joint signs")
    print()
    print("joint_offsets=(" + ", ".join(f"{int(round(o / (np.pi / 2)))} * np.pi / 2" for o in offsets) + "),")
    print("joint_signs=" + str(tuple(args.joint_signs)) + ",")
    print(f"gripper raw now: {np.rad2deg(raw[-1]):.1f} deg  (record it with the gripper fully open and fully closed)")


if __name__ == "__main__":
    main(tyro.cli(Args))
