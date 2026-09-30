"""Window showing D1 angles and raw GELLO leader angles side by side, live. Read-only.

For finding joint_signs: move the same joint on both in the same direction and
compare the change (d) since the last "Reset d".

    uv run python scripts/compare_live.py --nic en10 --port /dev/cu.usbserial-XXXX
"""

import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import tyro
from gello.dynamixel.driver import DynamixelDriver
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from d1t_teleop.config import LEADER_CONFIG, D1TConfig
from d1t_teleop.d1t_robot import D1TRobot

NAMES = [f"J{i}" for i in range(6)] + ["grip"]
COLS = ["D1 [deg]", "d", "leader raw [deg]", "d"]
COL_X = [0.30, 0.47, 0.72, 0.90]


@dataclass
class Args:
    nic: str
    port: str
    baudrate: int = 57600
    hz: float = 10.0


def main(args: Args) -> None:
    ids = list(LEADER_CONFIG.joint_ids) + [LEADER_CONFIG.gripper_config[0]]
    leader = DynamixelDriver(ids, port=args.port, baudrate=args.baudrate, use_fake_fallback=False)
    robot = D1TRobot(D1TConfig(nic=args.nic), dry_run=True)  # never publishes

    t0 = time.time()
    while not robot.has_state():
        if time.time() - t0 > 3.0:
            raise RuntimeError("No D1 state received. Run scripts/probe_d1.py first.")
        time.sleep(0.05)
    for _ in range(10):
        leader.get_joints()  # warmup

    def read():
        return robot.get_state_deg(), np.rad2deg(leader.get_joints())

    ref = list(read())

    fig = plt.figure("D1 / GELLO", figsize=(8, 5))
    ax = fig.add_axes([0, 0.12, 1, 0.88])
    ax.axis("off")
    for x, c in zip(COL_X, COLS):
        ax.text(x, 0.95, c, ha="right", fontsize=12, fontweight="bold")
    ax.axhline(0.91, 0.03, 0.97, color="gray", lw=0.8)
    cells = []
    for r, n in enumerate(NAMES):
        y = 0.82 - r * 0.12
        ax.text(0.05, y, n, fontsize=14, fontweight="bold")
        cells.append([ax.text(x, y, "", ha="right", fontsize=14, family="monospace") for x in COL_X])

    def reset(_=None):
        ref[:] = read()

    btn = Button(fig.add_axes([0.40, 0.02, 0.20, 0.07]), "Reset d")
    btn.on_clicked(reset)

    def update(_):
        d1, ld = read()
        for r in range(len(NAMES)):
            vals = [d1[r], d1[r] - ref[0][r], ld[r], ld[r] - ref[1][r]]
            for c, (cell, v) in enumerate(zip(cells[r], vals)):
                cell.set_text(f"{v:+.1f}" if c % 2 else f"{v:.1f}")
        return [c for row in cells for c in row]

    anim = FuncAnimation(fig, update, interval=1000 / args.hz, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        leader.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
