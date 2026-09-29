"""GELLO -> D1-T teleoperation.

Dry-run by default: reads the leader and prints the commands it would send.

    # leader only (D1 not needed)
    uv run python scripts/teleop.py --gello-port /dev/serial/by-id/usb-FTDI_...

    # with the D1 connected, still not moving it
    uv run python scripts/teleop.py --gello-port ... --nic enp3s0 --use-d1

    # live
    uv run python scripts/teleop.py --gello-port ... --nic enp3s0 --use-d1 --live

We don't use gello's experiments/run_env.py: on startup it moves the follower to
a hard-coded UR pose, which would also trigger for a 7-dof D1-T.
"""

import time
from dataclasses import dataclass

import numpy as np
import tyro
from gello.agents.gello_agent import GelloAgent

from d1t_teleop.config import LEADER_CONFIG, D1TConfig
from d1t_teleop.d1t_robot import D1TRobot


@dataclass
class Args:
    gello_port: str
    """e.g. /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXX-if00-port0"""

    nic: str = "eth0"
    use_d1: bool = False
    """Connect to the D1 (subscribe to its state). Without this, leader only."""

    live: bool = False
    """Actually publish commands. Requires --use-d1."""

    hz: float = 100.0
    sync_speed_deg: float = 20.0
    """Joint speed [deg/s] used to bring the follower to the leader pose at start."""


def main(args: Args) -> None:
    assert not (args.live and not args.use_d1), "--live requires --use-d1"

    agent = GelloAgent(port=args.gello_port, dynamixel_config=LEADER_CONFIG)
    leader = agent.act({})
    print("leader [deg]:", np.round(np.rad2deg(leader[:6]), 1), "gripper:", round(leader[6], 2))

    robot = None
    if args.use_d1:
        robot = D1TRobot(D1TConfig(nic=args.nic), dry_run=not args.live)
        t0 = time.time()
        while not robot.has_state():
            if time.time() - t0 > 3.0:
                raise RuntimeError("No D1 state received. Run scripts/probe_d1.py first.")
            time.sleep(0.05)
        follower = robot.get_joint_state()
        print("follower [deg]:", np.round(np.rad2deg(follower[:6]), 1), "gripper:", round(follower[6], 2))

        # Bring the follower to the leader pose slowly.
        delta = np.abs(leader[:6] - follower[:6]).max()
        steps = max(1, int(np.rad2deg(delta) / args.sync_speed_deg * args.hz))
        print(f"syncing: max diff {np.rad2deg(delta):.1f} deg over {steps / args.hz:.1f} s")
        robot.reset_command_history()
        for q in np.linspace(follower, leader, steps):
            robot.command_joint_state(q)
            time.sleep(1.0 / args.hz)

    print("teleop running (Ctrl-C to stop)")
    period = 1.0 / args.hz
    next_t = time.time()
    try:
        while True:
            q = agent.act({})
            if robot is not None:
                robot.command_joint_state(q)
                if robot.state_age() > 0.5:
                    print(f"[warn] D1 state is {robot.state_age():.2f} s old")
            else:
                print("leader [deg]:", np.round(np.rad2deg(q[:6]), 1), "gripper:", round(q[6], 2))
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main(tyro.cli(Args))
