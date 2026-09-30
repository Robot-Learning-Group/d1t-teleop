"""GELLO -> D1-T teleoperation.

Dry-run by default: reads the leader and prints the commands it would send.

    # leader only (D1 not needed)
    uv run python scripts/teleop.py --gello-port /dev/serial/by-id/usb-FTDI_...

    # with the D1 connected, still not moving it
    uv run python scripts/teleop.py --gello-port ... --nic enp3s0 --use-d1

    # live
    uv run python scripts/teleop.py --gello-port ... --nic enp3s0 --use-d1 --live

Commands are sent once per D1 feedback message (30 Hz), see D1TRobot.

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

    sync_speed_deg: float = 20.0
    """Joint speed [deg/s] used to bring the follower to the leader pose at start."""


def main(args: Args) -> None:
    assert not (args.live and not args.use_d1), "--live requires --use-d1"

    agent = GelloAgent(port=args.gello_port, dynamixel_config=LEADER_CONFIG)
    # gello silently falls back to a fake driver (all zeros) if the port can't be
    # opened, which would command the D1 to the zero pose. Refuse to run then.
    if getattr(agent._robot._driver, "_is_fake", False):
        raise RuntimeError(f"Could not open the GELLO on {args.gello_port} (fake driver in use).")
    leader = agent.act({})
    print("leader [deg]:", np.round(np.rad2deg(leader[:6]), 1), "gripper:", round(leader[6], 2))

    cfg = D1TConfig(nic=args.nic)
    robot = None
    if args.use_d1:
        robot = D1TRobot(cfg, dry_run=not args.live)
        t0 = time.time()
        while not robot.has_state():
            if time.time() - t0 > 3.0:
                raise RuntimeError("No D1 state received. Run scripts/probe_d1.py first.")
            time.sleep(0.05)
        follower = robot.get_joint_state()
        print("follower [deg]:", np.round(np.rad2deg(follower[:6]), 1), "gripper:", round(follower[6], 2))

        # Bring the follower to the leader pose slowly.
        delta = np.abs(leader[:6] - follower[:6]).max()
        steps = max(1, int(np.rad2deg(delta) / args.sync_speed_deg * cfg.feedback_hz))
        print(f"syncing: max diff {np.rad2deg(delta):.1f} deg over {steps / cfg.feedback_hz:.1f} s")
        robot.reset_command_history()
        for q in np.linspace(follower, leader, steps):
            robot.wait_for_state()
            robot.command_joint_state(q)

    print("teleop running (Ctrl-C to stop)")
    try:
        while True:
            # Read the leader first so the command goes out right after the feedback.
            q = agent.act({})
            if robot is not None:
                if not robot.wait_for_state():
                    print(f"[warn] D1 state is {robot.state_age():.2f} s old")
                robot.command_joint_state(q)
            else:
                print("leader [deg]:", np.round(np.rad2deg(q[:6]), 1), "gripper:", round(q[6], 2))
                time.sleep(1.0 / cfg.feedback_hz)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main(tyro.cli(Args))
