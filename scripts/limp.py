"""Put the D1 in damping mode so it can be moved by hand.

The arm is no longer held: joints under load will sag or drop. Support it by
hand (or have it in a resting pose) before running. Any angle command
(cmd_test.py, teleop.py) makes the servos hold position again.

    uv run python scripts/limp.py --nic en10              # power 0 = free
    uv run python scripts/limp.py --nic en10 --power 300  # some resistance
"""

import time
from dataclasses import dataclass

import tyro

from d1t_teleop.config import D1TConfig
from d1t_teleop.d1t_robot import D1TRobot


@dataclass
class Args:
    nic: str
    power: int = 0
    """Damping power [mW]. 0 = free, larger = more resistance."""


def main(args: Args) -> None:
    robot = D1TRobot(D1TConfig(nic=args.nic), dry_run=False)
    time.sleep(0.5)  # let discovery match the reader before the first write
    if not robot.wait_for_state(timeout=1.0):
        raise RuntimeError("No D1 state received. Run scripts/probe_d1.py first.")
    robot.set_damping(args.power)
    print(f"damping power={args.power} sent to J0..J6")


if __name__ == "__main__":
    main(tyro.cli(Args))
