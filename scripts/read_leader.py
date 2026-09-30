"""Print the raw GELLO leader joint angles (no offsets / signs applied).

Use it to check the U2D2 connection and the Dynamixel IDs, and to find each
joint's rotation direction before running leader_offset.py.

    ls /dev/cu.usbserial-*          # macOS (Linux: /dev/serial/by-id/...). A trailing "%" is zsh's, not part of the name
    uv run python scripts/read_leader.py --port /dev/cu.usbserial-XXXX
"""

import time
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tyro
from gello.dynamixel.driver import DynamixelDriver


@dataclass
class Args:
    port: str
    ids: Tuple[int, ...] = (0, 1, 2, 3, 4, 5, 6)
    """Dynamixel IDs, J0..J5 then the gripper."""
    baudrate: int = 57600
    hz: float = 10.0


def main(args: Args) -> None:
    # gello silently falls back to a fake driver that returns zeros; fail instead.
    driver = DynamixelDriver(
        list(args.ids), port=args.port, baudrate=args.baudrate, use_fake_fallback=False
    )
    print(f"reading IDs {list(args.ids)} on {args.port} @ {args.baudrate} (Ctrl-C to stop)")
    print("raw [deg]:  " + "  ".join(f"id{i:<5d}" for i in args.ids))
    try:
        while True:
            deg = np.rad2deg(driver.get_joints())
            print("           " + "  ".join(f"{a:7.1f}" for a in deg))
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
