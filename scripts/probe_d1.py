"""Read-only probe: subscribe to D1 state topics and print their receive rates.

Never publishes anything, so it is safe to run while the arm is powered.

    uv run python scripts/probe_d1.py --nic en10

rt/arm_Feedback carries several message kinds (servo angles from
marm_controller_node, arm status from marm_communication_node, ...), so it is
reported per (address, funcode).
"""

import json
import threading
import time
from collections import defaultdict
from dataclasses import dataclass

import numpy as np
import tyro
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

from d1t_teleop.msg import ArmString_, PubServoInfo_

TOPICS = {
    "current_servo_angle": PubServoInfo_,
    "rt/arm_Feedback": ArmString_,
}


@dataclass
class Args:
    nic: str = "eth0"
    domain_id: int = 0
    duration: float = 0.0
    """Seconds to run (0 = until Ctrl-C)."""
    interval: float = 2.0
    """Seconds between reports."""


def _key(topic: str, msg) -> str:
    if isinstance(msg, ArmString_):
        try:
            d = json.loads(msg.data)
            return f"{topic} a={d.get('address')} f={d.get('funcode')}"
        except (json.JSONDecodeError, AttributeError):
            return f"{topic} (non-json)"
    return topic


def _stats(stamps: list) -> str:
    if len(stamps) < 2:
        return f"n={len(stamps):3d}"
    dt = np.diff(stamps) * 1000.0
    return (
        f"n={len(stamps):3d} {1000.0 / dt.mean():6.2f} Hz  "
        f"dt mean {dt.mean():6.1f} / min {dt.min():6.1f} / max {dt.max():6.1f} ms"
    )


def main(args: Args) -> None:
    ChannelFactoryInitialize(args.domain_id, args.nic)

    lock = threading.Lock()
    stamps = defaultdict(list)
    last = {}

    def make_handler(topic):
        def handler(msg):
            now = time.perf_counter()
            key = _key(topic, msg)
            with lock:
                stamps[key].append(now)
                last[key] = msg

        return handler

    subs = []
    for topic, typ in TOPICS.items():
        sub = ChannelSubscriber(topic, typ)
        sub.Init(make_handler(topic), 10)
        subs.append(sub)

    print(f"listening on {list(TOPICS)} via {args.nic} (Ctrl-C to stop)")
    start = time.time()
    try:
        while args.duration <= 0 or time.time() - start < args.duration:
            time.sleep(args.interval)
            with lock:
                snapshot = {k: v[:] for k, v in stamps.items()}
                for v in stamps.values():
                    del v[:-1]  # keep the last stamp so the next window has no gap
                latest = dict(last)
            print(f"--- t={time.time() - start:5.1f}s")
            if not snapshot:
                print("  (nothing received)")
            for key in sorted(snapshot):
                msg = latest[key]
                if isinstance(msg, PubServoInfo_):
                    body = " ".join(f"{a:7.2f}" for a in msg.as_list())
                else:
                    body = msg.data[:100]
                print(f"  {key:<28s} {_stats(snapshot[key])}")
                print(f"  {'':<28s} {body}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(tyro.cli(Args))
