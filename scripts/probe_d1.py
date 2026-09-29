"""Read-only probe: subscribe to D1 state topics and print their receive rates.

Never publishes anything, so it is safe to run while the arm is powered.

    uv run python scripts/probe_d1.py --nic enp3s0
"""

import threading
import time
from collections import defaultdict
from dataclasses import dataclass

import tyro
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

from d1t_teleop.msg import ArmString_, PubServoInfo_

# The official sample subscribes "arm_Feedback" while the driver publishes
# "rt/arm_Feedback", so listen on both.
TOPICS = {
    "current_servo_angle": PubServoInfo_,
    "arm_Feedback": ArmString_,
    "rt/arm_Feedback": ArmString_,
}


@dataclass
class Args:
    nic: str = "eth0"
    domain_id: int = 0
    duration: float = 0.0
    """Seconds to run (0 = until Ctrl-C)."""


def main(args: Args) -> None:
    ChannelFactoryInitialize(args.domain_id, args.nic)

    lock = threading.Lock()
    counts = defaultdict(int)
    last = {}

    def make_handler(topic):
        def handler(msg):
            with lock:
                counts[topic] += 1
                last[topic] = msg

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
            time.sleep(1.0)
            with lock:
                snapshot = dict(counts)
                counts.clear()
                latest = dict(last)
            print(f"--- t={time.time() - start:5.1f}s")
            for topic in TOPICS:
                hz = snapshot.get(topic, 0)
                msg = latest.get(topic)
                if msg is None:
                    body = "(nothing received)"
                elif isinstance(msg, PubServoInfo_):
                    body = " ".join(f"{a:7.2f}" for a in msg.as_list())
                else:
                    body = msg.data[:120]
                print(f"{topic:>20s} {hz:4d} Hz  {body}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main(tyro.cli(Args))
