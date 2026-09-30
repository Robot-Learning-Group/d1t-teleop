"""Command test: move one joint and log the feedback.

Dry-run by default (prints what it would send). All other joints, including the
gripper, are held at the angle read at startup. Commands go through D1TRobot
(SetServoAngle_ on set_servo_angle, one command right after each feedback).

    # dry-run
    uv run python scripts/cmd_test.py --nic en10 --joint 0

    # live, smooth oscillation: start -> start+5 -> start, 2 cycles @ 0.25 Hz
    uv run python scripts/cmd_test.py --nic en10 --joint 0 --live

    # live, linear moves through absolute angles at constant speed:
    # start -> -10 -> 10 -> start at 5 deg/s
    uv run python scripts/cmd_test.py --nic en10 --joint 0 --waypoints -10 10 --speed-deg 5 --live

Default profile: offset = amplitude * (1 - cos(2 pi f t)) / 2, so it starts and
ends at the start pose with zero velocity. With --waypoints, the joint moves
linearly start -> waypoints... -> start at --speed-deg.
Log goes to logs/cmd_test_<time>.npz.
"""

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import tyro

from d1t_teleop.config import D1TConfig
from d1t_teleop.d1t_robot import D1TRobot

MAX_AMPLITUDE_DEG = 20.0
"""Max distance from the start angle, for both profiles."""
MAX_SPEED_DEG = 30.0


@dataclass
class Args:
    nic: str = "eth0"
    joint: int = 0
    """Joint to move (0-5)."""
    amplitude_deg: float = 5.0
    """Peak offset from the start angle [deg]. Sign sets the direction."""
    freq: float = 0.25
    """Oscillation frequency [Hz]."""
    cycles: int = 2
    waypoints: Tuple[float, ...] = ()
    """Absolute angles [deg] to move through linearly. Overrides the cosine profile."""
    speed_deg: float = 5.0
    """Joint speed [deg/s] for --waypoints."""
    live: bool = False


def main(args: Args) -> None:
    assert 0 <= args.joint <= 5, "joint must be 0-5 (6 is the gripper)"
    cfg = D1TConfig(nic=args.nic)

    lock = threading.Lock()
    fb_t, fb_q = [], []

    def log_state(t: float, deg: np.ndarray) -> None:
        with lock:
            fb_t.append(t)
            fb_q.append(deg)

    robot = D1TRobot(cfg, dry_run=not args.live, on_state=log_state)

    t0 = time.time()
    while not robot.has_state():
        if time.time() - t0 > 3.0:
            raise RuntimeError("No D1 state received. Run scripts/probe_d1.py first.")
        time.sleep(0.05)
    start = robot.get_state_deg()

    j = args.joint
    lo, hi = cfg.joint_lower_deg[j], cfg.joint_upper_deg[j]
    mode = "" if args.live else " (dry-run)"
    print("start [deg]:", np.round(start, 1))

    if args.waypoints:
        assert 0 < args.speed_deg <= MAX_SPEED_DEG, f"speed must be in (0, {MAX_SPEED_DEG}] deg/s"
        pts = np.array([start[j], *args.waypoints, start[j]])
        for a in args.waypoints:
            assert lo <= a <= hi, f"J{j}: waypoint {a} leaves [{lo}, {hi}]"
            assert abs(a - start[j]) <= MAX_AMPLITUDE_DEG, (
                f"J{j}: waypoint {a} is {abs(a - start[j]):.1f} deg from start {start[j]:.1f} "
                f"(> {MAX_AMPLITUDE_DEG})")
        knots = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(pts)) / args.speed_deg)])
        duration = knots[-1]

        def profile(t: float) -> float:
            return float(np.interp(t, knots, pts))

        print(f"J{j}: {' -> '.join(f'{a:.1f}' for a in pts)} deg @ {args.speed_deg} deg/s "
              f"({duration:.1f} s){mode}")
    else:
        peak = start[j] + args.amplitude_deg
        assert abs(args.amplitude_deg) <= MAX_AMPLITUDE_DEG, f"|amplitude| > {MAX_AMPLITUDE_DEG} deg"
        assert lo <= peak <= hi, f"J{j}: start {start[j]:.1f} + {args.amplitude_deg} leaves [{lo}, {hi}]"
        duration = args.cycles / args.freq

        def profile(t: float) -> float:
            return start[j] + args.amplitude_deg * (1 - np.cos(2 * np.pi * args.freq * t)) / 2

        print(f"J{j}: {start[j]:.1f} -> {peak:.1f} deg, {args.cycles} cycles @ {args.freq} Hz{mode}")

    if args.live:
        time.sleep(0.5)  # let discovery match the reader before the first write

    cmd_t, cmd_q = [], []
    robot.wait_for_state()
    t_start = time.perf_counter()
    try:
        while True:
            robot.wait_for_state()
            t = time.perf_counter() - t_start
            if t > duration:
                break
            target = start.copy()
            target[j] = profile(t)
            robot.send_deg(target)
            cmd_t.append(time.perf_counter())
            cmd_q.append(target[j])
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        # Hold the start pose for a moment so the last command is not an offset one.
        for _ in range(int(cfg.feedback_hz)):
            robot.wait_for_state()
            robot.send_deg(start)
        time.sleep(0.5)

    with lock:
        ft = np.array(fb_t) - t_start
        fq = np.array(fb_q)
    ct = np.array(cmd_t) - t_start
    cq = np.array(cmd_q)

    _report(args, ct, cq, ft, fq, start)

    if args.live:
        out = Path("logs") / f"cmd_test_{time.strftime('%Y%m%d_%H%M%S')}.npz"
        out.parent.mkdir(exist_ok=True)
        np.savez(out, args=json.dumps(vars(args)), cmd_t=ct, cmd_q=cq, fb_t=ft, fb_q=fq, start=start)
        print(f"saved {out}")


def _report(args: Args, ct, cq, ft, fq, start) -> None:
    j = args.joint
    dt_cmd = np.diff(ct) * 1000
    print(f"commands: n={len(ct)}  {1000 / dt_cmd.mean():.1f} Hz  dt max {dt_cmd.max():.1f} ms")
    # Where in the D1 read cycle the commands land: time since the latest feedback.
    idx = np.searchsorted(ft, ct) - 1
    ok = idx >= 0
    since_fb = (ct[ok] - ft[idx[ok]]) * 1000
    print(f"command sent after latest feedback: p50 {np.median(since_fb):.1f} / "
          f"p99 {np.percentile(since_fb, 99):.1f} / max {since_fb.max():.1f} ms")

    m = (ft >= 0) & (ft <= ct[-1])
    dt_fb = np.diff(ft[m]) * 1000
    if len(dt_fb) > 1:
        print(f"feedback while commanding: n={m.sum()}  {1000 / dt_fb.mean():.1f} Hz  "
              f"dt p50 {np.median(dt_fb):.1f} / p99 {np.percentile(dt_fb, 99):.1f} / max {dt_fb.max():.1f} ms")
    if not args.live or m.sum() < 10:
        return

    # Lag: shift the command trace until it best matches the feedback.
    fb_j = fq[m, j]
    best = min(
        range(0, 500, 5),
        key=lambda lag_ms: np.mean((np.interp(ft[m] - lag_ms / 1000, ct, cq) - fb_j) ** 2),
    )
    err = fb_j - np.interp(ft[m], ct, cq)
    others = np.delete(np.arange(7), j)
    drift = np.abs(fq[m][:, others] - start[others]).max()
    print(f"J{j}: lag ~{best} ms, tracking error rms {np.sqrt(np.mean(err ** 2)):.2f} / max {np.abs(err).max():.2f} deg")
    print(f"      cmd range [{cq.min():.2f}, {cq.max():.2f}], feedback range [{fb_j.min():.2f}, {fb_j.max():.2f}] deg")
    print(f"other joints max drift from start: {drift:.2f} deg")
    jumps = np.abs(np.diff(fq[m], axis=0)).max()
    print(f"max feedback jump between samples (any joint): {jumps:.2f} deg")


if __name__ == "__main__":
    main(tyro.cli(Args))
