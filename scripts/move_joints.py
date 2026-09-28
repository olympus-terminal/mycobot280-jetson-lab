#!/usr/bin/env python3
"""Staged move: bring joints to target angles ONE JOINT AT A TIME, in a given order, and report what each achieved.

SUPERVISED USE ONLY: a person at the arm with a hand near its power plug.

Safety behaviour:
  - speed capped at 30; every target checked against the firmware limits before anything moves
  - joints move one at a time, in --order (choose an order that lifts away from the table first)
  - each joint waits until its angle has stopped changing (or --timeout), then the next starts
  - Ctrl-C sends stop()
  - --delay N waits N seconds first (the person walks to the arm)

Targets: six values, J1..J6; use "keep" to leave a joint where it is.

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/move_joints.py --target keep 0 0 0 0 0 --order 2 3 4 5 6 --speed 10 --delay 15
"""
import argparse
import datetime
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = "1000000"


def read_angles(mc, tries=5):
    for _ in range(tries):
        a = mc.get_angles()
        if isinstance(a, list) and len(a) == 6:
            return a
        time.sleep(0.1)
    raise RuntimeError(f"no angle reading (last: {a!r})")


def wait_settled(mc, j, timeout, still_for=1.0):
    """Poll joint j until it stops changing for `still_for` seconds or `timeout` elapses."""
    t0 = last_change = time.time()
    last = None
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        a = mc.get_angles()
        if not (isinstance(a, list) and len(a) == 6):
            continue
        v = a[j - 1]
        if last is None or abs(v - last) > 0.2:
            last, last_change = v, time.time()
        elif time.time() - last_change >= still_for:
            return v, round(time.time() - t0, 1)
    return last, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", nargs=6, required=True, help='six angles J1..J6 or "keep"')
    ap.add_argument("--order", nargs="+", type=int, required=True)
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--delay", type=float, default=0)
    ap.add_argument("--timeout", type=float, default=15.0, help="per joint")
    ap.add_argument("--port", default=DEFAULT_PORT)
    args = ap.parse_args()

    if not 1 <= args.speed <= 30:
        sys.exit("refusing: speed must be 1..30")
    targets = [None if t == "keep" else float(t) for t in args.target]
    if sorted(set(args.order)) != sorted(args.order) or any(j not in range(1, 7) for j in args.order):
        sys.exit("refusing: --order must be distinct joints 1..6")
    missing = [j for j in range(1, 7) if targets[j - 1] is not None and j not in args.order]
    if missing:
        sys.exit(f"refusing: joints {missing} have targets but aren't in --order")

    print(f"move_joints.py  {datetime.datetime.now().isoformat(timespec='seconds')}  pymycobot {pymycobot.__version__}")
    mc = MyCobot280(args.port, BAUD)
    try:
        for j in args.order:
            t = targets[j - 1]
            if t is None:
                continue
            lo, hi = mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)
            if not (lo <= t <= hi):
                sys.exit(f"refusing: J{j} target {t} outside limits {lo}..{hi}")
        start = read_angles(mc)
        print(f"start angles:  {start}")
        print(f"start coords:  {mc.get_coords()}")
        print(f"targets:       {targets}  order {args.order}  speed {args.speed}")
        if args.delay > 0:
            print(f"waiting {args.delay:.0f} s")
            time.sleep(args.delay)
        mc.clear_error_information()
        rows = []
        for j in args.order:
            t = targets[j - 1]
            if t is None:
                continue
            before = read_angles(mc)[j - 1]
            mc.send_angle(j, t, args.speed)
            reached, secs = wait_settled(mc, j, args.timeout)
            err = None if reached is None else round(reached - t, 2)
            e = mc.get_error_information()
            rows.append((j, before, t, reached, err, secs, e))
            print(f"J{j}: {before} -> target {t}: reached {reached} (err {err}, settled {secs} s, controller error {e})")
            if e not in (0, None, -1):
                print("controller error set: stopping the sequence")
                break
        end = read_angles(mc)
        print(f"end angles:    {end}")
        print(f"end coords:    {mc.get_coords()}")
    except KeyboardInterrupt:
        print("\nCtrl-C: stop()")
        mc.stop()
        return 130
    finally:
        mc._serial_port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
