#!/usr/bin/env python3
"""Hand-guide ONE joint into a target window: release it (damped), let a person move it, re-lock.

Needed when a joint sits beyond its firmware limit: the controller then reports that joint in
get_error_information() and refuses all motion commands (seen 2026-09-24 with J2 at -138.33).

SUPERVISED USE ONLY: the person must be HOLDING the arm before the joint is released.

Behaviour:
  - releases only --joint, in the default damped mode (resists, but moves by hand)
  - polls the angle every 0.1 s; as soon as it's inside [--lo, --hi] it re-locks (focus_servo)
  - re-locks at --timeout, on Ctrl-C, and on any error (finally block)

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/guide_joint.py --joint 2 --lo -133 --hi -120 --timeout 45
"""
import argparse
import datetime
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = "1000000"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    ap.add_argument("--lo", type=float, required=True)
    ap.add_argument("--hi", type=float, required=True)
    ap.add_argument("--timeout", type=float, default=45)
    ap.add_argument("--port", default=DEFAULT_PORT)
    args = ap.parse_args()
    j = args.joint

    print(f"guide_joint.py  {datetime.datetime.now().isoformat(timespec='seconds')}  pymycobot {pymycobot.__version__}")
    mc = MyCobot280(args.port, BAUD)
    lo_fw, hi_fw = mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)
    if not (lo_fw <= args.lo < args.hi <= hi_fw):
        sys.exit(f"refusing: window {args.lo}..{args.hi} not inside firmware limits {lo_fw}..{hi_fw}")
    start = mc.get_angles()
    print(f"start angles: {start}   error: {mc.get_error_information()}")
    if isinstance(start, list) and len(start) == 6 and args.lo <= start[j - 1] <= args.hi:
        print(f"J{j} already in window ({start[j - 1]}); not releasing")
        mc._serial_port.close()
        return 0
    print(f"J{j} window {args.lo}..{args.hi}; releasing J{j} (damped) for up to {args.timeout:.0f} s")
    trace, locked_in_window = [], False
    try:
        print(f"release_servo({j}) -> {mc.release_servo(j)!r}")
        t0 = time.time()
        while time.time() - t0 < args.timeout:
            a = mc.get_angles()
            if isinstance(a, list) and len(a) == 6:
                trace.append((round(time.time() - t0, 1), a[j - 1]))
                if args.lo <= a[j - 1] <= args.hi:
                    locked_in_window = True
                    break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Ctrl-C")
    finally:
        print(f"focus_servo({j}) -> {mc.focus_servo(j)!r}")
        time.sleep(0.3)
        end = mc.get_angles()
        print(f"J{j} trace (s, deg), every 5th: {trace[::5]}")
        print(f"end angles:   {end}")
        print(f"in window when locked: {locked_in_window}")
        print(f"error after: {mc.get_error_information()}")
        mc._serial_port.close()
    return 0 if locked_in_window else 1


if __name__ == "__main__":
    sys.exit(main())
