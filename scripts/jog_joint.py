#!/usr/bin/env python3
"""Move ONE joint of the myCobot 280 by a small relative amount, then report what happened.

SUPERVISED USE ONLY: a person must be at the arm with a hand near its power plug.

Safety behaviour:
  - refuses steps larger than --max-step (default 10 deg) and speeds above 30
  - refuses targets outside the firmware joint limits (read from the controller)
  - sends only the one joint; other joints aren't commanded
  - Ctrl-C (or a timeout) sends stop()

Remote-supervision options (the person can't see this terminal from the arm):
  --delay N      wait N seconds before moving; the ATOM LED shows the state:
                 YELLOW = waiting (walk to the arm), RED = moving, GREEN = reached target, BLUE = no motion / refused
  --clear-error  call clear_error_information() just before sending the move

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/jog_joint.py --joint 1 --delta 5 --speed 15
    ~/venvs/mycobot/bin/python scripts/jog_joint.py --joint 2 --delta 5 --speed 10 --delay 60 --clear-error
"""
import argparse
import datetime
import sys
import time

import pymycobot
from pymycobot import MyCobot280

YELLOW, RED, GREEN, BLUE = (255, 160, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255)
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = "1000000"  # verified 2026-09-24 (probe_arm.py)


def read_angles(mc, tries=5):
    for _ in range(tries):
        a = mc.get_angles()
        if isinstance(a, list) and len(a) == 6:
            return a
        time.sleep(0.1)
    raise RuntimeError(f"no angle reading from controller (last: {a!r})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    ap.add_argument("--delta", type=float, required=True, help="degrees, relative to the current angle")
    ap.add_argument("--speed", type=int, default=15)
    ap.add_argument("--max-step", type=float, default=10.0)
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--delay", type=float, default=0, help="seconds to wait (LED yellow) before moving")
    ap.add_argument("--clear-error", action="store_true")
    args = ap.parse_args()

    if abs(args.delta) > args.max_step:
        sys.exit(f"refusing: |delta| {abs(args.delta)} > max-step {args.max_step}")
    if not 1 <= args.speed <= 30:
        sys.exit("refusing: speed must be 1..30 for jogging")

    print(f"jog_joint.py  {datetime.datetime.now().isoformat(timespec='seconds')}  pymycobot {pymycobot.__version__}")
    mc = MyCobot280(args.port, BAUD)
    j = args.joint
    try:
        lo, hi = mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)
        start = read_angles(mc)
        target = round(start[j - 1] + args.delta, 2)
        print(f"start angles: {start}")
        print(f"J{j}: {start[j - 1]} -> target {target}  (firmware limits {lo}..{hi})  speed {args.speed}")
        if not (isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and lo <= target <= hi):
            sys.exit(f"refusing: target {target} outside limits {lo}..{hi}")

        if args.delay > 0:
            mc.set_color(*YELLOW)
            print(f"LED yellow: waiting {args.delay:.0f} s before moving")
            time.sleep(args.delay)
        if args.clear_error:
            print(f"error before clear: {mc.get_error_information()!r}; clear -> {mc.clear_error_information()!r}")
        mc.set_color(*RED)
        t0 = time.time()
        reply = mc.send_angle(j, target, args.speed)
        print(f"send_angle reply: {reply!r}")
        trace = []
        while time.time() - t0 < args.timeout:
            time.sleep(0.25)
            a = mc.get_angles()
            if isinstance(a, list) and len(a) == 6:
                trace.append((round(time.time() - t0, 2), a[j - 1]))
                if abs(a[j - 1] - target) < 1.0:
                    break
        else:
            print("timeout: sending stop()")
            mc.stop()

        end = read_angles(mc)
        print(f"J{j} trace (s, deg): {trace}")
        print(f"end angles:   {end}")
        moved = [round(e - s, 2) for s, e in zip(start, end)]
        print(f"change per joint: {moved}")
        err = round(end[j - 1] - target, 2)
        print(f"J{j} error vs target: {err} deg")
        print(f"controller error after: {mc.get_error_information()!r}")
        mc.set_color(*(GREEN if abs(err) < 1.0 else BLUE))
        print(f"LED {'green' if abs(err) < 1.0 else 'blue'}")
    except KeyboardInterrupt:
        print("\nCtrl-C: sending stop()")
        mc.stop()
        return 130
    finally:
        mc._serial_port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
