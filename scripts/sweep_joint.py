#!/usr/bin/env python3
"""Sweep ONE joint in small steps toward a goal, with stall (collision) detection and automatic back-off.

Used to map safe joint ranges (e.g. J5 vs the gripper/camera protrusions).

SUPERVISED USE ONLY: a person at the arm with a hand near its power plug.

Each step: send_angle(next), poll up to --step-timeout s. If the joint doesn't get within --tol of the
step target (a stall, e.g. contact), or any servo exceeds --max-temp, it commands the previous
(good) step angle and stops. Reports the last good angle = the mapped limit in that direction.

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/sweep_joint.py --joint 5 --goal 0 --step 10 --speed 10 --delay 15
"""
import argparse
import datetime
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = "1000000"


def angles(mc):
    for _ in range(5):
        a = mc.get_angles()
        if isinstance(a, list) and len(a) == 6:
            return a
        time.sleep(0.1)
    raise RuntimeError("no angle reading")


def robust_temps(mc, n=3):
    """Per-joint MINIMUM over n reads: a single corrupted reading can't trip the heat guard
    (seen 2026-09-24: J3 spiking 38..73 between normal 32 readings)."""
    reads = [t for t in (mc.get_servo_temps() for _ in range(n)) if isinstance(t, list) and len(t) == 6]
    return [min(r[k] for r in reads) for k in range(6)] if reads else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    ap.add_argument("--goal", type=float, required=True)
    ap.add_argument("--step", type=float, default=10)
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--tol", type=float, default=3.0, help="deg; farther than this from a step target = stall")
    ap.add_argument("--step-timeout", type=float, default=5.0)
    ap.add_argument("--max-temp", type=float, default=55)
    ap.add_argument("--delay", type=float, default=0)
    ap.add_argument("--port", default=DEFAULT_PORT)
    args = ap.parse_args()
    if not 1 <= args.speed <= 30 or not 0 < args.step <= 15:
        sys.exit("refusing: speed 1..30, step (0, 15]")

    j = args.joint
    print(f"sweep_joint.py  {datetime.datetime.now().isoformat(timespec='seconds')}  pymycobot {pymycobot.__version__}")
    mc = MyCobot280(args.port, BAUD)
    try:
        lo, hi = mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)
        if not lo <= args.goal <= hi:
            sys.exit(f"refusing: goal {args.goal} outside limits {lo}..{hi}")
        a0 = angles(mc)
        print(f"start angles {a0}; J{j} {a0[j - 1]} -> goal {args.goal}, step {args.step}, speed {args.speed}")
        if args.delay:
            print(f"waiting {args.delay:.0f} s")
            time.sleep(args.delay)
        mc.clear_error_information()
        good = a0[j - 1]
        direction = 1 if args.goal > good else -1
        result = "reached goal"
        while abs(args.goal - good) > args.tol:
            nxt = good + direction * min(args.step, abs(args.goal - good))
            mc.send_angle(j, round(nxt, 2), args.speed)
            t0, v = time.time(), None
            while time.time() - t0 < args.step_timeout:
                time.sleep(0.2)
                a = mc.get_angles()
                if isinstance(a, list) and len(a) == 6:
                    v = a[j - 1]
                    if abs(v - nxt) <= args.tol:
                        break
            temps = robust_temps(mc)
            hot = temps is not None and max(temps) > args.max_temp
            print(f"  step -> {nxt:.1f}: at {v}  temps {temps}")
            if v is None or abs(v - nxt) > args.tol or hot:
                result = f"STALL at step {nxt:.1f} (reading {v})" if not hot else f"HOT servo {temps}"
                mc.send_angle(j, round(good, 2), args.speed)  # back off to the last good angle
                time.sleep(2.0)
                break
            good = v
        end = angles(mc)
        print(f"result: {result}; last good J{j} = {good}")
        print(f"end angles {end}  error {mc.get_error_information()}  coords {mc.get_coords()}")
    except KeyboardInterrupt:
        print("\nCtrl-C: stop()")
        mc.stop()
        return 130
    finally:
        mc._serial_port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
