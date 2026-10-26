#!/usr/bin/env python3
"""Read-only probe of the myCobot 280 Arduino: firmware, power, servo state, joint angles.

Sends NO motion, power or configuration commands, only get_*/is_* queries.
Note: opening the serial port resets the Arduino base board (DTR), as with any Arduino.

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/probe_arm.py [--port PORT] [--baud 115200|1000000] [--check]
With no --baud, tries 1000000 (this arm, verified 2026-09-24) then 115200 and stops at the first that answers.
--check adds a go/no-go list (baud, servo power, temperatures, joints inside the firmware limits, distance to REST);
exit 0 = go, 2 = no-go, 1 = no answer. Run it before the first motion of every session (docs/SETUP_PROTOCOL.md).
"""
import argparse
import datetime
import json
import os
import platform
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
POSES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "poses.json")
START_ALL_C = 52   # demo_loop.py START_ALL: no move starts with any servo above this
REST_TOL = 8.0     # deg, run_movement.py START_TOL

# Read-only queries, in order. Name -> call.
QUERIES = [
    ("is_controller_connected", lambda mc: mc.is_controller_connected()),
    ("get_system_version", lambda mc: mc.get_system_version()),
    ("get_basic_version", lambda mc: mc.get_basic_version()),
    ("get_modify_version", lambda mc: mc.get_modify_version()),
    ("get_transponder_mode", lambda mc: mc.get_transponder_mode()),
    ("is_power_on", lambda mc: mc.is_power_on()),
    ("is_all_servo_enable", lambda mc: mc.is_all_servo_enable()),
    ("get_servo_status", lambda mc: mc.get_servo_status()),
    ("get_servo_voltages", lambda mc: mc.get_servo_voltages()),
    ("get_servo_temps", lambda mc: mc.get_servo_temps()),
    ("get_error_information", lambda mc: mc.get_error_information()),
    ("get_angles", lambda mc: mc.get_angles()),
    ("get_coords", lambda mc: mc.get_coords()),
    ("joint_min_angles", lambda mc: [mc.get_joint_min_angle(j) for j in range(1, 7)]),
    ("joint_max_angles", lambda mc: [mc.get_joint_max_angle(j) for j in range(1, 7)]),
]


def answered(value):
    """pymycobot returns None / -1 / [] when the controller doesn't reply."""
    return value not in (None, -1, [], "")


def probe(port, baud):
    print(f"\n--- port={port} baud={baud}")
    mc = MyCobot280(port, str(baud))
    results = {}
    try:
        for name, call in QUERIES:
            try:
                value = call(mc)
            except Exception as exc:  # report, don't hide
                value = f"ERROR {type(exc).__name__}: {exc}"
            results[name] = value
            print(f"{name:24s} {value!r}")
            time.sleep(0.05)
    finally:
        mc._serial_port.close()
    return results


def go_no_go(results, baud):
    """--check: one PASS/FAIL line per thing that blocked or endangered first motion on our arm (LAB_NOTEBOOK day 0-1)."""
    fails = 0

    def line(ok, what, hint):
        nonlocal fails
        fails += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {what}" + ("" if ok else f"\n      -> {hint}"))

    print("\n--- go/no-go")
    line(baud == 1000000, f"baud {baud}",
         "the motion scripts open the port at 1000000 (what our arm answers at). If yours answers only at 115200, do not "
         "move it with these scripts yet: see docs/SETUP_PROTOCOL.md, troubleshooting")
    line(results.get("is_power_on") == 1, f"servo power on (is_power_on = {results.get('is_power_on')!r})",
         "is the 12 V adapter plugged in and switched on?")
    temps = results.get("get_servo_temps")
    ok = isinstance(temps, list) and len(temps) == 6 and max(temps) <= START_ALL_C
    line(ok, f"servo temperatures {temps!r} <= {START_ALL_C} C",
         "let the arm cool while parked, or rerun (single reads glitch high; the motion code uses the minimum of 3 reads)")
    q, lo, hi = results.get("get_angles"), results.get("joint_min_angles"), results.get("joint_max_angles")
    if not all(isinstance(v, list) and len(v) == 6 for v in (q, lo, hi)):
        line(False, f"angles and joint limits readable ({q!r}, {lo!r}, {hi!r})", "rerun; if it persists, check the cable and power")
    else:
        out = [f"J{j + 1} {q[j]:.1f} outside [{lo[j]}, {hi[j]}]" for j in range(6) if not lo[j] <= q[j] <= hi[j]]
        line(not out, "all joints inside the firmware limits" + (": " + "; ".join(out) if out else ""),
             "the controller refuses ALL motion while any joint is out of range (even back toward the range): unplug the "
             "12 V, move that joint by hand toward the middle of its range, plug in again, rerun")
        try:
            with open(POSES) as f:
                rest = json.load(f)["REST"]["angles"]
            dmax = max(abs(a - b) for a, b in zip(q, rest))
            print(f"INFO  largest joint difference to REST in config/poses.json: {dmax:.1f} deg "
                  + ("(the arm is at REST)" if dmax <= REST_TOL else
                     "(if the arm is lying in its parked pose now, record that pose as REST: SETUP_PROTOCOL.md step 6)"))
        except (OSError, KeyError, ValueError) as exc:
            print(f"INFO  REST not compared ({type(exc).__name__}: {exc})")
    print(f"\nCHECK: {'GO' if not fails else f'NO-GO, {fails} failed'}")
    return 0 if not fails else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, choices=[115200, 1000000])
    ap.add_argument("--check", action="store_true", help="add the go/no-go list (exit 2 on no-go)")
    args = ap.parse_args()

    print(f"probe_arm.py  {datetime.datetime.now().isoformat(timespec='seconds')}")
    print(f"python {platform.python_version()}  pymycobot {pymycobot.__version__}  host {platform.node()}")

    if not os.path.exists(args.port):
        print(f"\nRESULT: {args.port} does not exist. Is the arm's USB cable in? `ls /dev/serial/by-id/` and "
              "`lsmod | grep ch341`; on a Jetson the CH340 driver must be built: sudo bash scripts/build_ch341.sh")
        return 1
    if not os.access(args.port, os.R_OK | os.W_OK):
        print(f"\nRESULT: no read/write permission on {args.port}: sudo usermod -aG dialout $USER, then log out and in")
        return 1

    for baud in [args.baud] if args.baud else [1000000, 115200]:
        results = probe(args.port, baud)
        if answered(results.get("get_angles")) or answered(results.get("get_system_version")):
            print(f"\nRESULT: controller answered at {baud} baud")
            return go_no_go(results, baud) if args.check else 0
    print("\nRESULT: no answer at any baud rate. Check the 12 V power, then the Transponder firmware on the base board")
    return 1


if __name__ == "__main__":
    sys.exit(main())
