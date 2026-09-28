#!/usr/bin/env python3
"""Read-only probe of the myCobot 280 Arduino: firmware, power, servo state, joint angles.

Sends NO motion, power or configuration commands, only get_*/is_* queries.
Note: opening the serial port resets the Arduino base board (DTR), as with any Arduino.

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/probe_arm.py [--port PORT] [--baud 115200|1000000]
With no --baud, tries 1000000 (this arm, verified 2026-09-24) then 115200 and stops at the first that answers.
"""
import argparse
import datetime
import platform
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, choices=[115200, 1000000])
    args = ap.parse_args()

    print(f"probe_arm.py  {datetime.datetime.now().isoformat(timespec='seconds')}")
    print(f"python {platform.python_version()}  pymycobot {pymycobot.__version__}  host {platform.node()}")

    for baud in [args.baud] if args.baud else [1000000, 115200]:
        results = probe(args.port, baud)
        if answered(results.get("get_angles")) or answered(results.get("get_system_version")):
            print(f"\nRESULT: controller answered at {baud} baud")
            return 0
    print("\nRESULT: no answer at any baud rate. Check the 12 V power, then the Transponder firmware on the base board")
    return 1


if __name__ == "__main__":
    sys.exit(main())
