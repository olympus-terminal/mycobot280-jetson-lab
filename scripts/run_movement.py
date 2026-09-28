#!/usr/bin/env python3
"""Execute a movement file (movements/*.json) on the arm, step by step, with checks. Logs what happened.

SUPERVISED USE ONLY: a person at the arm with a hand near its power plug.

Step types:  {"angles": [6 deg], "speed": n}  |  {"gripper": "open"|"close"}  |  {"wait": seconds}
             {"release": true}  (servos off; only as the LAST step, right after a step labelled "rest")
Start check: if the file has "start_pose" (6 deg), the arm must be within START_TOL deg of it on every joint.
Safety: speed <= 30; every angle within firmware limits AND J5 >= 48 (gripper self-collision, see NOTES);
after each angles step, if any joint ends > --tol from its target (contact/stall), the sequence STOPS.

--log FILE.csv records every angle reading with host time (time.time(), s): columns
    t_host, event, step, j1..j6   (event: start/poll/step_begin/step_end/gripper/stop/end)

Usage (on the Jetson):
    ~/venvs/mycobot/bin/python scripts/run_movement.py movements/pick_test_v1.json --delay 15
    ~/venvs/mycobot/bin/python scripts/run_movement.py movements/pick_test_v1.json --log /tmp/joints.csv
"""
import argparse
import csv
import datetime
import hashlib
import json
import sys
import time

import pymycobot
from pymycobot import MyCobot280

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = "1000000"
J5_MIN = 48.0
TEMP_START_MAX = 55  # deg C: refuse to start above this (servo rating unknown; J5 reached 59 holding a bent wrist)
TEMP_ABORT = 62      # deg C: stop the sequence above this
START_TOL = 8.0      # deg: max distance from the movement's start_pose
# Per-joint arrival tolerance multiplier on --tol. J5 sags under the hand+camera load near horizontal
# (1.1 deg short at 90, 3.7 at 105, 5.1 at 120; no contact observed, 2026-09-25), so it gets 2x.
TOL_SCALE = [1, 1, 1, 1, 2, 1]


LOG = None  # csv.writer when --log is given
STEP = ""


def log(event, a=None):
    if LOG is not None:
        LOG.writerow([f"{time.time():.4f}", event, STEP] + (list(a) if a else [""] * 6))


def max_temp(mc, n=3):
    """Max over joints of the per-joint MINIMUM of n reads (a single corrupted reading can't trip the guard)."""
    reads = [t for t in (mc.get_servo_temps() for _ in range(n)) if isinstance(t, list) and len(t) == 6]
    if not reads:
        return None, None
    t = [min(r[k] for r in reads) for k in range(6)]
    return max(t), t


def angles(mc):
    for _ in range(5):
        a = mc.get_angles()
        if isinstance(a, list) and len(a) == 6:
            log("read", a)
            return a
        time.sleep(0.1)
    raise RuntimeError("no angle reading")


def settle(mc, timeout, target=None, tol=3.0):
    """Poll (and log) until the arm has settled: near `target` (within tol) and still for 0.5 s, or still for
    1.5 s after at least 1 s (e.g. short of target: contact/stall). Commands are sent async, so polling covers
    the whole motion."""
    t0 = last_t = time.time()
    last = None
    while time.time() - t0 < timeout:
        time.sleep(0.05)
        a = mc.get_angles()
        if not (isinstance(a, list) and len(a) == 6):
            continue
        log("poll", a)
        now = time.time()
        if last is None or max(abs(x - y) for x, y in zip(a, last)) > 0.2:
            last, last_t = a, now
            continue
        near = target is not None and max(abs(x - y) for x, y in zip(a, target)) <= tol
        if (near and now - last_t >= 0.5) or (now - t0 >= 1.0 and now - last_t >= 1.5):
            break
    return angles(mc)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("movement")
    ap.add_argument("--delay", type=float, default=0)
    ap.add_argument("--tol", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--log", help="CSV file for timestamped joint readings")
    args = ap.parse_args()
    global LOG, STEP
    logf = None
    if args.log:
        logf = open(args.log, "w", newline="")
        LOG = csv.writer(logf)
        LOG.writerow(["t_host", "event", "step", "j1", "j2", "j3", "j4", "j5", "j6"])

    raw = open(args.movement, "rb").read()
    mv = json.loads(raw)
    print(f"run_movement.py  {datetime.datetime.now().isoformat(timespec='seconds')}  pymycobot {pymycobot.__version__}")
    print(f"movement {mv['name']}  sha256 {hashlib.sha256(raw).hexdigest()[:16]}  steps {len(mv['steps'])}")

    mc = MyCobot280(args.port, BAUD)
    try:
        lims = [(mc.get_joint_min_angle(j), mc.get_joint_max_angle(j)) for j in range(1, 7)]
        for s in mv["steps"]:
            if "angles" in s:
                if not 1 <= s["speed"] <= 30:
                    sys.exit(f"refusing: step {s['label']} speed {s['speed']}")
                for j, (v, (lo, hi)) in enumerate(zip(s["angles"], lims), 1):
                    if not lo <= v <= hi:
                        sys.exit(f"refusing: step {s['label']} J{j}={v} outside {lo}..{hi}")
                if s["angles"][4] < J5_MIN:
                    sys.exit(f"refusing: step {s['label']} J5={s['angles'][4]} < {J5_MIN}")
        steps = mv["steps"]
        for k, s in enumerate(steps):
            if "release" in s and (k != len(steps) - 1 or k == 0 or steps[k - 1].get("label") != "rest"):
                sys.exit("refusing: 'release' must be the last step and follow the 'rest' step")
        start_now = angles(mc)
        print(f"start angles {start_now}")
        if "start_pose" in mv:
            dev = max(abs(a - b) for a, b in zip(start_now, mv["start_pose"]))
            if dev > START_TOL:
                sys.exit(f"refusing: arm is {dev:.1f} deg from this movement's start_pose {mv['start_pose']} "
                         f"(max {START_TOL}); run the right movement first (e.g. unpark)")
        tmax, temps = max_temp(mc)
        print(f"start temps {temps}")
        moves = any("angles" in s or "gripper" in s for s in mv["steps"])
        if moves and not mv.get("allow_hot") and tmax is not None and tmax > TEMP_START_MAX:
            sys.exit(f"refusing: servo temp {tmax} C > {TEMP_START_MAX} C; let it cool")
        if args.delay:
            print(f"waiting {args.delay:.0f} s")
            time.sleep(args.delay)
        mc.clear_error_information()
        for s in mv["steps"]:
            t0 = time.time()
            STEP = s.get("label", "")
            log("step_begin")
            if "angles" in s:
                mc.send_angles(s["angles"], s["speed"], _async=True)  # write once, no blocking re-sends
                log("sent")
                a = settle(mc, args.timeout, s["angles"], args.tol)
                errs = [round(x - y, 2) for x, y in zip(a, s["angles"])]
                worst = max(abs(e) / k for e, k in zip(errs, TOL_SCALE))  # in units of --tol
                print(f"[{s['label']}] reached {a}  err {errs}  ({time.time() - t0:.1f} s)  coords {mc.get_coords()}")
                if worst > args.tol:
                    print(f"STOP: joint error {worst} > tol {args.tol} (contact/stall?). Holding current angles.")
                    mc.send_angles(a, 5, _async=True)
                    break
            elif "gripper" in s:
                mc.set_gripper_state(0 if s["gripper"] == "open" else 1, 30)
                time.sleep(3)
                print(f"[{s['label']}] gripper {s['gripper']} -> value {mc.get_gripper_value()}")
            elif "release" in s:
                print(f"[{s.get('label', 'release')}] release_all_servos -> {mc.release_all_servos()!r}")
                log("release")
                break
            elif "wait" in s:
                while time.time() - t0 < s["wait"]:
                    a = mc.get_angles()
                    if isinstance(a, list) and len(a) == 6:
                        log("poll", a)
                    time.sleep(0.1)
            log("step_end")
            tmax, temps = max_temp(mc)
            if tmax is not None and tmax > TEMP_ABORT:
                print(f"STOP: servo temp {tmax} C > {TEMP_ABORT} C {temps}. Holding current angles.")
                mc.send_angles(angles(mc), 5)
                break
        print(f"end angles {angles(mc)}  error {mc.get_error_information()}  temps {mc.get_servo_temps()}")
    except KeyboardInterrupt:
        print("\nCtrl-C: stop()")
        mc.stop()
        return 130
    finally:
        mc._serial_port.close()
        if logf:
            logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
