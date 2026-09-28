#!/usr/bin/env python3
"""Closed-gripper touch probes at given table points, run ON THE JETSON (usable under record_cycle.py for video).

Crosses to the far side via the raised pose when a point is at theta < 0 (like cycle_local). At each point: hover,
probe down (arm_local.probe; contact = stall with the command below the measured tip), hold 2 s at the bottom
(label 'probe_hold_<i>' in the joint log, for frame lookup), lift. Then park. Prints "SUMMARY {...}".

Usage (Jetson): ~/venvs/mycobot/bin/python scripts/probe_local.py --pts X1 Y1 [X2 Y2 ...] [--z-end 10] --log FILE.csv
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat, S_NEAR, pose_xyz  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pts", nargs="+", type=float, required=True)
    ap.add_argument("--z-end", type=float, default=10.0)
    ap.add_argument("--picker", action="store_true", help="vertical-tool 'picker' IK branch (far_side.PICKER)")
    ap.add_argument("--log")
    a = ap.parse_args()
    if a.picker:
        import far_side as _F
        _F.PICKER = True
    pts = list(zip(a.pts[0::2], a.pts[1::2]))
    S = {"pts": pts, "probes": []}
    arm = Arm(a.log)
    try:
        t = arm.temps()
        S["temps_start"] = None if t is None else t.tolist()
        if K.fingertip(arm.angles())[0][2] < 50:
            arm.unpark()
        arm.gripper(1)
        far = False
        for i, (x, y) in enumerate(pts):
            want_far = y < 0 and np.degrees(np.arctan2(y, x)) < -20
            if want_far and not far:
                arm.move(S_NEAR, 10, "raise")
                s = S_NEAR.copy()
                s[0] = pose_xyz(np.array([x, y, 80.0]))[0]
                arm.move(s, 12, "rotate_far")
                far = True
            res, z = arm.probe(x, y, z_start=60.0, z_end=a.z_end, step=4.0)
            # probe() lifted back to z_start: go down again to the found height for a 2 s visible hold
            q = arm.goto((x, y, max(z, a.z_end) + 2.0), tol=3.0, label=f"probe_hold_{i}", iters=2, tip_min=a.z_end - 2)
            arm._w("hold_begin", f"probe_hold_{i}", arm.angles())
            time.sleep(2.0)
            arm._w("hold_end", f"probe_hold_{i}", arm.angles())
            tip = K.fingertip(arm.angles())[0]
            S["probes"].append({"xy": [x, y], "result": res, "z": round(float(z), 1), "hold_tip": np.round(tip, 1).tolist()})
            print(f"probe {i} ({x:.1f}, {y:.1f}): {res} at z {z:.1f}", flush=True)
            arm.goto((x, y, 60.0), tol=4.0, label="probe_up", iters=2)
        arm.gripper(0)
        if far:
            arm.move(arm.H_far, 10, "far_home")
            arm.park_near()
        else:
            arm.move(arm.H_near, 10, "home")
            arm.park_rest_from_home()
        S["result"] = "ok"
        return 0
    except Overheat as e:
        S["result"] = f"overheat {e}"
        arm.safe_exit("overheat")
        return 4
    except Exception as e:
        S["result"] = f"error {type(e).__name__}: {e}"
        try:
            arm.safe_exit("error")
        except Exception as e2:
            S["safe_exit_error"] = str(e2)
        return 1
    finally:
        t = arm.temps()
        S["temps_end"] = None if t is None else t.tolist()
        try:
            print("SUMMARY " + json.dumps(S), flush=True)
        except (BrokenPipeError, OSError):
            pass
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
