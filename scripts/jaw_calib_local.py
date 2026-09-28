#!/usr/bin/env python3
"""Jaw calibration for the PICKER grasp, run ON THE JETSON (under record_cycle.py for RealSense video).

Why: a supervised picker grasp (2026-09-25 18:06) closed "between the two blocks" and "45 deg wrong" (user): the jaw
centre is not on the modelled fingertip point, and the J6 -> closing-axis mapping is off.

Procedure: picker hover at each --spots point (z --z); open; a person puts a block between the fingers (square to them;
the laptop plays the handoff voice line when /tmp/handoff_open appears); close; then J6 steps through --j6 values with
2 s holds (labels hold_jc_<spot>_<j6> in the joint log); put the block down at the last spot; park.
The laptop then finds the block's SIDE tags per hold -> block centre and face directions vs the flange pose
(scripts/jaw_calib_fit.py).

Usage (Jetson): ~/venvs/mycobot/bin/python scripts/jaw_calib_local.py --spots 0 200 60 190 --z 80 --log FILE.csv
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402

F.PICKER = True
import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat, pose_xyz  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spots", nargs="+", type=float, default=[0, 200, 60, 190])
    ap.add_argument("--z", type=float, default=80.0)
    ap.add_argument("--j6", nargs="+", type=float, default=[-40, -30, -20, -10, 0, 10, 20, 30, 40])
    ap.add_argument("--handoff", type=float, default=25.0)
    ap.add_argument("--put-z", type=float, default=12.0)
    ap.add_argument("--no-block", action="store_true", help="no handoff: gripper stays OPEN; the depth camera finds the fingertips")
    ap.add_argument("--log")
    a = ap.parse_args()
    spots = list(zip(a.spots[0::2], a.spots[1::2]))
    S = {"spots": spots, "holds": []}
    arm = Arm(a.log)
    try:
        if K.fingertip(arm.angles())[0][2] < 50:
            arm.unpark()
        x0, y0 = spots[0]
        arm.goto((x0, y0, a.z), tol=3.0, label="jc_hover", j6=0.0)
        arm.gripper(0)
        if a.no_block:
            S["handoff_value"] = None
        else:
            open("/tmp/handoff_open", "w").write(f"{time.time():.2f}\n")
        v = None
        if not a.no_block:
            print(f"HANDOFF ({a.handoff:.0f} s)", flush=True)
            time.sleep(a.handoff)
            v = arm.gripper(1)
            os.remove("/tmp/handoff_open")
            S["handoff_value"] = v
        if not a.no_block and not (v and 12 < v < 70):
            S["result"] = f"handoff: not holding (value {v})"
            arm.gripper(0)
            arm.move(arm.H_near, 10, "home")
            arm.park_rest_from_home()
            return 3
        for si, (x, y) in enumerate(spots):
            for j6 in a.j6:
                q = pose_xyz(np.array([x, y, a.z]))
                q[5] = j6
                q = arm.move(q, 8, f"jc_{si}_{j6:+.0f}")
                lab = f"hold_jc_{si}_{j6:+.0f}"
                arm._w("hold_begin", lab, q)
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    qq = arm.mc.get_angles()
                    if isinstance(qq, list) and len(qq) == 6:
                        arm._w("poll", lab, qq)
                    time.sleep(0.05)
                arm._w("hold_end", lab, arm.angles())
                S["holds"].append(lab)
        if a.no_block:
            arm.move(arm.H_near, 10, "home")
            arm.park_rest_from_home()
            S["result"] = "ok"
            return 0
        # put the block down at the last spot, J6 0
        x, y = spots[-1]
        arm.goto((x, y, a.put_z + 20), tol=3.0, label="jc_put_near", j6=0.0)
        arm.goto((x, y, a.put_z), tol=3.0, speed=4, label="jc_put", j6=0.0, tip_min=a.put_z - 8)
        S["release"] = arm.gripper(0)
        arm.goto((x, y, 80), tol=4.0, label="jc_clear", j6=0.0)
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
