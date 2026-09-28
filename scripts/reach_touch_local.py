#!/usr/bin/env python3
"""Touch test of the FAR-REACH IK branch (far_side.REACH_AXIS) on EMPTY table spots, run ON THE JETSON.

For each (r, theta): hover at z 80 on the reach branch, then a closed-gripper probe down in 5 mm steps to z_end
(default -10) until the fingertip stops descending -> the fingertip-point z where the hand meets the table on this
branch (working-tilt value: HAND_FLOOR = 5). Then lift and park at REST. Prints "SUMMARY {...}".

Usage (Jetson): ~/venvs/mycobot/bin/python scripts/reach_touch_local.py --spots 270 55 300 55 --log FILE.csv
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat  # noqa: E402

START_MAX_C = 45


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spots", nargs="+", type=float, required=True, help="r1 th1 r2 th2 ...")
    ap.add_argument("--z-end", type=float, default=-35.0, help="target-space; the command adds the ~+20 mm sag offset")
    ap.add_argument("--log")
    a = ap.parse_args()
    spots = list(zip(a.spots[0::2], a.spots[1::2]))
    S = {"spots": spots, "touches": []}
    arm = Arm(a.log)
    try:
        t = arm.temps()
        S["temps_start"] = None if t is None else t.tolist()
        if t is not None and t.max() > START_MAX_C:
            S["result"] = f"too warm to start {t.tolist()}"
            return 5
        q = arm.angles()
        if K.fingertip(q)[0][2] < 50:
            arm.unpark()
        arm.gripper(1)   # closed: a probe with the fingers together
        for r, th in spots:
            x, y = r * np.cos(np.radians(th)), r * np.sin(np.radians(th))
            arm.goto((x, y, 80), tol=3.0, label=f"reach_hover_{r:.0f}", reach=True)
            res, z = arm.probe(x, y, z_start=50.0, z_end=a.z_end, step=5.0, reach=True)
            q = arm.angles()
            tip = K.fingertip(q)[0]
            S["touches"].append({"r": r, "th": th, "result": res, "z": round(float(z), 1), "q": q.round(2).tolist(),
                                 "tip_xy": np.round(tip[:2], 1).tolist()})
            print(f"touch r {r:.0f} th {th:.0f}: {res} at fingertip z {z:.1f}", flush=True)
            arm.goto((x, y, 80), tol=4.0, label="reach_up", reach=True, iters=2)
        arm.move(arm.H_near, 10, "home")
        arm.park_rest_from_home()
        S["result"] = "ok"
        return 0
    except Overheat as e:
        S["result"] = f"overheat: {e}"
        arm.safe_exit("overheat")
        return 4
    except Exception as e:
        S["result"] = f"error: {type(e).__name__}: {e}"
        try:
            arm.safe_exit("error")
        except Exception as e2:
            S["safe_exit_error"] = f"{type(e2).__name__}: {e2}"
        return 1
    finally:
        t = arm.temps()
        S["temps_end"] = None if t is None else t.tolist()
        print("SUMMARY " + json.dumps(S), flush=True)
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
