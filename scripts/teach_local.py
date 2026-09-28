#!/usr/bin/env python3
"""Kinesthetic teaching ON THE JETSON: the arm goes limp (servos released) while a person guides it; joint angles are
logged at ~10 Hz; keyframes = poses held still >= HOLD_S. At the end the servos are re-powered AT THE CURRENT POSE
(send_angles(current), no jump), then the arm returns to REST (from near home) and is released there.

The person must be HOLDING the arm when it is released (it drops otherwise) and when it re-locks.
The laptop gives the voice countdowns (the Jetson has no speaker); this script only waits --delay seconds first.

Usage (Jetson): ~/venvs/mycobot/bin/python scripts/teach_local.py --seconds 60 --delay 8 --out /tmp/teach.json --log FILE.csv
Output JSON: samples [[t, j1..j6], ...], keyframes [{t0, t1, angles, tip_xyz, tool_axis, tilt_from_vertical_deg}], gripper values.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402
from arm_local import REST, Arm  # noqa: E402

HOLD_S = 1.5       # a keyframe: still for this long
STILL_DEG = 0.8    # max joint change within a keyframe window


def keyframes(samples):
    out, i = [], 0
    t = np.array([s[0] for s in samples])
    q = np.array([s[1:] for s in samples])
    while i < len(samples):
        j = i
        while j + 1 < len(samples) and np.abs(q[j + 1] - q[i]).max() <= STILL_DEG:
            j += 1
        if t[j] - t[i] >= HOLD_S:
            qm = np.median(q[i:j + 1], 0)
            tip, ax = K.fingertip(qm)
            out.append({"t0": round(float(t[i] - t[0]), 2), "t1": round(float(t[j] - t[0]), 2), "angles": qm.round(2).tolist(),
                        "tip_xyz": tip.round(1).tolist(), "tool_axis": ax.round(3).tolist(),
                        "tilt_from_vertical_deg": round(float(np.degrees(np.arccos(-ax[2]))), 1)})
            i = j + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--delay", type=float, default=8, help="seconds before releasing (the laptop counts down)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--log")
    a = ap.parse_args()
    arm = Arm(a.log)
    samples, grip = [], []
    res = {"result": "?"}
    try:
        time.sleep(a.delay)
        arm.mc.release_all_servos()
        arm._w("release", "teach")
        t0 = time.time()
        while time.time() - t0 < a.seconds:
            q = arm.mc.get_angles()
            if isinstance(q, list) and len(q) == 6:
                samples.append([time.time()] + [float(v) for v in q])
                arm._w("teach", "", q)
            time.sleep(0.08)
        # re-lock where it is (the laptop warns the person first; the last seconds of --seconds are the warning window)
        q = arm.angles()
        arm.mc.send_angles([round(float(v), 2) for v in q], 10, _async=True)
        arm._w("send", "teach_lock", q)
        time.sleep(1.5)
        res["locked_at"] = arm.angles().round(2).tolist()
        res["result"] = "ok"
    except Exception as e:
        res["result"] = f"error {type(e).__name__}: {e}"
    finally:
        kf = keyframes(samples) if samples else []
        res.update({"n_samples": len(samples), "keyframes": kf, "samples": [[round(s[0], 3)] + [round(v, 2) for v in s[1:]] for s in samples]})
        json.dump(res, open(a.out, "w"), indent=1)
        try:
            print("SUMMARY " + json.dumps({k: v for k, v in res.items() if k != "samples"}), flush=True)
        except (BrokenPipeError, OSError):
            pass
    # back to REST: lift a little if low, near home, rest, release (normal parking)
    try:
        if res["result"] == "ok":
            q = arm.angles()
            if K.fingertip(q)[0][2] < 60:
                up = q.copy()
                up[1] = min(up[1] + 15, 90)
                arm.move(up, 6, "teach_up", check_path=False, check_heat=False)
            arm.into_windows("teach_wrist")
            arm.move(arm.H_near, 8, "teach_home", check_heat=False)
            arm.park_rest_from_home()
    except Exception as e:
        print(f"PARK ERROR {e}: servos left powered where they are", flush=True)
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
