#!/usr/bin/python3
"""Wrist-camera scan ON THE JETSON (system python3 + the mycobot venv's site-packages: cv2 + pymycobot in one process).

At a raised look-down pose (fingertip r, z configurable; hand tilted 55 deg, the most J5 allows), rotate J1 through the
given range in steps; at each stop wait for the arm to settle, grab N wrist frames, save them with the measured joint
angles. Then park at REST. Output dir: --out (images + scan.json). Detection/mapping happens on the laptop.

Usage (Jetson): /usr/bin/python3 scripts/wrist_scan_local.py --out /tmp/wscan --j1 80 -122 --step 15 [--log FILE]
"""
import argparse
import glob
import json
import os
import sys
import time

sys.path += glob.glob("/home/nvidia/venvs/mycobot/lib/python3*/site-packages")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat, S_NEAR  # noqa: E402


def grab(cap, n):
    for _ in range(4):  # flush stale buffered frames
        cap.grab()
    frames = []
    for _ in range(n):
        ok, f = cap.read()
        if ok:
            frames.append(f)
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--j1", nargs=2, type=float, default=[80, -122])
    ap.add_argument("--step", type=float, default=15)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--pose", nargs=6, type=float, default=list(S_NEAR), help="look pose (J1 is replaced per stop)")
    ap.add_argument("--log")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    arm = Arm(a.log)
    recs, result = [], "ok"
    try:
        q = arm.angles()
        if K.fingertip(q)[0][2] < 50:
            arm.unpark()
        look = np.array(a.pose, float)
        look[0] = arm.angles()[0]
        arm.move(look, 10, "look_raise")
        j1s = np.arange(a.j1[0], a.j1[1] - 1e-6 if a.j1[1] < a.j1[0] else a.j1[1] + 1e-6,
                        -a.step if a.j1[1] < a.j1[0] else a.step)
        for j1 in j1s:
            look[0] = j1
            q = arm.move(look, 12, f"scan_j1_{j1:+.0f}")
            time.sleep(0.4)
            q = arm.angles()
            for k, f in enumerate(grab(cap, a.frames)):
                fn = f"j1_{j1:+04.0f}_{k}.jpg"
                cv2.imwrite(os.path.join(a.out, fn), f)
                recs.append({"file": fn, "j1_cmd": float(j1), "angles": q.round(2).tolist(), "t": time.time()})
            print(f"scan J1 {j1:+.0f}: saved {a.frames} frames", flush=True)
        arm.park_near() if arm.angles()[0] < 0 else (arm.move(arm.H_near, 10, "home"), arm.park_rest_from_home())
    except Overheat as e:
        result = f"overheat {e}"
        arm.safe_exit("overheat")
    except Exception as e:
        result = f"error {type(e).__name__}: {e}"
        try:
            arm.safe_exit("error")
        except Exception as e2:
            result += f"; safe_exit error {e2}"
    finally:
        json.dump({"result": result, "pose": a.pose, "records": recs}, open(os.path.join(a.out, "scan.json"), "w"), indent=1)
        t = arm.temps()
        print("SUMMARY " + json.dumps({"result": result, "n_images": len(recs), "temps_end": None if t is None else t.tolist()}), flush=True)
        cap.release()
        arm.close()
    return 0 if result == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
