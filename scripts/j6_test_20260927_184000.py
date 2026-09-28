#!/usr/bin/env python3
"""J6 tracking test (JETSON, mycobot venv): does the gripper reach the commanded J6? (2026-09-27: grasps at J6 ≈ -27…-39
came out skewed or missed although the computed finger/block misalignment was 0.)

Hover the (open) gripper over a tag-up block at z HOVER, command J6 through a list, and at each J6 save a hand-camera
frame (capture_wrist.py must be running with --latest) and the J6 read-back. The hand camera turns with J6, so the tag's
angle in the image must change by -ΔJ6 (sign depending on the camera orientation) if J6 tracks the command.
No grasping. Ends with park (REST, released).

Usage: ~/venvs/mycobot/bin/python scripts/j6_test_20260927_184000.py X Y [--z 80] [--j6 -30 -15 0 15 30] --out DIR
Output: DIR/j6_<value>.jpg + DIR/j6_test.json (commanded J6, read-back J6 before/after the frame, tip xyz)
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402
import kinematics as K  # noqa: E402
from arm_local import Arm  # noqa: E402

F.PICKER = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--z", type=float, default=80.0)
    ap.add_argument("--j6", nargs="+", type=float, default=[-30, -15, 0, 15, 30])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    arm = Arm(os.path.join(a.out, "joints.csv"))
    rec = {"target": [a.x, a.y, a.z], "points": []}
    try:
        arm.unpark()
        for j6 in a.j6:
            tip, _ = arm.goto((a.x, a.y, a.z), tol=4.0, iters=2, label=f"j6test_{j6:+.0f}", j6=j6)
            time.sleep(1.0)
            q0 = arm.angles()
            t0 = time.time()
            path = os.path.join(a.out, f"j6_{j6:+.0f}.jpg")
            r = subprocess.run(["/usr/bin/python3", os.path.join(os.path.dirname(os.path.abspath(__file__)), "wrist_look.py"),
                                "--after", f"{t0:.3f}", "--save", path], capture_output=True, text=True, timeout=10)
            q1 = arm.angles()
            look = json.loads(r.stdout.strip().splitlines()[-1]) if r.stdout.strip() else {}
            rec["points"].append({"j6_cmd": j6, "j6_read_before": float(q0[5]), "j6_read_after": float(q1[5]),
                                  "angles": np.round(q1, 2).tolist(), "tip": np.round(tip, 1).tolist(), "look": look})
            print(f"J6 cmd {j6:+.0f} read {q1[5]:+.1f}  tags {[(t['id'], t['center']) for t in look.get('tags', [])]}", flush=True)
    finally:
        try:
            arm.move(arm.H_near, 10, "j6test_home")
            arm.park_rest_from_home()
        finally:
            json.dump(rec, open(os.path.join(a.out, "j6_test.json"), "w"), indent=1)
            arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
