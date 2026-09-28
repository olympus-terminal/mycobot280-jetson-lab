#!/usr/bin/env python3
"""Named arm poses (JETSON, mycobot venv) for the camera "photo pose" (2026-09-27, user: several blocks sit behind the
parked arm; "you'd have to lift it vertical").

  photo_up : from REST (unpark) or home -> near home -> straight up [J1_home, 0, 0, 0, 0, J6_home] (low load, smallest
             silhouette in the RealSense view). Holds there.
  home     : -> near home (H_near). Holds there (a cycle can start from here).
  park     : -> near home -> REST, released.

Usage: ~/venvs/mycobot/bin/python scripts/arm_pose.py {photo_up|home|park} [--log /tmp/x.csv]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402  (sets the picker joint windows)
import kinematics as K  # noqa: E402
from arm_local import Arm  # noqa: E402

F.PICKER = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pose", choices=["photo_up", "home", "park"])
    ap.add_argument("--log", default="/tmp/arm_pose_joints.csv")
    a = ap.parse_args()
    arm = Arm(a.log)
    try:
        q = arm.angles()
        low = K.fingertip(q)[0][2] < 50
        if a.pose == "photo_up":
            if low:
                arm.unpark()                         # ends at H_near
            up = np.array([arm.H_near[0], 0.0, 0.0, 0.0, 0.0, arm.H_near[5]])
            arm.move(up, 10, "photo_up")
        elif a.pose == "home":
            if low:
                arm.unpark()
            else:
                arm.move(arm.H_near, 10, "photo_home")
        else:
            if not low:
                arm.move(arm.H_near, 10, "photo_home")
                arm.park_rest_from_home()
        print("POSE", a.pose, np.round(arm.angles(), 1).tolist(), flush=True)
        return 0
    finally:
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
