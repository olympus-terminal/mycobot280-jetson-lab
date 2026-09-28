#!/usr/bin/env python3
"""Calibration tour ON THE JETSON while HOLDING a tagged block: near side, lateral, far side (raised transitions between
sides), 2 s holds at each pose (labels 'hold_*' for calibrate_cam_arm.py), then put the block down at --put X Y and park.
Usage: ~/venvs/mycobot/bin/python scripts/tour_local.py --put X Y --log FILE.csv
       [--handoff 15]: start from REST: unpark, near home, open the gripper, wait N s for a person to put a tagged block
       (tag up) between the fingers, close, require gripper value > 10 (else park), then the tour.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402
import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat, S_NEAR, pose_xyz  # noqa: E402

SIDES = {  # (r, theta, z) holds; the tag must face the camera (az ~127): near + lateral first, then far
    "near": [(150, 60, 80), (180, 75, 80), (210, 60, 120), (180, 100, 120), (150, 90, 120), (210, 85, 80)],
    "lateral": [(180, 40, 80), (210, 25, 120), (150, 15, 80), (180, 0, 120)],
    "far": [(180, -60, 80), (210, -75, 120), (150, -90, 80), (180, -100, 120)],
    # 2026-09-25 new high camera (az ~128): the held tag is only visible for theta ~40-118. Spread in r AND z (incl. low
    # holds near the table) so a pixel-reprojection fit pins the camera's range, not just its viewing directions.
    # 2026-09-25 13:20: holds with a clear camera line of sight past the arm links (capsule check vs the 11:57 calibration).
    # Nothing past theta -32 is visible with a held block -> the far side is covered by --placelook instead.
    "vis": [(150, 118, 60), (190, 118, 120), (230, 103, 60), (150, 88, 120), (190, 88, 60), (230, 73, 60),
            (150, 58, 120), (190, 58, 60), (230, 43, 60), (150, 28, 60), (190, 28, 120), (230, 13, 60),
            (150, -2, 120), (190, -2, 60), (230, -17, 60), (190, -32, 60)],
    "dense": [(140, 45, 40), (200, 45, 100), (235, 50, 40), (170, 55, 160),
              (140, 70, 100), (200, 70, 40), (220, 75, 80), (170, 80, 160),
              (140, 95, 40), (185, 95, 140), (235, 95, 40), (170, 100, 100),
              (140, 115, 160), (200, 115, 40), (215, 115, 80), (170, 118, 40)],
}


def hold(arm, r, th, z, label):
    q = arm.move(F.pose(r, th, z), 10, label)
    arm._w("hold_begin", f"hold_{label}", q)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        a = arm.mc.get_angles()
        if isinstance(a, list) and len(a) == 6:
            arm._w("poll", f"hold_{label}", a)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--put", nargs=2, type=float, default=[0.0, 180.0])
    ap.add_argument("--log")
    ap.add_argument("--sides", nargs="+", default=["near", "lateral", "far"])
    ap.add_argument("--handoff", type=float, default=0.0, help="seconds to wait for a hand-placed block")
    ap.add_argument("--placelook", nargs="*", type=float, default=[],
                    help="X1 Y1 X2 Y2 ...: after the holds, put the block down at each far point, look from a low lateral pose "
                         "(labels put_i / look_i), re-grasp; the block is left at the last point")
    ap.add_argument("--grasp-z", type=float, default=13.0)
    a = ap.parse_args()
    arm = Arm(a.log)
    S = {"start": time.strftime("%Y-%m-%dT%H:%M:%S"), "holds": []}
    try:
        if a.handoff > 0:
            if K.fingertip(arm.angles())[0][2] < 50:
                arm.unpark()
            arm.move(arm.H_near, 10, "handoff_home")
            arm.gripper(0)
            open("/tmp/handoff_open", "w").write(f"{time.time():.2f} {a.handoff:.0f}\n")   # instant flag for the laptop's voice cue
            print(f"HANDOFF: put the block between the fingers now ({a.handoff:.0f} s)", flush=True)
            time.sleep(a.handoff)
            v = arm.gripper(1)
            os.remove("/tmp/handoff_open")
            S["handoff_value"] = v
            if not (v and 12 < v < 70):
                S["result"] = f"handoff: gripper empty (value {v})"
                arm.gripper(0)
                arm.park_rest_from_home()
                return 3
        for side in a.sides:
            if side == "far":
                arm.move(S_NEAR, 10, "raise")
                s_far = S_NEAR.copy()
                s_far[0] = -80.0
                arm.move(s_far, 12, "rotate_far")
            for (r, th, z) in SIDES[side]:
                lab = f"{side}_r{r}_th{th}_z{z}"
                hold(arm, r, th, z, lab)
                S["holds"].append(lab)
                print("held", lab, flush=True)
            if side == "far":
                s_far = S_NEAR.copy()
                s_far[0] = arm.angles()[0]
                arm.move(s_far, 10, "raise_back")
                arm.move(S_NEAR, 12, "rotate_near")
        pl = list(zip(a.placelook[0::2], a.placelook[1::2]))
        if pl:
            look = F.pose(200, 20, 60)          # hand low on the lateral side: out of the camera's line to the far side
            S["placelook"] = []
            for i, (x, y) in enumerate(pl):
                arm.goto((x, y, 80), tol=3, label=f"pl_hover_{i}")
                arm.goto((x, y, a.grasp_z + 20), tol=3, speed=5, label=f"pl_near_{i}")
                arm.goto((x, y, a.grasp_z + 2), tol=3, speed=4, label=f"pl_down_{i}", tip_min=a.grasp_z - 2)
                q_put = arm.angles()
                arm._w("put", f"put_{i}", q_put)
                arm.gripper(0)
                arm.goto((x, y, 80), tol=4, label=f"pl_up_{i}", iters=2)
                arm.move(look, 10, f"pl_look_{i}")
                arm._w("hold_begin", f"look_{i}", arm.angles())
                time.sleep(2.5)
                arm._w("hold_end", f"look_{i}", arm.angles())
                rec = {"xy": [x, y], "q_put": q_put.round(2).tolist(), "tip_put": np.round(K.fingertip(q_put)[0], 1).tolist()}
                last = i == len(pl) - 1
                if not last:     # re-grasp where it was put (same xy, same height -> same in-hand pose)
                    arm.goto((x, y, 80), tol=3, label=f"pl_back_{i}")
                    arm.goto((x, y, a.grasp_z + 20), tol=3, speed=5, label=f"pl_rg_near_{i}")
                    arm.goto((x, y, a.grasp_z), tol=3, speed=4, label=f"pl_rg_{i}", iters=2, tip_min=a.grasp_z - 2)
                    v = arm.gripper(1)
                    rec["regrasp"] = v
                    arm.goto((x, y, 80), tol=4, label=f"pl_lift_{i}", iters=2)
                S["placelook"].append(rec)
                print(f"placelook {i} ({x:.0f}, {y:.0f}) done" + ("" if last else f", regrasp {rec['regrasp']}"), flush=True)
                if not last and not (rec["regrasp"] and 12 < rec["regrasp"] < 70):
                    print("regrasp failed: stopping place-and-look", flush=True)
                    arm.gripper(0)
                    break
            arm.move(arm.H_near, 10, "home_near")
            arm.park_rest_from_home()
            S["result"] = "tour done (placelook)"
            return 0
        arm.move(arm.H_near, 10, "home_near")
        # put the block back down where it came from (grasp height = place height on the table)
        x, y = a.put
        arm.goto((x, y, 80), tol=3, label="put_hover")
        arm.goto((x, y, K.HAND_FLOOR + 15 + 12), tol=3, speed=5, label="put_near")
        arm.goto((x, y, K.HAND_FLOOR + 15 + 2), tol=2.5, speed=4, label="put_down")
        S["release"] = arm.gripper(0)
        arm.goto((x, y, 80), tol=4, label="put_clear")
        arm.move(arm.H_near, 10, "home_near2")
        arm.park_rest_from_home()
        S["result"] = "tour done"
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
        print("SUMMARY " + json.dumps(S), flush=True)
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
