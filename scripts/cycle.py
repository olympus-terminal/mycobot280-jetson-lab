#!/usr/bin/env python3
"""One pyramid cycle on the FAR side (arm at far home): pick the block near a camera estimate, place it in a slot.

  1. probe once (closed gripper) at the estimate: 'contact' at z 28-50 => a block is under the hand
  2. open, encoder-feedback descent at the estimate to z 40, then z HAND_FLOOR+15; close; value > 10 = holding
     (if empty: probe +/-12 mm radially and tangentially, re-center on the contacts, retry once)
  3. lift to z 80, precise move above the slot, descend to the place height (level), open, lift, far home
  4. log: block estimate, grasp point, final placed position (measured fingertip at release) -> calibration points
Usage: python scripts/cycle.py EST_X EST_Y SLOT_R SLOT_THETA LEVEL
"""
import datetime
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402
import kinematics as K  # noqa: E402
import precise as P  # noqa: E402

ON_BLOCK = (28.0, 50.0)
GRASP_Z = K.HAND_FLOOR + 15


def xy_polar(r, th):
    return np.array([r * np.cos(np.radians(th)), r * np.sin(np.radians(th))])


def try_grasp(xy):
    print("  open ->", P.gripper(0))
    P.goto((xy[0], xy[1], 60), tol=2.5)
    P.goto((xy[0], xy[1], 40), tol=2.5)
    P.goto((xy[0], xy[1], GRASP_Z), tol=2.5, speed=5)
    v = P.gripper(1)
    print(f"  grasp at {np.round(xy, 1).tolist()}: gripper {v} ({'HOLDING' if v and v > 10 else 'empty'})")
    return v


def main():
    ex, ey, sr, sth, level = [float(a) for a in sys.argv[1:6]]
    level = int(level)
    log = {"start": datetime.datetime.now().isoformat(timespec="seconds"), "estimate": [ex, ey], "slot": [sr, sth, level]}
    est = np.array([ex, ey])
    u = est / np.linalg.norm(est)
    v = np.array([-u[1], u[0]])
    print("close ->", P.gripper(1))
    kind, z, _ = P.probe(ex, ey, verbose=False)
    on = kind == "contact" and ON_BLOCK[0] <= z <= ON_BLOCK[1]
    log["probe"] = {"result": kind, "z": round(float(z), 1), "on_block": bool(on)}
    print(f"probe at estimate: {kind} z {z:.1f} -> {'block' if on else 'NO block'}")
    grasp_xy = est
    val = try_grasp(grasp_xy) if on else None
    if not (val and val > 10):
        pts = [est] if on else []
        for d in (u, -u, v, -v):
            p = est + 12 * d
            print("close ->", P.gripper(1))
            k2, z2, _ = P.probe(p[0], p[1], verbose=False)
            hit = k2 == "contact" and ON_BLOCK[0] <= z2 <= ON_BLOCK[1]
            print(f"  probe {np.round(p, 1).tolist()}: {k2} z {z2:.1f} -> {'block' if hit else '-'}")
            if hit:
                pts.append(p)
        if not pts:
            P.goto((ex, ey, 80), tol=4)
            log["result"] = "block not found"
            json.dump(log, open(f"data/perception/cycle_{datetime.datetime.now():%Y%m%d_%H%M%S}.json", "w"), indent=1)
            print("RESULT: block not found")
            return 2
        grasp_xy = np.mean(pts, 0)
        val = try_grasp(grasp_xy)
    log["grasp_xy"] = np.round(grasp_xy, 1).tolist()
    log["grip_value"] = val
    if not (val and val > 10):
        P.goto((grasp_xy[0], grasp_xy[1], 80), tol=4)
        log["result"] = "grasp failed"
        json.dump(log, open(f"data/perception/cycle_{datetime.datetime.now():%Y%m%d_%H%M%S}.json", "w"), indent=1)
        print("RESULT: grasp failed")
        return 3
    P.goto((grasp_xy[0], grasp_xy[1], 80), tol=4)
    slot = xy_polar(sr, sth)
    zp = F.zgrasp(level, place=True)
    zh = max(80.0, zp + 40)
    P.goto((slot[0], slot[1], zh), tol=2.5)
    P.goto((slot[0], slot[1], zp + 12), tol=2.5, speed=5)
    tip, _, q = P.goto((slot[0], slot[1], zp), tol=2.0, speed=4)
    print("  release ->", P.gripper(0))
    log["placed_tip"] = np.round(tip, 1).tolist()
    log["placed_block_center_est"] = [round(float(tip[0]), 1), round(float(tip[1]), 1), round(float(K.BLOCK * level + 15), 1)]
    P.goto((slot[0], slot[1], zh), tol=4)
    P.move_to(F.pose(180, F.FAR_THETA, 80), 8, "home_far")
    log["result"] = "placed"
    json.dump(log, open(f"data/perception/cycle_{datetime.datetime.now():%Y%m%d_%H%M%S}.json", "w"), indent=1)
    print(f"RESULT: placed in slot (r {sr}, th {sth}, level {level}); measured tip at release {np.round(tip, 1).tolist()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
