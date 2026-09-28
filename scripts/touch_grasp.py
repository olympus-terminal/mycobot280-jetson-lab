#!/usr/bin/env python3
"""Find a block by touch (closed-gripper probes) around a camera estimate, then grasp it at the found center.

Probe results: 'contact' at measured z ~ 30-45 mm = block top under the hand; 'free' down to 25 mm = no block.
Search: estimate, then a ring of 8 points at 15 mm. Edges: from the first contact point, step outward along the radial
(u) and tangential (v) directions in 8 mm steps until 'free'; center = midpoint of the outermost contacts per axis.
Grasp: open, encoder-feedback descent to z 40 then z 20 (HAND_FLOOR + 15), close, check the gripper value (>10 = holding),
lift to z 80.

Usage (arm at FAR home, cool servos):  python scripts/touch_grasp.py X Y
Logs every probe to data/perception/touch_<stamp>.json
"""
import datetime
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import precise as P  # noqa: E402

BLOCK_Z = (28.0, 50.0)  # measured probe z range that counts as "on a block top"


def main():
    x0, y0 = float(sys.argv[1]), float(sys.argv[2])
    log = {"start": datetime.datetime.now().isoformat(timespec="seconds"), "estimate": [x0, y0], "probes": []}
    u = np.array([x0, y0]) / np.hypot(x0, y0)       # radial
    v = np.array([-u[1], u[0]])                      # tangential

    def pr(p):
        kind, z, xy = P.probe(p[0], p[1])
        on = kind == "contact" and BLOCK_Z[0] <= z <= BLOCK_Z[1]
        log["probes"].append({"xy": [round(float(p[0]), 1), round(float(p[1]), 1)], "result": kind, "z": round(float(z), 1), "on_block": bool(on)})
        print(f"  probe {np.round(p, 1).tolist()}: {kind} at z {z:.1f} -> {'BLOCK' if on else 'no block'}", flush=True)
        return on

    print("gripper close ->", P.gripper(1))
    p0 = np.array([x0, y0])
    hit = p0 if pr(p0) else None
    if hit is None:
        for ang in range(0, 360, 45):
            p = p0 + 15 * (np.cos(np.radians(ang)) * u + np.sin(np.radians(ang)) * v)
            if pr(p):
                hit = p
                break
    if hit is None:
        print("no block found within 15 mm")
        json.dump(log, open(f"data/perception/touch_{datetime.datetime.now():%Y%m%d_%H%M%S}.json", "w"), indent=1)
        return 1
    center = hit.copy()
    for axis in (u, v):
        ext = []
        for sgn in (1, -1):
            last = 0.0
            for d in (8, 16, 24):
                if pr(center + sgn * d * axis):
                    last = d
                else:
                    break
            ext.append(sgn * (last + 4))   # edge lies between the last contact and the first free probe
        center = center + (ext[0] + ext[1]) / 2 * axis
        print(f"  axis {np.round(axis, 2).tolist()}: extents {ext} -> center {np.round(center, 1).tolist()}")
    log["center"] = np.round(center, 1).tolist()
    print("gripper open ->", P.gripper(0))
    P.goto((center[0], center[1], 60), tol=2.0)
    P.goto((center[0], center[1], 40), tol=2.0)
    P.goto((center[0], center[1], 20), tol=2.5, speed=5)
    val = P.gripper(1)
    log["grip_value"] = val
    print("gripper close ->", val, "(holding)" if val and val > 10 else "(EMPTY)")
    P.goto((center[0], center[1], 80), tol=4.0)
    json.dump(log, open(f"data/perception/touch_{datetime.datetime.now():%Y%m%d_%H%M%S}.json", "w"), indent=1)
    return 0 if val and val > 10 else 2


if __name__ == "__main__":
    sys.exit(main())
