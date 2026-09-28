#!/usr/bin/env python3
"""Plan a 5-block 3D pyramid (2x2 base + 1 top) with the working-tilt grasp; writes movement files.

Geometry (all in the controller frame; polar r = radius from the base axis, theta = azimuth, deg):
  - Fingers close RADIALLY (validated 2026-09-25: closing axis within 6 deg of horizontal, ~radial), so blocks must
    have faces toward the base; blocks placed by the arm come out radially aligned.
  - Heights: fingertip point z = HAND_FLOOR + 15 + 30*level (+ RELEASE_CLEAR when placing); HAND_FLOOR from the
    table-touch test.
  - Base: rings r 160 / 205 (15 mm radial gap for the fingers), theta 110 then 97 (place high theta first: the hand
    leans over the lower-theta side). Top: r 182.5, theta 103.5, level 1.
  - Supply: 5 spots at lower theta; the arm hovers there with open fingers while the user places blocks.
Every angles step is checked with gen_movements.path_ok (grasp tip minimum) before a file is written.

Usage:
  python scripts/pyramid.py supply                      -> movements/pyramid/supply_hover.json
  python scripts/pyramid.py pickplace K R THETA         -> movements/pyramid/pp_K.json (block measured at r, theta)
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_movements as G  # noqa: E402
import kinematics as K  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "movements", "pyramid")
RELEASE_CLEAR = 2.0   # mm above the resting height when opening
HOVER_Z = 80.0        # mm fingertip point for transit/hover
SUPPLY = [(160, 50), (205, 50), (160, 66), (205, 66), (182, 82)]          # (r, theta)
SLOTS = [(160, 110, 0), (205, 110, 0), (160, 97, 0), (205, 97, 0), (182.5, 103.5, 1)]  # (r, theta, level)


def pose(r, theta, z):
    """Working-tilt pose with the fingertip point at (r, theta, z)."""
    q, err = G.solve_rz(r, z)
    if q is None:
        raise ValueError(f"no IK for r={r} z={z} (resid {err:.1f})")
    return G.at_az(q, theta - G.REF_AZ)


def zlevel(level, place=False):
    return K.HAND_FLOOR + 15 + K.BLOCK * level + (RELEASE_CLEAR if place else 0.0)


def check(steps, tip_min):
    qs = [np.array(s["angles"]) for s in steps if "angles" in s]
    for a, b in zip(qs, qs[1:]):
        ok, why = G.path_ok(a, b, tip_min)
        if not ok:
            raise ValueError(why)


def write(name, desc, steps, tip_min):
    check(steps, tip_min)
    os.makedirs(OUT, exist_ok=True)
    H = [s for s in steps if "angles" in s][0]["angles"]
    mv = {"name": name, "description": desc, "created": "2026-09-25", "generator": "scripts/pyramid.py",
          "start_pose": H, "units": "degrees; speed 1-100", "steps": steps}
    path = os.path.join(OUT, f"{name}.json")
    json.dump(mv, open(path, "w"), indent=1)
    print(f"wrote {os.path.relpath(path)}  steps {len(steps)}  est {G.duration(steps)} s")


def main():
    H = G.solve_rz(180, 80)[0]
    if sys.argv[1] == "supply":
        steps = [G.A(H, label="home"), {"label": "open", "gripper": "open"}]
        for k, (r, th) in enumerate(SUPPLY):
            steps += [G.A(pose(r, th, HOVER_Z), label=f"hover_supply{k}"), {"label": f"wait_user_{k}", "wait": 12}]
        steps.append(G.A(H, label="home"))
        write("supply_hover", "Hover open fingers over each supply spot for 12 s so the user can place a block under them "
              "(square to the fingers, tag up).", steps, G.TIP_MIN)
    elif sys.argv[1] == "pickplace":
        k, r, th = int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
        sr, sth, lvl = SLOTS[k]
        g_hover, g_low = pose(r, th, HOVER_Z), pose(r, th, zlevel(0))
        p_hover = pose(sr, sth, max(HOVER_Z, zlevel(lvl) + 40))
        p_low = pose(sr, sth, zlevel(lvl, place=True))
        steps = [G.A(H, label="home"), {"label": "open", "gripper": "open"},
                 G.A(g_hover, label="above_block"), G.A(g_low, 8, "down_to_block"), {"label": "grasp", "gripper": "close"},
                 G.A(g_hover, label="lift"), G.A(p_hover, label="above_slot"), G.A(p_low, 6, "down_to_slot"),
                 {"label": "release", "gripper": "open"}, G.A(p_hover, label="clear"), G.A(H, label="home")]
        write(f"pp_{k}", f"Pick block at r={r:.1f} theta={th:.1f}, place in slot {k} (r={sr} theta={sth} level {lvl}).",
              steps, G.TIP_MIN_GRASP)
    return 0


if __name__ == "__main__":
    sys.exit(main())
