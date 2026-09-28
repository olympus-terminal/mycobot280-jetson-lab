#!/usr/bin/env python3
"""Generate a validated library of well-defined movements (movement files) for recording sessions.

All movements keep the hand at the WORKING TILT (tool 60 deg from vertical, J5 ~54, J6 ~20): the wrist camera
and the lower protrusion prevent J5 from going below ~44 (self-collision), so a straight-down hand is impossible.

Reaches are solved in cylindrical coordinates: IK once per (radius r, height z) at a reference azimuth,
then other azimuths only change J1 (the J1 axis is the controller z axis). The hand keeps the same tilt
relative to the arm.

Every waypoint AND the joint-space path between consecutive waypoints (21 samples) is checked:
firmware limits, the windows in WINDOWS, all joint frames >= FRAME_MARGIN above the table,
fingertip >= TIP_MIN (TIP_MIN_GRASP for the grasp families).

Usage:
    python scripts/gen_movements.py --out movements/library_v1
"""
import argparse
import datetime
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402

# Allowed joint windows for library motions (deg). J1: the sector used so far (Jetson/tripod elsewhere).
# J5 upper 110: stalled at 114.9 going to 120 with J6 ~20 (batch 1, 2026-09-24)
# J6 upper 35: J6 -> 45 stopped at 40.6 (camera-cable twist suspected, 2026-09-25)
WINDOWS = np.array([[30, 100], [-135, 135], [-150, 150], [-145, 145], [50, 110], [-10, 35]], float)
FRAME_MARGIN = 25.0   # mm above the table estimate for every joint frame
TIP_MIN = 30.0        # mm fingertip height for non-grasp movements
TIP_MIN_GRASP = 15.0  # mm for grasp movements
TILT = np.radians(60)
REF_TIP_XY = np.array([9.0, 180.0])            # verified pick location (controller frame)
REF_AZ = np.degrees(np.arctan2(REF_TIP_XY[1], REF_TIP_XY[0]))
REF_AXIS = np.array([-np.sin(TILT), 0.0, -np.cos(TILT)])  # verified working tilt at REF_AZ
REF_J1 = 69.0
DEG_PER_S_AT_SPEED10 = 14.0  # observed 2026-09-24 (J2 76 deg in 5.4 s at speed 10)


def solve_rz(r, z, axis=None, seed=None):
    """Joint angles for fingertip at radius r, height z, at the reference azimuth, working tilt (or `axis`)."""
    tip = np.array([r * np.cos(np.radians(REF_AZ)), r * np.sin(np.radians(REF_AZ)), z])
    seed = [REF_J1, -40, -80, 0, 54, 20] if seed is None else seed
    q, err = K.ik_tool(tip, REF_AXIS if axis is None else axis, seed, WINDOWS)
    return (q, err) if err < 0.5 else (None, err)


def at_az(q, dtheta):
    q = np.array(q, float)
    q[0] += dtheta
    return q


def path_ok(qa, qb, tip_min):
    for s in np.linspace(0, 1, 21):
        q = qa + (qb - qa) * s
        if np.any(q < WINDOWS[:, 0] - 1e-6) or np.any(q > WINDOWS[:, 1] + 1e-6):
            return False, f"outside windows at s={s:.2f}: {np.round(q, 1).tolist()}"
        frames = K.fk_frames(q)[1:]
        low = min(F[2, 3] for F in frames)
        tip = K.fingertip(q)[0][2]
        if low < K.TABLE_Z + FRAME_MARGIN:
            return False, f"joint frame z {low:.0f} mm at s={s:.2f}"
        if tip < K.TABLE_Z + tip_min:
            return False, f"fingertip z {tip:.0f} mm at s={s:.2f}"
    return True, ""


def duration(steps):
    t, prev = 0.0, None
    for s in steps:
        if "angles" in s:
            if prev is not None:
                t += np.abs(np.array(s["angles"]) - prev).max() / (DEG_PER_S_AT_SPEED10 * s["speed"] / 10) + 1.0
            prev = np.array(s["angles"])
        elif "gripper" in s:
            t += 3.0
        elif "wait" in s:
            t += s["wait"]
    return round(t, 1)


def steps_label(steps, k):
    """Label of the k-th angles step."""
    return [s.get("label", "") for s in steps if "angles" in s][k]


def A(q, speed=10, label=""):
    return {"label": label, "angles": [round(float(v), 2) for v in q], "speed": speed}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="movements/library_v1")
    args = ap.parse_args()

    grid = {}
    for r in (150, 180, 210):
        for z in (40, 80, 120, 160):
            q, err = solve_rz(r, z)
            grid[(r, z)] = q
            print(f"IK r={r} z={z}: {'OK ' + str(np.round(q, 1).tolist()) if q is not None else f'no solution (resid {err:.1f})'}")
    H = grid[(180, 80)]
    if H is None:
        sys.exit("home pose (r=180, z=80) not solvable")

    def sweep(j, lo, hi):
        a, b = H.copy(), H.copy()
        a[j - 1], b[j - 1] = lo, hi
        return [A(H, label="home"), A(a, label=f"J{j}_{lo:g}"), A(b, label=f"J{j}_{hi:g}"), A(H, label="home")]

    lib = {
        "sweep_j1": ("J1 base rotation across the working sector", sweep(1, 40, 95), TIP_MIN),
        "sweep_j2": ("J2 shoulder +-10 deg", sweep(2, H[1] - 10, H[1] + 10), TIP_MIN),
        "sweep_j3": ("J3 elbow +-12 deg", sweep(3, H[2] - 12, H[2] + 12), TIP_MIN),
        "sweep_j4": ("J4 wrist +-15 deg", sweep(4, H[3] - 15, H[3] + 15), TIP_MIN),
        "sweep_j5": ("J5 wrist tilt 60..105 deg", sweep(5, 60, 105), TIP_MIN),
        "sweep_j6": ("J6 wrist roll 5..35 deg", sweep(6, 5, 35), TIP_MIN),
    }
    pts = [(r, z) for r in (150, 180, 210) for z in (80, 120) if grid.get((r, z)) is not None]
    steps = [A(H, label="home")]
    for dth in (-20, 0, 20):
        for r, z in pts:
            steps.append(A(at_az(grid[(r, z)], dth), label=f"r{r}_z{z}_az{dth:+d}"))
    steps.append(A(H, label="home"))
    lib["reach_grid"] = ("Visit a grid of reach points (r 150-210, z 80-120) at three azimuths", steps, TIP_MIN)
    bob = [A(H, label="home")]
    for _ in range(2):
        for z in (40, 120, 80):
            bob.append(A(grid[(180, z)], label=f"z{z}"))
    lib["vertical_bob"] = ("Raise and lower the hand at r=180 (z 40-120), twice", bob, TIP_MIN)
    lib["radial_in_out"] = ("Move the hand in and out (r 150-210) at z=80",
                            [A(H, label="home")] + [A(grid[(r, 80)], label=f"r{r}") for r in (150, 210, 150, 210)]
                            + [A(H, label="home")], TIP_MIN)
    sq = [(150, 80), (210, 80), (210, 120), (150, 120), (150, 80)]
    lib["square_rz"] = ("Trace a square in the radius-height plane", [A(H, label="home")]
                        + [A(grid[p], label=f"r{p[0]}_z{p[1]}") for p in sq] + [A(H, label="home")], TIP_MIN)
    lib["gripper_cycle"] = ("Open/close the gripper three times at home",
                            [A(H, label="home")] + [{"label": f"{g}{k}", "gripper": g} for k in range(3)
                                                    for g in ("open", "close")] + [{"label": "open", "gripper": "open"}],
                            TIP_MIN)
    g40, g20 = K.ik_tool([9, 180, 40], REF_AXIS, grid[(180, 80)], WINDOWS)[0], K.ik_tool([9, 180, 20], REF_AXIS, grid[(180, 80)], WINDOWS)[0]
    poses = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "poses.json")))
    REST, PRE = np.array(poses["REST"]["angles"]), np.array(poses["PRE_REST"]["angles"])
    lib["park"] = ("Go to the remembered REST pose (hand on the table), then release the servos",
                   [A(H, label="home"), A(PRE, label="pre_rest"), A(REST, 5, "rest"), {"label": "release", "release": True}],
                   None)
    lib["unpark"] = ("From REST, lift and go to home", [A(REST, 5, "rest"), A(PRE, 8, "pre_rest"), A(H, label="home")], None)
    lib["air_pick_place"] = ("Pick-and-place motion without a block (same poses as the verified pick/place)",
                             [A(H, label="pregrasp"), {"label": "open", "gripper": "open"}, A(g40, label="lower_z40"),
                              A(g20, 8, "lower_z20"), {"label": "close", "gripper": "close"}, A(H, label="lift"),
                              A(at_az(H, -25), label="carry"), A(at_az(g40, -25), label="lower_z40"),
                              A(at_az(g20, -25), 8, "lower_z20"), {"label": "open", "gripper": "open"},
                              A(at_az(H, -25), label="lift"), A(H, label="home")], TIP_MIN_GRASP)

    os.makedirs(args.out, exist_ok=True)
    index = {"generated": datetime.datetime.now().isoformat(timespec="seconds"), "generator": "scripts/gen_movements.py",
             "constraints": {"windows_deg": WINDOWS.tolist(), "frame_margin_mm": FRAME_MARGIN, "tip_min_mm": TIP_MIN,
                             "tip_min_grasp_mm": TIP_MIN_GRASP, "table_z_estimate_mm": K.TABLE_Z, "tilt_deg": 60,
                             "gripper_len_mm": K.GRIPPER_LEN},
             "movements": []}
    ok_all = True
    for name, (desc, steps, tip_min) in lib.items():
        qs = [np.array(s["angles"]) for s in steps if "angles" in s]
        problems = []
        for k, (a, b) in enumerate(zip(qs, qs[1:])):
            if tip_min is None and ("rest" in (steps_label(steps, k), steps_label(steps, k + 1))):
                continue  # the pre_rest <-> rest segment ends/starts on the table by design (J2-only move)
            ok, why = path_ok(a, b, TIP_MIN if tip_min is None else tip_min)
            if not ok:
                problems.append(why)
        mv = {"name": name, "description": desc, "created": datetime.date.today().isoformat(),
              **({"allow_hot": True} if name == "park" else {}),
              "start_pose": [round(float(v), 2) for v in qs[0]],
              "units": "degrees; speed 1-100", "generator": "scripts/gen_movements.py", "steps": steps}
        raw = json.dumps(mv, indent=1).encode()
        status = "OK" if not problems else "REJECTED: " + problems[0]
        print(f"{name:15s} {len(steps):2d} steps  ~{duration(steps):5.1f} s  {status}")
        if problems:
            ok_all = False
            continue
        open(os.path.join(args.out, f"{name}.json"), "wb").write(raw)
        index["movements"].append({"name": name, "file": f"{name}.json", "sha256": hashlib.sha256(raw).hexdigest(),
                                   "steps": len(steps), "est_duration_s": duration(steps), "description": desc})
    json.dump(index, open(os.path.join(args.out, "index.json"), "w"), indent=1)
    print(f"total ~{sum(m['est_duration_s'] for m in index['movements']):.0f} s for {len(index['movements'])} movements")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
