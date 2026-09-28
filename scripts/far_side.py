#!/usr/bin/env python3
"""Movement files for the FAR-SIDE workspace (J1 ~ -70..-122; theta = J1 + 18.1): the open table the RealSense sees.

Found 2026-09-25: the near side (J1 30..100) is at the table's front edge and at the bottom edge of the RealSense view.
All poses use the working tilt (fingertip point, 60 deg from vertical), rotated about the base by J1 only.
Every file's angles steps are checked with gen_movements.path_ok before writing (grasp steps with the grasp minimum).

Writes movements/far/:
  to_far.json      near home -> raised (survey) pose -> rotate J1 raised -> far home
  to_near.json     far home -> raised -> rotate back -> near home
  touch_far.json   far-side table-touch test (5 mm steps, stops on contact)
  handoff_calib_far.json   hover open over the hand-off spot 15 s (user places a block), pick, 12-pose tour, put it back, far home
  park_far.json    far home -> pre-rest -> REST_FAR (REST with J1 = -115, hand on the table) -> release
  unpark_far.json  REST_FAR -> pre-rest -> far home
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_movements as G  # noqa: E402
import kinematics as K  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "movements", "far")
W = G.WINDOWS.copy()
W[0] = [-122, 140]   # 100 -> 110 (12:20) -> 120 (14:40) -> 140 (17:00: user-placed blocks at theta 117-120 need J1 ~134-138; firmware 168)
W[4] = [-5, 110]     # J5: the user's picker demo (14:34) used J5 ~0-5 with J4 +30..+40: no self-collision on that branch
W[5] = [-40, 40]     # J6: demo used -40..+43 (camera cable OK)
G.WINDOWS = W  # far side needs J1 down to -122 (mapped clear 2026-09-25)
J1_OFF = G.REF_AZ - G.REF_J1  # theta = J1 + J1_OFF
FAR_THETA = -85.0
HANDOFF = (180, -80.0)


# Far-reach branch (2026-09-25): tool tilt 60 deg from vertical but leaning 60 deg from radial (working tilt leans 93),
# elbow/wrist on the J4 ~ +60..+115 branch. Model reach at grasp height r 200-300 mm (working tilt: <= 243).
REACH_R = 238.0
_az = np.radians(G.REF_AZ)
_rad, _tan = np.array([np.cos(_az), np.sin(_az), 0.0]), np.array([-np.sin(_az), np.cos(_az), 0.0])
REACH_AXIS = np.sin(np.radians(60)) * (np.cos(np.radians(60)) * _rad + np.sin(np.radians(60)) * _tan) - np.cos(np.radians(60)) * np.array([0, 0, 1.0])
REACH_SEED = [75, -76, -71, 84, 56, 20]
# touch test 2026-09-25 10:33 (data/*_cycle_reach_touch2, r 270/280): on this branch the hand meets the table at
# fingertip-point z ~ 21 (working tilt: K.HAND_FLOOR = 5); the arm also sags ~20 mm below its command here.
HAND_FLOOR_REACH = 21.0
# 10:36 reach pick test: at fingertip-point z 36 (= floor + 15) the closed fingers sat at the block TOP (RealSense frames):
# on this branch the fingertips are ~16 mm above the table when the hand (another part) meets it. Grasp the block's upper
# part: fingertip-point z = HAND_FLOOR_REACH + 2.
GRASP_Z_REACH = HAND_FLOOR_REACH + 2.0


# "Picker" branch (2026-09-25 14:40, from the user's limp-arm demo): tool pointing straight down, wrist nearly straight.
# Seed = the demo keyframe [98.7, -63.3, -31.7, 5.8, 2.7, -32.2] (tip (14, 276, 28), tool 3 deg from vertical) at REF_AZ.
PICKER = False                      # module default, set by scripts (arm_local.pose_xyz passes picker=None -> this)
PICKER_AXIS = np.array([0.0, 0.0, -1.0])
PICKER_SEED = [98.7, -63.3, -31.7, 5.8, 2.7, 0.0]


def pose(r, theta, z, reach=None, picker=None):
    """Joint angles: fingertip at (r, theta, z). reach=None -> far-reach branch iff r > REACH_R.
    picker (None -> module PICKER): vertical tool; J6 left at 0 (callers set it for the block yaw)."""
    if picker is None:
        picker = PICKER
    if picker:
        q, err = G.solve_rz(r, z, PICKER_AXIS, PICKER_SEED)
        if q is None:
            raise ValueError(f"no picker IK r={r} z={z}")
        q = G.at_az(q, theta - G.REF_AZ)
        q[5] = 0.0
        return q
    if reach is None:
        reach = r > REACH_R
    q, err = G.solve_rz(r, z, REACH_AXIS, REACH_SEED) if reach else G.solve_rz(r, z)
    if q is None:
        raise ValueError(f"no IK r={r} z={z}")
    return G.at_az(q, theta - G.REF_AZ)


def write(name, desc, steps, start, tip_min=G.TIP_MIN, skip=()):
    qs = [(s.get("label", ""), np.array(s["angles"])) for s in steps if "angles" in s]
    for (la, a), (lb, b) in zip(qs, qs[1:]):
        if la in skip or lb in skip:
            continue
        ok, why = G.path_ok(a, b, tip_min)
        if not ok:
            raise ValueError(f"{name}: {la}->{lb}: {why}")
    os.makedirs(OUT, exist_ok=True)
    mv = {"name": name, "description": desc, "created": "2026-09-25", "generator": "scripts/far_side.py",
          **({"allow_hot": True} if name == "park_far" else {}),
          "start_pose": [round(float(v), 2) for v in start], "units": "degrees; speed 1-100", "steps": steps}
    json.dump(mv, open(os.path.join(OUT, f"{name}.json"), "w"), indent=1)
    print(f"wrote far/{name}.json  steps {len(steps)}  est {G.duration(steps)} s")


def zgrasp(level=0, floor=None, place=False):
    """Fingertip-point z for holding a block whose bottom is at `level` blocks above the table."""
    f = K.HAND_FLOOR if floor is None else floor
    return f + 15 + K.BLOCK * level + (2.0 if place else 0.0)


def gen_pick(r, th, floor, name="pick"):
    H_far = pose(180, FAR_THETA, 80)
    zg = zgrasp(0, floor)
    steps = [G.A(H_far, label="home_far"), {"label": "open", "gripper": "open"},
             G.A(pose(r, th, 80), label="above_block"), G.A(pose(r, th, zg + 20), 8, "near_block"),
             G.A(pose(r, th, zg), 6, "down_to_block"), {"label": "grasp", "gripper": "close"},
             G.A(pose(r, th, 80), 8, "lift"), G.A(H_far, label="home_far")]
    write(name, f"Pick the block at r={r:.1f} theta={th:.1f} (grasp fingertip z {zg:.1f})", steps, H_far, tip_min=G.TIP_MIN_GRASP)


def gen_place(r, th, level, floor, name="place"):
    H_far = pose(180, FAR_THETA, 80)
    zp = zgrasp(level, floor, place=True)
    zh = max(80.0, zp + 45)
    steps = [G.A(H_far, label="home_far"), G.A(pose(r, th, zh), label="above_slot"),
             G.A(pose(r, th, zp + 15), 8, "near_slot"), G.A(pose(r, th, zp), 5, "down_to_slot"),
             {"label": "release", "gripper": "open"}, G.A(pose(r, th, zh), 8, "clear"), G.A(H_far, label="home_far")]
    write(name, f"Place the held block at r={r:.1f} theta={th:.1f} level {level} (fingertip z {zp:.1f})", steps, H_far,
          tip_min=G.TIP_MIN_GRASP)


def gen_tour(name="tour_far"):
    H_far = pose(180, FAR_THETA, 80)
    tour = [G.A(H_far, label="home_far")]
    for rr, zz in ((150, 80), (180, 80), (210, 80), (150, 120), (180, 120), (210, 120)):
        for tth in ((-100, -70) if zz == 80 else (-92, -62)):
            tour += [G.A(pose(rr, tth, zz), label=f"r{rr}_z{zz}_th{tth}"), {"label": f"hold_r{rr}_z{zz}_th{tth}", "wait": 2.0}]
    tour.append(G.A(H_far, label="home_far"))
    write(name, "Far-side calibration tour holding a tagged block: 12 poses, 2 s holds", tour, H_far)


def main():
    if len(sys.argv) > 1:
        cmd, a = sys.argv[1], [float(x) for x in sys.argv[2:]]
        if cmd == "pick":
            gen_pick(a[0], a[1], a[2] if len(a) > 2 else None, name=f"pick")
        elif cmd == "place":
            gen_place(a[0], a[1], int(a[2]), a[3] if len(a) > 3 else None, name="place")
        elif cmd == "hover":  # hover R TH Z [from_r from_th from_z]: move (from far home or a previous hover) and stay
            dst = pose(a[0], a[1], a[2])
            src = pose(a[3], a[4], a[5]) if len(a) >= 6 else pose(180, FAR_THETA, 80)
            write("hover", f"Hover the fingertip point at r={a[0]:.1f} theta={a[1]:.1f} z={a[2]:.0f}",
                  [G.A(src, label="from"), G.A(dst, 8, "hover")], src)
        elif cmd == "tour":
            gen_tour()
        return 0
    H_near = G.solve_rz(180, 80)[0]
    H_far = pose(180, FAR_THETA, 80)
    S_near = np.array([70.8, -32.0, -33.6, -49.8, 50.6, 20.0])  # survey (raised) pose, verified 2026-09-25
    S_far = S_near.copy()
    S_far[0] = H_far[0]
    A = G.A
    write("to_far", "Near home -> raise -> rotate J1 to the far side (raised) -> far home",
          [A(H_near, label="home_near"), A(S_near, label="raise"), A(S_far, 10, "rotate_far"), A(H_far, label="home_far")], H_near)
    write("to_near", "Far home -> raise -> rotate back -> near home",
          [A(H_far, label="home_far"), A(S_far, label="raise"), A(S_near, 10, "rotate_near"), A(H_near, label="home_near")], H_far)

    # touch test on the far side
    r, th = 180, -80.0
    steps, seed = [A(H_far, label="home_far"), {"label": "close", "gripper": "close"}], None
    for z in [60, 40, 30] + list(range(25, -30, -5)):
        steps.append(A(pose(r, th, z), 5, f"z{z}"))
    write("touch_far", "Far-side table touch: fingertip (r180, theta -80) down in 5 mm steps until contact stops the arm",
          steps, H_far, tip_min=-50, skip=())

    # hand-off + calibration tour (far side)
    hr, hth = HANDOFF
    grasp_z = K.HAND_FLOOR + 15
    tour = []
    for rr, zz in ((150, 80), (180, 80), (210, 80), (150, 120), (180, 120), (210, 120)):
        for tth in ((-100, -70) if zz == 80 else (-92, -62)):
            tour += [A(pose(rr, tth, zz), label=f"r{rr}_z{zz}_th{tth}"), {"label": f"hold_r{rr}_z{zz}_th{tth}", "wait": 2.0}]
    steps = [A(H_far, label="home_far"), {"label": "open", "gripper": "open"},
             A(pose(hr, hth, 80), label="hover_handoff"), {"label": "wait_user_place", "wait": 15},
             A(pose(hr, hth, 40), label="lower_z40"), A(pose(hr, hth, grasp_z), 8, "lower_grasp"),
             {"label": "grasp", "gripper": "close"}, A(pose(hr, hth, 80), label="lift")] + tour + [
             A(pose(hr, hth, 80), label="back_to_handoff"), A(pose(hr, hth, grasp_z + 2), 6, "put_down"),
             {"label": "release", "gripper": "open"}, A(pose(hr, hth, 80), label="clear"), A(H_far, label="home_far")]
    write("handoff_calib_far", "Hand-off: user places a tagged block under the open fingers (15 s), pick, 12-pose far-side "
          "calibration tour (2 s holds), put the block back, far home", steps, H_far, tip_min=G.TIP_MIN_GRASP)

    poses = json.load(open(os.path.join(HERE, "..", "config", "poses.json")))
    REST_F = np.array(poses["REST"]["angles"])
    REST_F[0] = -115.0
    PRE_F = REST_F.copy()
    PRE_F[1] += 20
    write("park_far", "Far home -> pre-rest -> REST_FAR (REST with J1 -115: hand resting on the table at theta ~-98) -> release",
          [A(H_far, label="home_far"), A(PRE_F, label="pre_rest"), A(REST_F, 5, "rest"), {"label": "release", "release": True}],
          H_far, skip=("rest",))
    write("unpark_far", "REST_FAR -> pre-rest -> far home", [A(REST_F, 5, "rest"), A(PRE_F, 8, "pre_rest"), A(H_far, label="home_far")],
          REST_F, skip=("rest",))
    return 0


if __name__ == "__main__":
    sys.exit(main())
