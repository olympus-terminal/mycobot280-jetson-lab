#!/usr/bin/env python3
"""Three-block pyramid (LAPTOP): two base blocks side by side, one bridging them on top (2026-09-27, user: "lets keep
trying to get the pyramid").

  1. base A: a tag-up block (θ 0-70, clear of others, reachable at level 1)
  2. B beside A: slot = A + SPACING along one of A's face directions (the free, reachable one), block yaw = A's yaw,
     place J6 chosen so the fingers close ACROSS the row (neither finger touches the face next to A)
  3. camera check (photo pose): measured A and B
  4. top: slot = midpoint of the measured A and B, level 1, fingers again across the row
Each step: cat guard (waits), cool parked, photo pose for looks, park. Recorded like any cycle (record_cycle).

Usage: ~/venvs/realsense/bin/python scripts/pyramid_test_20260927_204500.py [--dry-run] [--spacing 34]
Output: data/pyramid/pyramid_<stamp>.json (plan, results, measured positions)
"""
import argparse
import datetime
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
import demo_loop as D  # noqa: E402
import stack_test_20260927_165000 as ST  # noqa: E402
import kinematics as K  # noqa: E402
from arm_local import pose_xyz, PICKER_J6_OFFSET  # noqa: E402
import gen_movements as G  # noqa: E402


def closing_az(xy, z, j6):
    """Azimuth (deg, mod 180) of the finger closing axis at the pose reaching xy, z with this J6."""
    q = pose_xyz(np.array([xy[0], xy[1], z]))
    q[5] = j6
    a = K.fk_frames(q)[-1][:3, 1]
    return (math.degrees(math.atan2(a[1], a[0])) + PICKER_J6_OFFSET) % 180


def j6_across(xy, z, row_az):
    """J6 in the window whose closing axis is perpendicular to the row (within the J6 window), and its residual."""
    best = None
    for j6 in np.arange(G.WINDOWS[5, 0] + 1, G.WINDOWS[5, 1] - 1 + 1e-6, 1.0):
        err = ((closing_az(xy, z, j6) - (row_az + 90)) + 90) % 180 - 90
        if best is None or abs(err) < abs(best[1]):
            best = (float(j6), float(err))
    return best


def scan(stamp, tag):
    tmp = f"/tmp/pyr_{tag}_{stamp}.json"
    if not D.arm_pose("photo_up"):
        D.arm_pose("park")
        sys.exit("photo pose failed")
    blocks = D.find_blocks(tmp)
    D.cool_parked(True)
    return blocks, tmp


def place(stamp, name, src, slot, level, pj6):
    r, th = math.hypot(*slot), math.degrees(math.atan2(slot[1], slot[0]))
    th_src = math.degrees(math.atan2(src["xy"][1], src["xy"][0]))
    gb = D.rt_to_xy(src["xy"], D.region_rt(th_src))
    pb = D.place_rt(th)
    cmd = [D.PY, os.path.join(HERE, "record_cycle.py"), f"pyr{name}_{stamp}", "cycle_local.py",
           f"{src['xy'][0]:.1f}", f"{src['xy'][1]:.1f}", f"{r:.1f}", f"{th:.2f}", str(level),
           "--side", "far" if src["xy"][1] < 0 else "near", "--picker", "--grasp-z", str(D.GRASP_Z),
           "--max-attempts", str(D.GRASP_ATTEMPTS), "--release-to", "60", "--release-speed", "10",
           "--square-grip-max", "33", "--jaw-xy", str(D.JAW_XY[0]), str(D.JAW_XY[1]),
           "--place-bias-rt", f"{D.PLACE_BIAS_RT[0] + pb[0]:.1f}", f"{D.PLACE_BIAS_RT[1] + pb[1]:.1f}",
           "--bias", f"{gb[0]:.1f}", f"{gb[1]:.1f}", "--yaw", f"{src['yaw']:.1f}", "--place-j6", f"{pj6:.1f}", "--wrist-look"]
    ST.cat_wait(f"before {name}")
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=1500)
    res = [l for l in out.stdout.splitlines() if l.startswith("RESULT")]
    return res[-1] if res else f"exit {out.returncode}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--spacing", type=float, default=34.0, help="centre distance of the two base blocks (block 30 mm)")
    ap.add_argument("--scan", help="dry run: plan from a saved find_blocks JSON instead of the camera")
    a = ap.parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(REPO, "data", "pyramid"), exist_ok=True)
    rec = {"stamp": stamp, "spacing": a.spacing}
    save = lambda: json.dump(rec, open(os.path.join(REPO, "data", "pyramid", f"pyramid_{stamp}.json"), "w"), indent=1,
                             default=lambda o: np.round(o, 1).tolist() if isinstance(o, np.ndarray) else str(o))
    if not a.dry_run:
        ST.cat_wait("start")
        subprocess.run(["rsync", "-a", "--delete", "scripts", f"{D.JETSON}:orin-mycobot/"], cwd=REPO, capture_output=True)
        D.cool_parked(True)
        blocks, _ = scan(stamp, "0")
    else:
        blocks = D.find_blocks(a.scan, run=False) if a.scan else D.find_blocks(f"/tmp/pyr_dry_{stamp}.json")
    tops = [b for b in blocks if b["top"] and b["yaw"] is not None]
    pts = [b["xy"] for b in blocks]
    clear = lambda xy, excl=(): min([np.linalg.norm(xy - p) for p in pts if all(np.linalg.norm(p - e) > 1 for e in excl)] or [999])
    # --- plan A, B slot, row direction
    plan = None
    for A in sorted(tops, key=lambda b: abs(math.hypot(*b["xy"]) - 185)):
        thA = math.degrees(math.atan2(A["xy"][1], A["xy"][0]))
        if not (0 <= thA <= 70) or not D.reachable(*A["xy"], z=D.GRASP_Z + 32):
            continue
        for k in range(4):
            row_az = (A["yaw"] + 90 * k) % 360
            u = np.array([math.cos(math.radians(row_az)), math.sin(math.radians(row_az))])
            B = A["xy"] + a.spacing * u
            thB = math.degrees(math.atan2(B[1], B[0]))
            mid = (A["xy"] + B) / 2
            if not (-5 <= thB <= 72) or clear(B, excl=(A["xy"],)) < 55 or clear(A["xy"], excl=(A["xy"],)) < 60:
                continue
            if not (D.reachable(*B, z=D.GRASP_Z + 2) and D.slot_reachable(B) and D.reachable(*mid, z=D.GRASP_Z + 32)):
                continue
            j6B, eB = j6_across(B, D.GRASP_Z + 2, row_az)
            j6T, eT = j6_across(mid, D.GRASP_Z + 32, row_az)
            if abs(eB) > 8 or abs(eT) > 8:
                continue
            plan = {"A": A, "B": B, "row_az": row_az, "j6B": j6B, "errB": eB, "j6T": j6T, "errT": eT}
            break
        if plan:
            break
    if not plan:
        if not a.dry_run:
            D.arm_pose("park")
        sys.exit("no base block with a free, reachable neighbour slot")
    srcs = [b for b in tops if np.linalg.norm(b["xy"] - plan["A"]["xy"]) > 45 and np.linalg.norm(b["xy"] - plan["B"]) > 45
            and D.reachable(*b["xy"]) and D.grasp_reachable(b) and clear(b["xy"], excl=(b["xy"],)) >= D.SRC_CLEAR]
    if len(srcs) < 2:
        if not a.dry_run:
            D.arm_pose("park")
        sys.exit(f"need 2 source blocks, found {len(srcs)}")
    srcs.sort(key=lambda b: np.linalg.norm(b["xy"] - plan["B"]))
    rec["plan"] = {"A": {"id": plan["A"]["id"], "xy": plan["A"]["xy"], "yaw": plan["A"]["yaw"]}, "B_slot": plan["B"],
                   "row_az": plan["row_az"], "j6B": plan["j6B"], "j6_err_B": plan["errB"], "j6T": plan["j6T"],
                   "j6_err_T": plan["errT"], "srcB": {"id": srcs[0]["id"], "xy": srcs[0]["xy"]},
                   "srcT_first_choice": {"id": srcs[1]["id"], "xy": srcs[1]["xy"]}}
    print("PLAN", json.dumps(rec["plan"], default=lambda o: np.round(o, 1).tolist()), flush=True)
    save()
    if a.dry_run:
        return 0
    # --- B
    D.say("Building a pyramid.", True)
    rec["resB"] = place(stamp, "B", srcs[0], plan["B"], 0, plan["j6B"])
    print("B:", rec["resB"], flush=True)
    save()
    if "placed" not in rec["resB"]:
        return 3
    # --- measure A and B
    ST.cat_wait("measure")
    D.cool_parked(True)
    blocks2, _ = scan(stamp, "1")
    tops2 = [b for b in blocks2 if b["top"]]
    near = lambda p: min(tops2, key=lambda b: np.linalg.norm(b["xy"] - p)) if tops2 else None
    mA, mB = near(plan["A"]["xy"]), near(plan["B"])
    rec["measured"] = {"A": None if mA is None else mA["xy"], "B": None if mB is None else mB["xy"]}
    print("MEASURED", json.dumps(rec["measured"], default=lambda o: np.round(o, 1).tolist()), flush=True)
    if mA is None or mB is None or np.linalg.norm(mA["xy"] - plan["A"]["xy"]) > 15 or np.linalg.norm(mB["xy"] - plan["B"]) > 25:
        rec["result"] = "base pair not as planned"
        save()
        D.arm_pose("park")
        print("STOP: base pair not as planned", flush=True)
        return 4
    gap = np.linalg.norm(mB["xy"] - mA["xy"])
    rec["measured"]["centre_distance"] = round(float(gap), 1)
    if not (30 <= gap <= 44):
        rec["result"] = f"base spacing {gap:.1f} mm outside 30-44"
        save()
        D.arm_pose("park")
        print("STOP:", rec["result"], flush=True)
        return 4
    # --- top on the measured midpoint; re-pick a source from the new scan
    mid = (mA["xy"] + mB["xy"]) / 2
    row_az = math.degrees(math.atan2(*(mB["xy"] - mA["xy"])[::-1]))
    j6T, eT = j6_across(mid, D.GRASP_Z + 32, row_az)
    pts2 = [b["xy"] for b in blocks2]
    srcT = [b for b in tops2 if np.linalg.norm(b["xy"] - mA["xy"]) > 45 and np.linalg.norm(b["xy"] - mB["xy"]) > 45
            and b["yaw"] is not None and D.reachable(*b["xy"]) and D.grasp_reachable(b)
            and min([np.linalg.norm(b["xy"] - p) for p in pts2 if np.linalg.norm(b["xy"] - p) > 1] or [999]) >= D.SRC_CLEAR]
    if not srcT:
        rec["result"] = "no source for the top block"
        save()
        print("STOP:", rec["result"], flush=True)
        return 5
    srcT.sort(key=lambda b: np.linalg.norm(b["xy"] - mid))
    rec["top"] = {"slot": mid, "row_az": row_az, "j6": j6T, "j6_err": eT, "src": {"id": srcT[0]["id"], "xy": srcT[0]["xy"]}}
    rec["resT"] = place(stamp, "T", srcT[0], mid, 1, j6T)
    print("TOP:", rec["resT"], flush=True)
    # --- final check
    ST.cat_wait("final look")
    D.cool_parked(True)
    tmp = f"/tmp/pyr_final_{stamp}.json"
    D.arm_pose("photo_up")
    D.find_blocks(tmp)
    D.arm_pose("park")
    cal = json.load(open(D.CALIB))
    best = None
    for b in json.load(open(tmp))["blocks"]:
        if b["top_face_tag"] and b["id"] not in D.FIXED_TAGS:
            d = float(np.linalg.norm(ST.top_xy_at(b["pixel"], 44.9, cal) - mid))
            best = d if best is None or d < best else best
    rec["check_top_mm"] = None if best is None else round(best, 1)
    print("CHECK top tag at level 1 vs midpoint:", rec["check_top_mm"], "mm", flush=True)
    save()
    return 0


if __name__ == "__main__":
    sys.exit(main())
