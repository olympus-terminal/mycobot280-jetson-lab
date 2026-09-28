#!/usr/bin/env python3
"""Two-block tower test (LAPTOP): stack one block on another, recorded, then camera-check the stack.

2026-09-27 (user: "can we build a pyramid? any other structures?"): first step = level-1 placement onto a block.
Uses demo_loop's pieces (find_blocks, reach checks, grasp/place region corrections) so the move is planned exactly
like a rehearsal move, except the slot is the base block's centre at level 1 and the place J6 aligns the fingers
(and so the held block) with the base block's edges.

Usage: ~/venvs/realsense/bin/python scripts/stack_test_20260927_165000.py [--base-tag ID] [--src-tag ID] [--dry-run]
Output: data/<stamp>_cycle_stack_<n>/ (record_cycle) + data/stack/stack_<stamp>.json (plan + camera check)
"""
import argparse
import datetime
import glob
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
import demo_loop as D  # noqa: E402
from arm_local import j6_picker  # noqa: E402

INHAND_START_J5 = 44     # J5 limit before the pick-hold-measure-place sequence
Z_TOP1 = 14.9 + 30.0     # tag plane of a block standing on another block (find_blocks.Z_TOP + one block)


def top_xy_at(pixel_1920, z, cal):
    """The tag-centre pixel ray (1920x1080 frame) cut at arm-frame height z."""
    R, t, K = np.array(cal["R_base_to_cam"]), np.array(cal["t_base_to_cam_mm"]), cal["intrinsics"]
    u, v = pixel_1920[0] / 1.5, pixel_1920[1] / 1.5          # calibration intrinsics are for 1280x720
    ray_b = R.T @ np.array([(u - K["ppx"]) / K["fx"], (v - K["ppy"]) / K["fy"], 1.0])
    cam_b = -R.T @ t
    s = (z - cam_b[2]) / ray_b[2]
    return (cam_b + s * ray_b)[:2]


def cat_wait(where):
    """2026-09-27 20:15: a cat was on the table and the arm moved (the manual stack_test runs never checked the guard;
    my shell chain ignored its exit code). Wait here until the silent check is clear; never time out."""
    t0 = time.time()
    while True:
        g = subprocess.run([D.PY, os.path.join(REPO, "cat_guard", "cat_guard.py"), "check"], capture_output=True, text=True)
        if g.returncode == 0:
            return
        if time.time() - t0 > 20:
            print(f"cat guard ({where}): intrusion - waiting", flush=True)
            t0 = time.time()
        time.sleep(10)


HOLD_TAG_ABOVE_TIP = 9.9   # mm: block top above the model fingertip when grasped at z 5 (block top 14.9 on the table)


def read_angles():
    r = subprocess.run(["ssh", "-o", "LogLevel=ERROR", D.JETSON,
                        "~/venvs/mycobot/bin/python -c \"from pymycobot import MyCobot280; "
                        "mc=MyCobot280('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0','1000000'); print(mc.get_angles())\""],
                       capture_output=True, text=True, timeout=30)
    return np.array(json.loads(r.stdout.strip().splitlines()[-1]))


def measured_place(cmd, rec, base, r, th, pj6, a, stamp):
    """(A) pick and hold at home (--no-place), (B) camera: the held block's top tag at its known height -> in-hand offset
    of the block from the model fingertip, in the flange frame, (C) --place-only with that offset as --jaw-xy and no
    place bias. If the held tag isn't found, put the block back at its grasp spot (level 0)."""
    import kinematics as K
    out = {}
    # (0) extra cooling: A + hold + C hold the arm ~2.5 min (J5 +12 C / 100 s)
    t = D.temps()
    while t is not None and t[4] > INHAND_START_J5:
        print(f"cooling for the in-hand sequence: {t}", flush=True)
        time.sleep(45)
        t = D.temps()
    cat_wait("before A")
    cmd_a = list(cmd)
    cmd_a[2] = f"stackA_{stamp}"
    cmd_a += ["--no-place"]
    ra = subprocess.run(cmd_a, capture_output=True, text=True, timeout=1500)
    res_a = [l for l in ra.stdout.splitlines() if l.startswith("RESULT")]
    out["result_a"] = res_a[-1] if res_a else f"exit {ra.returncode}"
    print("A:", out["result_a"], flush=True)
    if "holding at home" not in out["result_a"]:
        out["result"] = "A failed: " + out["result_a"]
        return out
    sess_a = sorted(glob.glob(os.path.join(REPO, "data", f"*_cycle_stackA_{stamp}")))[-1]
    gxy = json.load(open(os.path.join(sess_a, "summary.json")))["grasp_xy"]
    # (B) measure while holding
    q = read_angles()
    tip = K.fingertip(q)[0]
    Fl = K.fk_frames(q)[-1]
    tmp = f"/tmp/stack_inhand_{stamp}.json"
    D.find_blocks(tmp)
    cal = json.load(open(D.CALIB))
    z_tag = tip[2] + HOLD_TAG_ABOVE_TIP
    best = None
    for b in json.load(open(tmp))["blocks"]:
        if b["id"] in D.FIXED_TAGS:
            continue
        p = top_xy_at(b["pixel"], z_tag, cal)
        d = float(np.linalg.norm(p - tip[:2]))
        if best is None or d < best[0]:
            best = (d, p, b["id"], b["n_frames"])
    out["inhand"] = {"angles": np.round(q, 2).tolist(), "tip": np.round(tip, 1).tolist(), "z_tag": round(float(z_tag), 1),
                     "best": None if best is None else {"d": round(best[0], 1), "xy": np.round(best[1], 1).tolist(),
                                                        "id": best[2], "frames": best[3]}}
    if best is None or best[0] > 45:
        print("held tag not found:", out["inhand"], flush=True)
        slot_r, slot_th, lvl, jaw = math.hypot(*gxy), math.degrees(math.atan2(gxy[1], gxy[0])), 0, D.JAW_XY
    else:
        off_w = best[1] - tip[:2]
        jaw = (float(off_w @ Fl[:2, 0] / np.linalg.norm(Fl[:2, 0])), float(off_w @ Fl[:2, 1] / np.linalg.norm(Fl[:2, 1])))
        out["inhand"]["offset_world"] = np.round(off_w, 1).tolist()
        out["inhand"]["jaw_xy_measured"] = [round(jaw[0], 1), round(jaw[1], 1)]
        print(f"in-hand: tag {best[2]} at {np.round(best[1], 1).tolist()} vs tip {np.round(tip[:2], 1).tolist()} -> "
              f"jaw_xy {out['inhand']['jaw_xy_measured']} (model {list(D.JAW_XY)})", flush=True)
        slot_r, slot_th, lvl = r, th, a.level
    # (C) place only (while holding: no waiting on the guard here would leave the arm heating; place, then the next
    # sequence waits)
    cmd_c = [D.PY, os.path.join(HERE, "record_cycle.py"), f"stackC_{stamp}", "cycle_local.py",
             f"{gxy[0]:.1f}", f"{gxy[1]:.1f}", f"{slot_r:.1f}", f"{slot_th:.2f}", str(lvl), "--side", "near", "--picker",
             "--place-only", "--grasp-z", str(D.GRASP_Z), "--release-to", "60", "--release-speed", "10",
             "--jaw-xy", f"{jaw[0]:.1f}", f"{jaw[1]:.1f}", "--place-bias-rt", "0", "0", "--place-j6", f"{pj6:.1f}"]
    rc = subprocess.run(cmd_c, capture_output=True, text=True, timeout=1500)
    res_c = [l for l in rc.stdout.splitlines() if l.startswith("RESULT")]
    out["result"] = res_c[-1] if res_c else f"exit {rc.returncode}"
    if lvl == 0:
        out["result"] = "put back (held tag not found): " + out["result"]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-tag", type=int)
    ap.add_argument("--src-tag", type=int)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--level", type=int, default=1, help="1 = on a block, 2 = on a two-block tower")
    ap.add_argument("--base-xy", nargs=2, type=float, help="manual base (e.g. the camera level-1 position of a tower top)")
    ap.add_argument("--base-yaw", type=float, help="with --base-xy: the top block's tag yaw (deg mod 90)")
    ap.add_argument("--measure-inhand", action="store_true",
                    help="pick + hold at home, measure the held block's position with the camera, then place with that "
                         "in-hand offset (2026-09-27: squarely gripped blocks still landed 20-35 mm off: the block's place "
                         "across the finger width varies)")
    a = ap.parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(REPO, "data", "stack"), exist_ok=True)
    tmp = f"/tmp/stack_blocks_{stamp}.json"
    if not a.dry_run:
        cat_wait("start")
    subprocess.run(["rsync", "-a", "--delete", "scripts", f"{D.JETSON}:orin-mycobot/"], cwd=REPO, capture_output=True)
    photo = not a.dry_run
    if photo:   # 2026-09-27: cool first, parked (the first tower-3 try was refused: J5 55 > cycle_local START_MAX_C 52)
        # 18:05: ALWAYS park before waiting - the arm had been left upright (photo pose) and J5 rose 55 -> 70 C (servo
        # limit) during this "cooling" wait. arm_pose park is a no-op when the arm is already low.
        D.arm_pose("park")
        t = D.temps()
        while t is not None and (t[4] > D.START_J5 or max(t) > D.START_ALL):
            print(f"cooling (parked): {t}", flush=True)
            import time
            time.sleep(60)
            t = D.temps()
    if photo and not D.arm_pose("photo_up"):       # 2026-09-27: look with the arm straight up
        D.arm_pose("park")
        sys.exit("photo pose failed")
    blocks = D.find_blocks(tmp)
    raw = {tuple(np.round(b["plane_xy_mm"], 1)): b for b in json.load(open(tmp))["blocks"]}
    tops = [b for b in blocks if b["top"] and b["yaw"] is not None]
    pts = [b["xy"] for b in blocks]

    if a.base_xy:
        # 2026-09-27 17:58: a tower's top tag is projected onto the table plane by find_blocks -> ~20 mm off the tower's
        # real footprint (a neighbour 72 mm away read as 43 mm). Use the tower footprint (--base-xy) instead.
        cal0 = json.load(open(D.CALIB))
        raw0 = [r for r in json.load(open(tmp))["blocks"] if r["id"] not in D.FIXED_TAGS]
        for r in raw0:
            lvl = [float(np.linalg.norm(top_xy_at(r["pixel"], 14.9 + 30.0 * k, cal0) - np.array(a.base_xy))) for k in (1, 2)]
            if min(lvl) < 15:
                pts = [p for p in pts if np.linalg.norm(p - np.array(r["plane_xy_mm"])) > 1] + [np.array(a.base_xy)]
                tops = [b for b in tops if np.linalg.norm(b["xy"] - np.array(r["plane_xy_mm"])) > 1]

    def clear(b):
        return min([np.linalg.norm(b["xy"] - p) for p in pts if np.linalg.norm(b["xy"] - p) > 1] or [999])

    def ok_src(b):
        th = math.degrees(math.atan2(b["xy"][1], b["xy"][0]))
        return D.reachable(*b["xy"]) and D.grasp_reachable(b) and clear(b) >= D.SRC_CLEAR and \
            (-100.0 <= th <= D.SRC_THETA[1])   # 09-27: far side to -100 and the -30..0 gap closed (a source at -10.8)

    # base: reachable at level 1, clear of neighbours (the fingers straddle it), in the front band (θ 0-70)
    bases = []
    for b in tops:
        th = math.degrees(math.atan2(b["xy"][1], b["xy"][0]))
        # θ ≤ 70: tower 2 (θ 81) stood right beside the parked gripper (REST hand at θ ~75-85)
        if (a.base_tag is None or b["id"] == a.base_tag) and 0 <= th <= 70 and clear(b) >= 55 and \
                D.reachable(*b["xy"], z=D.GRASP_Z + 32) and D._ik_ok(b["xy"], D.GRASP_Z + 2 + 30 + 40):
            bases.append(b)
    if a.base_xy:
        base = {"id": None, "xy": np.array(a.base_xy), "yaw": a.base_yaw, "top": True}
        bases = [base]
    if not bases:
        if photo:
            D.arm_pose("park")
        sys.exit("no suitable base block (TOP, θ 0-70, 55 mm clear, reachable at level 1)")
    if not a.base_xy:
        base = min(bases, key=lambda b: abs(math.hypot(*b["xy"]) - 190))      # mid-reach is most accurate
    srcs = [b for b in tops if b is not base and np.linalg.norm(b["xy"] - base["xy"]) > 45 and ok_src(b)
            and (a.src_tag is None or b["id"] == a.src_tag)]
    if not srcs:
        if photo:
            D.arm_pose("park")
        sys.exit("no suitable source block")
    src = min(srcs, key=lambda b: np.linalg.norm(b["xy"] - base["xy"]))
    bx, by = base["xy"]
    r, th = math.hypot(bx, by), math.degrees(math.atan2(by, bx))
    pj6 = j6_picker((bx, by, D.GRASP_Z + 2 + 30 * a.level), base["yaw"])[0]           # fingers along the base block's faces
    th_src = math.degrees(math.atan2(src["xy"][1], src["xy"][0]))
    gb = D.rt_to_xy(src["xy"], D.region_rt(th_src))
    pb = D.place_rt(th)
    cmd = [D.PY, os.path.join(HERE, "record_cycle.py"), f"stack_{stamp}", "cycle_local.py",
           f"{src['xy'][0]:.1f}", f"{src['xy'][1]:.1f}", f"{r:.1f}", f"{th:.2f}", str(a.level),
           "--side", "far" if src["xy"][1] < 0 else "near", "--picker", "--grasp-z", str(D.GRASP_Z),
           "--max-attempts", str(D.GRASP_ATTEMPTS), "--release-to", "60", "--release-speed", "10", "--square-grip-max", "33", "--jaw-xy", str(D.JAW_XY[0]), str(D.JAW_XY[1]),
           "--place-bias-rt", f"{D.PLACE_BIAS_RT[0] + pb[0]:.1f}", f"{D.PLACE_BIAS_RT[1] + pb[1]:.1f}",
           "--bias", f"{gb[0]:.1f}", f"{gb[1]:.1f}", "--yaw", f"{src['yaw']:.1f}", "--place-j6", f"{pj6:.1f}", "--wrist-look"]
    rec = {"stamp": stamp, "level": a.level, "base": {"id": base["id"], "xy": base["xy"].round(1).tolist(), "yaw": base["yaw"]},
           "src": {"id": src["id"], "xy": src["xy"].round(1).tolist(), "yaw": src["yaw"]}, "place_j6": pj6, "cmd": cmd[2:]}
    print(json.dumps({k: rec[k] for k in ("base", "src", "place_j6")}), flush=True)
    if a.level >= 2:
        # 2026-09-27 17:38: the 3-high attempt placed onto a tower that had already been knocked down (planned from a
        # 10-min-old measurement) -> the camera must see a TOP tag at the (level-1) tag height on the base right now
        cal = json.load(open(D.CALIB))
        seen = [float(np.linalg.norm(top_xy_at(b["pixel"], 14.9 + 30.0 * (a.level - 1), cal) - base["xy"]))
                for b in json.load(open(tmp))["blocks"] if b["top_face_tag"] and b["id"] not in D.FIXED_TAGS]
        rec["precheck_mm"] = round(min(seen), 1) if seen else None
        if not seen or min(seen) > 15:
            print(f"PRECHECK FAILED: no tag-up block at level {a.level - 1} on the base (nearest {rec['precheck_mm']} mm)")
            if photo:
                D.arm_pose("park")
            json.dump(rec, open(os.path.join(REPO, "data", "stack", f"stack_{stamp}.json"), "w"), indent=1)
            return 2
    if a.dry_run:
        return 0
    D.cool_parked(True)   # 18:12: the photo pose alone warmed J5 50 -> 55 (> cycle_local START_MAX_C 52)
    cat_wait("before the move")
    D.say("Building a tower.", True)
    if a.measure_inhand:
        rec.update(measured_place(cmd, rec, base, r, th, pj6, a, stamp))
    else:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=1500)
        res = [l for l in out.stdout.splitlines() if l.startswith("RESULT")]
        rec["result"] = res[-1] if res else f"exit {out.returncode}"
    print(rec["result"], flush=True)
    # camera check: a TOP tag whose ray cut at the level-1 tag height lands on the base block
    cal = json.load(open(D.CALIB))
    D.arm_pose("photo_up")
    after = D.find_blocks(tmp)
    D.arm_pose("park")
    raw = json.load(open(tmp))["blocks"]
    best = None
    for b in raw:
        if not b["top_face_tag"] or b["id"] in D.FIXED_TAGS:
            continue
        p1 = top_xy_at(b["pixel"], 14.9 + 30.0 * a.level, cal)
        d = float(np.linalg.norm(p1 - base["xy"]))
        if best is None or d < best[0]:
            best = (d, b["id"], np.round(p1, 1).tolist(), b.get("yaw_deg_mod90"))
    rec["check"] = {"level": a.level, "nearest_top_tag_at_level_mm": None if best is None else round(best[0], 1),
                    "tag": None if best is None else best[1], "xy_level1": None if best is None else best[2],
                    "yaw": None if best is None else best[3],
                    "base_still_visible": any(np.linalg.norm(b["xy"] - base["xy"]) < 12 and b["top"] for b in after)}
    print("CHECK", json.dumps(rec["check"]), flush=True)
    json.dump(rec, open(os.path.join(REPO, "data", "stack", f"stack_{stamp}.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
