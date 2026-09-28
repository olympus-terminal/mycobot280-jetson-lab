#!/usr/bin/env python3
"""Expo / promo demo loop (LAPTOP): keep picking up random blocks and putting them down at random free spots, with
cooling breaks. Every move is one recorded session (record_cycle.py -> cycle_local.py --picker), so the demo also
produces real kinematics + video data.

Per cycle:
  1. cat guard check (cat_guard.py check) - wait while something is in the work volume
  2. servo temps: start only if J5 <= START_J5 (and all <= START_ALL); otherwise a cooling break until <= COOL_J5
  3. find_blocks.py (RealSense, near-side calibration) -> blocks (tag ray-plane positions; TOP tags preferred)
  4. source = random reachable block with >= SRC_CLEAR mm to every other block
     destination = random point in the visible reachable zones with >= DST_CLEAR mm to every block (and the source's
     old spot excluded), and >= MIN_MOVE mm from the source
  5. record_cycle.py demo_<n> cycle_local.py SRC DST --picker --grasp-z 14 [--yaw] --max-attempts 2 --release-to 55
  6. result logged to data/demo/demo_<stamp>.jsonl
Stop: create /tmp/demo_stop (finishes the current move, the arm is parked after every move), or --max-cycles, or
MAX_FAILS failures in a row.

Usage: ~/venvs/realsense/bin/python scripts/demo_loop.py [--dry-run] [--max-cycles N] [--voice] [--seed S]
--dry-run: camera + planning only, prints the moves, never moves the arm.
"""
import argparse
import datetime
import json
import math
import os
import random
import re
import subprocess
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..")
sys.modules.setdefault("pymycobot", types.SimpleNamespace(MyCobot280=None))  # planning only: no serial on the laptop
sys.path.insert(0, HERE)
import far_side as F  # noqa: E402

F.PICKER = True
from arm_local import j6_picker, pose_xyz  # noqa: E402
from cycle_local import jaw_world  # noqa: E402

PY = os.path.expanduser("~/venvs/realsense/bin/python")
CALIB = os.path.join(REPO, "config", "cam_arm_calib_20260927_170528_rotfix2.json")   # 2026-09-27 17:05: + 0.111 deg after a cat bumped the tripod (was 20260926_071021_rotfix)
JETSON = os.environ.get("JETSON_HOST", "nvidia@jetson-orin.local")   # ssh target of the Jetson (set JETSON_HOST)
STOP_FILE = "/tmp/demo_stop"
START_J5, START_ALL, COOL_J5 = 50, 52, 46   # 2026-09-27: was 46, 50, 41 (user: can go hotter). J5 +10 C per move (max 15) -> ends 56-60; ABORT_C 62, servo cutoff 70
SRC_CLEAR, DST_CLEAR, MIN_MOVE = 45.0, 55.0, 60.0
SRC_THETA = (0.0, 124.0)     # 2026-09-27 09:35: left front re-enabled with the region correction (region_rt)
SRC_THETA_FAR = (-90.0, 0.0)     # 09-27 18:10: was -30 (a -30..0 gap: blocks at θ -11 were never used); 2026-09-26 20:05: far-side sources (picker hover map 1.9 mm at θ -60/-30; camera
                                 # calibration extrapolated there) -> cycle_local --side far; they refill the near side
FIXED_TAGS = {30, 31, 32, 33, 34}   # 30: base plate (camera-pose check); 31-34: on the gripper (2026-09-26) - never blocks
MAX_FAILS = 3
DROP_Z = 110.0      # dice roll: tip height at the release (IK ok over ~90 % of the zones at 110-120)
FAR_GRASP_R = 0.0   # mm radial for far-side grasps (θ < 0): 12:10 -5; 12:35 -> 0 (12:08 and 12:31 held at net +3, 12:00 missed at +5; camera scatter there ±5 mm)
GRASP_R_ALL = 5.0   # mm radial (user: "maybe 5 mm"; was 8), added to every grasp aim (2026-09-27 11:50, user: fingers on the edge closest to the base)
PLACE_J6_MAX = 30.0  # deg: random J6 at the place (the block is set down rotated; variation, 2026-09-27 12:25). 0 = off
PHOTO_POSE = True   # 2026-09-27: look with the arm straight up (the parked arm hid several blocks)
VERIFY_R = 45.0     # a TOP tag within this of the slot target = placement verified (2026-09-27)
ROLL_CLEAR = 75.0   # a dropped block bounces: keep more room to the other blocks
GRASP_ATTEMPTS = 5   # 2026-09-26: 2 -> 5 (cands (0,0),(8,0),(-8,0),(0,12),(0,-12)): which offset catches the block at θ ~92 = the error direction
GRASP_Z = 5.0    # 2026-09-26 20:18: z 13 closed ABOVE the tag-1/2 blocks (0/5 since 15:00; depth: ~2-5 mm shorter than tag 4); z 5 on tag 2 -> held (29) first attempt
PLACE_BIAS_RT = (0.0, -6.0)    # 2026-09-27 11:55: 16 -> 0 outside the left front (run 13: RF landed +31 mm, far +41 mm radially
                               # outward, drifting blocks out of reach); the left front keeps +10 via place_rt()
PLACE_BIAS_RT_OLD = (29.0, -6.0)   # 2026-09-26: blocks landed ~29 mm radially inward, ~+6 tangential of the aim (2 TOP-tag checks)
JAW_XY = (7.0, 18.0)   # jaw centre in the flange frame (20:12 guided grasp)
KNOWN_R = 25.0   # a detection within this of an arm-placed spot IS that block (use the arm's own position)
# destination zones (r range, theta range): reachable by the picker AND visible to the RealSense with the arm parked
ZONES = [((175, 200), (78, 100)), ((175, 225), (12, 28)),
         ((140, 175), (75, 100)), ((175, 225), (-85, -60))]   # 11:55: r <= 225 so a +30 mm landing stays pickable (<= 237); 12:00: θ 78-100 only to r 200 (the parked gripper rests beside r ~210-260 there: hides the tag, collision risk); 13:20: RF band θ 12-28 (θ 32-35 at r > 200 is under the parked forearm in the camera view)
# 2026-09-27 10:05: inner right-front band (r 140-175, θ 12-35) dropped: blocks placed there are hidden from the camera by
# the parked (REST) forearm -> the placement check can't see them (run 12 c2: placed on target per the video, "not found")
# 2026-09-27: + inner band, + far side. The inner band skips θ 35-75: at REST (released after every move) the forearm
# lies low there (elbow z ~33 mm at r 184 θ 44, wrist r 247-254 θ 50-61)


CLIPS = {"Cooling break.": "cooling", "Demo stopping.": "stopping", "I need the blocks moved closer.": "closer",
         "Three misses in a row. Pausing the demo.": "misses"}


def say(text, voice):
    print(f"[voice] {text}", flush=True)
    if voice:
        import record_session as RS
        RS.play(CLIPS.get(text, "_none"), text)


def temps():
    cmd = ("~/venvs/mycobot/bin/python -c \"from pymycobot import MyCobot280; "
           "mc=MyCobot280('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0','1000000'); print(mc.get_servo_temps())\"")
    r = subprocess.run(["ssh", "-o", "LogLevel=ERROR", JETSON, cmd], capture_output=True, text=True, timeout=60)
    m = re.search(r"\[[\d, ]+\]", r.stdout)
    return json.loads(m.group(0)) if m else None


def reachable(x, y, z=GRASP_Z, margin=25.0):
    """Reachable with room for the jaw offset / placement correction (~20-30 mm): also check margin mm outward."""
    r = math.hypot(x, y)
    if r + margin > 262:
        return False
    try:
        for zz in (z, z + 20, z + 40, z + 60):
            q = pose_xyz(np.array([x, y, zz]))
            if not (F.W[0][0] <= q[0] <= F.W[0][1]):
                return False
        return True
    except ValueError:
        return False


def slot_reachable(d, place_j6=None):
    """The tip target cycle_local will actually aim at (place bias + jaw offset, as in cycle_local) must have picker IK
    at every place height up to the hover (2026-09-26 cycle 1: r 233 destination -> tip target r 277, no IK at z 80)."""
    us = d / np.linalg.norm(d)
    slot = d + PLACE_BIAS_RT[0] * us + PLACE_BIAS_RT[1] * np.array([-us[1], us[0]])
    sl = slot.copy()
    try:
        for _ in range(2):
            slot = sl - jaw_world(slot, 30.0, place_j6, JAW_XY)
    except ValueError:
        return False
    return reachable(slot[0], slot[1], z=GRASP_Z + 2, margin=5.0) and \
        all(_ik_ok(slot, zz) for zz in (GRASP_Z + 2, GRASP_Z + 14, 80.0))


def grasp_reachable(b):
    """The jaw-adjusted tip aim cycle_local will use for the grasp must be inside every joint window (with the chosen J6)
    at the grasp, mid and hover heights (2026-09-26 run 2 cycle 2: block at θ 116 -> aim θ 121 -> J1 140.3 > 140)."""
    blk = np.asarray(b["xy"], float)
    yaw = b["yaw"] if (b["top"] and b["yaw"] is not None) else None
    try:
        g = blk.copy()
        for _ in range(3):
            j6 = j6_picker((g[0], g[1], GRASP_Z), yaw)[0] if yaw is not None else None
            g = blk - jaw_world(g, GRASP_Z, j6, JAW_XY)
        for zz in (GRASP_Z, GRASP_Z + 20, 80.0):
            q = pose_xyz(np.array([g[0], g[1], zz]))
            if j6 is not None:
                q[5] = j6
            if not all(F.W[i][0] + 2 <= q[i] <= F.W[i][1] - 2 for i in range(6)):
                return False
        return True
    except ValueError:
        return False


def _ik_ok(xy, z):
    try:
        pose_xyz(np.array([xy[0], xy[1], z]))
        return True
    except ValueError:
        return False


def region_rt(th):
    """2026-09-27: the hand's real position vs the model differs on the LEFT FRONT (θ >= 75): grasps there caught the block
    only with the aim moved -8 mm radial / -12 mm tangential (regrasp_LF_bias2, 09:33, after 0/7 uncorrected since 09-26
    15:00), and place-and-look landed +9..19 mm radial / +9..15 mm tangential there (vs +13 / -2 on the right front).
    Returns the (radial, tangential) aim correction in mm: 0 up to θ 40, full from θ 75, linear in between.
    10:45 (user, run 12 c11 at θ 89: fingers closed on the block's edge facing the base / the RealSense camera; θ 108 c4:
    edge grip only after +8 radial) -> less inward, more tangential: (-8, -12) -> (-6, -16).
    11:50 (user screenshot docs/photos/grasp_near_edge_user_20260927_1147.png: still gripping the edge CLOSEST TO THE BASE,
    run 13) -> + GRASP_R_ALL radial everywhere (the jaw-offset model is ~5 mm short radially, not only on the left front).
    12:10: NOT on the far side (θ < 0): run 14 c2 at θ -68 with +5 missed 5/5 (closest: net -3 radial, value 11); earlier
    far-side grasps held at -8 (09:50) and 0 (11:32) -> -5 there."""
    if th < 0:
        return FAR_GRASP_R, 0.0
    w = min(max((th - 40.0) / 35.0, 0.0), 1.0)
    return GRASP_R_ALL - 6.0 * w, -16.0 * w


def place_rt(th):
    """Extra place aim correction (radial, tangential) on top of PLACE_BIAS_RT: on the left front +10 radial (its landings
    were +17/+3/-2 mm with a net +10) and the region tangential term; 0 elsewhere (2026-09-27 11:55)."""
    w = min(max((th - 40.0) / 35.0, 0.0), 1.0) if th >= 0 else 0.0
    return 10.0 * w, region_rt(th)[1]


def rt_to_xy(xy, rt):
    u = np.asarray(xy, float) / np.linalg.norm(xy)
    return rt[0] * u + rt[1] * np.array([-u[1], u[0]])


def arm_pose(name):
    """2026-09-27: named pose on the Jetson (scripts/arm_pose.py): photo_up (arm straight up, out of the camera's way),
    home, park. Returns True on success."""
    r = subprocess.run(["ssh", "-o", "LogLevel=ERROR", JETSON,
                        f"cd ~/orin-mycobot && ~/venvs/mycobot/bin/python scripts/arm_pose.py {name}"],
                       capture_output=True, text=True, timeout=120)
    ok = r.returncode == 0 and "POSE" in r.stdout
    if not ok:
        print(f"arm_pose {name} FAILED: {r.stdout[-300:]} {r.stderr[-300:]}", flush=True)
    return ok


def cool_parked(voice=False):
    """2026-09-27: park, then wait until J5 <= START_J5 and all <= START_ALL (the upright photo pose alone warms J5 ~5 C;
    holding it upright once took J5 to 70 C)."""
    arm_pose("park")
    t = temps()
    said = False
    while t is not None and (t[4] > START_J5 or max(t) > START_ALL) and not os.path.exists(STOP_FILE):
        if not said:
            say("Cooling break.", voice)
            said = True
        print(f"  cooling (parked): {t}", flush=True)
        time.sleep(45)
        t = temps()
    return t


def find_blocks(out_json, run=True):
    if run:   # run=False: re-read a saved scan (offline planning)
        subprocess.run([PY, os.path.join(HERE, "find_blocks.py"), "--frames", "20", "--calib", CALIB, "--json", out_json],
                       capture_output=True, text=True, timeout=180)
    d = json.load(open(out_json))
    # "top" only when the tag clearly faces up (n_z > 0.98) in >= 10 of 20 frames (09-27 18:15: was 15; a tag beside the
    # base read 13/20 behind cables; the IPPE fix makes n_z the real side/top test). A side tag's ray-plane position is ~15 mm off, enough to
    # land a finger on the block top (16:03-16:10 demo failures)
    return [{"xy": np.array(b["plane_xy_mm"]), "yaw": b["yaw_deg_mod90"], "id": b["id"],
             "top": bool(b["top_face_tag"] and b["face_normal"][2] > 0.98 and b["n_frames"] >= 10),
             # roll only a real side face (vertical: |n_z| small); n_z ~0.5 was an IPPE-flipped TOP tag (2026-09-26)
             "side_xy": None if (b.get("side_center_xy_mm") is None or abs(b["face_normal"][2]) > 0.5)
             else np.array(b["side_center_xy_mm"]),
             "side_yaw": b.get("side_yaw_deg_mod90")}
            for b in d["blocks"] if b["id"] not in FIXED_TAGS]


def merge_known(blocks, known):
    """Blocks the arm placed itself: replace the camera estimate by the arm's own placement (self-consistent to a few mm)
    and mark them graspable (J6 as placed). known: list of xy the arm placed (latest last)."""
    for b in blocks:
        for k in known[::-1]:
            if np.linalg.norm(b["xy"] - k) < KNOWN_R + 15:
                # 2026-09-25 20:45: placements land 20-35 mm from the aim (the fingers centre the block only along
                # the closing axis) -> never grasp from the remembered aim; the camera alone decides. Only log the error.
                if b["top"]:
                    b["known_err"] = (b["xy"] - np.array(k)).round(1).tolist()
                break
    return blocks


def plan(blocks, rng):
    pts = [b["xy"] for b in blocks]
    srcs = []
    for i, b in enumerate(blocks):
        x, y = b["xy"]
        th = math.degrees(math.atan2(y, x))
        if not (SRC_THETA[0] <= th <= SRC_THETA[1] or SRC_THETA_FAR[0] <= th <= SRC_THETA_FAR[1]) or not reachable(x, y):
            continue
        if not grasp_reachable(b):
            continue
        clear = min([np.linalg.norm(b["xy"] - p) for j, p in enumerate(pts) if j != i] or [999])
        if clear >= SRC_CLEAR:
            srcs.append((b, clear))
    tops = [s for s in srcs if s[0]["top"]]
    mode = "move"
    if not tops:
        # dice roll (2026-09-26, user): no tag up -> pick a SIDE-tag block (position/yaw from the side tag), drop it
        rolls = []
        for i, b in enumerate(blocks):
            if b["top"] or b["side_xy"] is None:
                continue
            x, y = b["side_xy"]
            th = math.degrees(math.atan2(y, x))
            if not (SRC_THETA[0] <= th <= SRC_THETA[1]) or not reachable(x, y):
                continue
            rb = {"xy": b["side_xy"], "yaw": b["side_yaw"], "top": True, "id": b["id"]}   # top=True: use the side yaw
            if not grasp_reachable(rb):
                continue
            clear = min([np.linalg.norm(b["side_xy"] - p) for j, p in enumerate(pts) if j != i] or [999])
            if clear >= SRC_CLEAR:
                rolls.append(({**b, "xy": b["side_xy"], "yaw": b["side_yaw"], "roll": True}, clear))
        if not rolls:
            return None, ("no reachable block with clearance" if not srcs else "no up-facing tag") + " and no side-tag block to roll"
        tops, mode = rolls, "roll"
    b, _ = rng.choice(tops)
    b = {**b, "mode": mode}
    for _ in range(400):
        (r0, r1), (t0, t1) = rng.choice(ZONES)
        r, th = rng.uniform(r0, r1), rng.uniform(t0, t1)
        d = np.array([r * math.cos(math.radians(th)), r * math.sin(math.radians(th))])
        if np.linalg.norm(d - b["xy"]) < MIN_MOVE:
            continue
        if min(np.linalg.norm(d - p) for p in pts) < DST_CLEAR:
            continue
        pj6 = round(rng.uniform(-PLACE_J6_MAX, PLACE_J6_MAX), 1) if PLACE_J6_MAX > 0 else None
        if not reachable(*d, z=GRASP_Z + 2) or not slot_reachable(d, pj6):
            continue
        b = {**b, "place_j6": pj6}
        if mode == "roll" and (min(np.linalg.norm(d - p) for p in pts) < ROLL_CLEAR or not _ik_ok(d, DROP_Z + 10)):
            continue
        return (b, d, r, th), "ok"
    return None, "no free destination"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--max-cycles", type=int, default=1000)
    ap.add_argument("--voice", action="store_true")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--known", nargs="*", type=float, default=[], help="X1 Y1 ...: spots where the arm already put blocks")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(REPO, "data", "demo"), exist_ok=True)
    log_path = os.path.join(REPO, "data", "demo", f"demo_{stamp}.jsonl")
    tmp = f"/tmp/demo_blocks_{stamp}.json"
    print(f"demo_loop {stamp}  dry_run={a.dry_run}  log {os.path.relpath(log_path, REPO)}", flush=True)
    fails, n = 0, 0
    known = [list(p) for p in zip(a.known[0::2], a.known[1::2])]   # xy where the arm put blocks down
    last_drop = None
    last_place = None   # slot target of the last "placed" cycle (camera-verified on the next pass)
    while n < a.max_cycles:
        if os.path.exists(STOP_FILE):
            say("Demo stopping.", a.voice)
            break
        rec = {"cycle": n, "t": datetime.datetime.now().isoformat(timespec="seconds")}
        if not a.dry_run:
            # SILENT check (the user: the cat guard's own alarm voice is too loud); wait up to 5 min for a clear view
            # 2026-09-26: never move on a timeout (unattended runs): wait until the view is clear or the stop file appears
            t_cg = time.time()
            while True:
                g = subprocess.run([PY, os.path.join(REPO, "cat_guard", "cat_guard.py"), "check"], capture_output=True, text=True)
                if g.returncode == 0 or os.path.exists(STOP_FILE):
                    break
                if time.time() - t_cg > 300:
                    print(f"cat guard not clear for {time.time() - t_cg:.0f} s; still waiting", flush=True)
                    t_cg = time.time()
                time.sleep(10)
            if os.path.exists(STOP_FILE):
                continue
            rec["cat_guard"] = g.stdout.strip().splitlines()[-1] if g.stdout.strip() else g.returncode
            t = temps()
            rec["temps"] = t
            if t is None:
                print("no temperature reading; retry in 30 s", flush=True)
                time.sleep(30)
                continue
            if t[4] > START_J5 or max(t) > START_ALL:
                say("Cooling break.", a.voice)
                while t is not None and (t[4] > COOL_J5 or max(t) > START_ALL) and not os.path.exists(STOP_FILE):
                    time.sleep(60)
                    t = temps()
                    print(f"  cooling: {t}", flush=True)
                continue
        photo = not a.dry_run and PHOTO_POSE
        if photo:
            subprocess.run(["rsync", "-a", "--delete", "scripts", f"{JETSON}:orin-mycobot/"], cwd=REPO, capture_output=True)
            if not arm_pose("photo_up"):
                arm_pose("park")
                break
        blocks = merge_known(find_blocks(tmp), known)
        rec["placement_errors"] = [b["known_err"] for b in blocks if "known_err" in b]   # camera (TOP tag) - arm aim
        if last_place is not None:   # 2026-09-27: "placed" only means the gripper opened at the slot -> camera check
            ok_v = [bb for bb in blocks if bb["top"] and np.linalg.norm(bb["xy"] - last_place) < VERIFY_R]
            rec["verify_prev"] = {"target": np.round(last_place, 1).tolist(),
                                  "verified": bool(ok_v),
                                  "error_mm": round(float(min(np.linalg.norm(bb["xy"] - last_place) for bb in ok_v)), 1) if ok_v else None}
            print(f"  verify previous placement: {'OK, ' + str(rec['verify_prev']['error_mm']) + ' mm' if ok_v else 'NOT FOUND (counted as a miss)'}", flush=True)
            if not ok_v:
                fails += 1
            last_place = None
        if last_drop is not None:   # dice-roll outcome: what the camera sees near the last drop spot
            near = [bb for bb in blocks if np.linalg.norm((bb["xy"] if bb["top"] or bb["side_xy"] is None else bb["side_xy"]) - last_drop) < 70]
            rec["roll_outcome"] = "tag up" if any(bb["top"] for bb in near) else ("tag on a side" if near else "no tag seen (picture up, rolled away, or occluded)")
            print(f"  last roll: {rec['roll_outcome']}", flush=True)
            last_drop = None
        p, why = plan(blocks, rng)
        rec["n_blocks"] = len(blocks)
        if photo:
            cool_parked(a.voice)   # park (always) and cool; the cycle unparks from REST
        if p is None:
            rec["result"] = why
            print(f"cycle {n}: {why} ({len(blocks)} blocks seen)", flush=True)
            json.dump(rec, open(log_path, "a"))
            open(log_path, "a").write("\n")
            fails += 1
            if fails >= MAX_FAILS:
                say("I need the blocks moved closer.", a.voice)
                break
            time.sleep(20)
            continue
        b, d, r, th = p
        rec.update({"src": b["xy"].round(1).tolist(), "src_tag": b["id"], "src_top": b["top"], "yaw": b["yaw"],
                    "dst": d.round(1).tolist(), "mode": b["mode"]})
        print(f"cycle {n}: {b['mode'].upper()} tag {b['id']} {'TOP' if b['top'] else 'SIDE'} {b['xy'].round(1).tolist()} -> {d.round(1).tolist()} "
              f"(r {r:.0f}, th {th:.0f})", flush=True)
        if a.dry_run:
            rec["result"] = "dry-run"
        else:
            cmd = [PY, os.path.join(HERE, "record_cycle.py"), f"demo_{stamp}_{n:03d}", "cycle_local.py",
                   f"{b['xy'][0]:.1f}", f"{b['xy'][1]:.1f}", f"{r:.1f}", f"{th:.2f}", "0", "--side", "far" if b["xy"][1] < 0 else "near", "--picker",
                   "--grasp-z", str(GRASP_Z), "--max-attempts", str(GRASP_ATTEMPTS), "--release-to", "55",
                   "--jaw-xy", str(JAW_XY[0]), str(JAW_XY[1]),
                   "--place-bias-rt", str(PLACE_BIAS_RT[0]), str(PLACE_BIAS_RT[1]), "--wrist-look"]
            if b.get("place_j6") is not None:
                cmd += ["--place-j6", str(b["place_j6"])]
                rec["place_j6"] = b["place_j6"]
            th_src = math.degrees(math.atan2(b["xy"][1], b["xy"][0]))
            gb = rt_to_xy(b["xy"], region_rt(th_src))
            pb = place_rt(th)
            cmd += ["--bias", f"{gb[0]:.1f}", f"{gb[1]:.1f}"]
            cmd[cmd.index("--place-bias-rt") + 1: cmd.index("--place-bias-rt") + 3] = [f"{PLACE_BIAS_RT[0] + pb[0]:.1f}", f"{PLACE_BIAS_RT[1] + pb[1]:.1f}"]
            rec["grasp_bias_xy"], rec["place_bias_rt"] = np.round(gb, 1).tolist(), [PLACE_BIAS_RT[0] + pb[0], PLACE_BIAS_RT[1] + pb[1]]
            if (b["top"] or b.get("roll")) and b["yaw"] is not None and not b.get("known"):
                cmd += ["--yaw", f"{b['yaw']:.1f}"]
            if b["mode"] == "roll":
                cmd += ["--drop-z", str(DROP_Z)]
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=1500)
            res = [l for l in out.stdout.splitlines() if l.startswith("RESULT")]
            rec["result"] = res[-1] if res else f"exit {out.returncode}"
            sess = [l for l in out.stdout.splitlines() if l.startswith("session ")]
            rec["session"] = sess[-1][8:] if sess else None
            print(f"  {rec['result']}", flush=True)
            if "placed" in rec["result"]:
                fails = 0
                last_place = d.copy()
                known = [k for k in known if np.linalg.norm(np.array(k) - b["xy"]) > KNOWN_R] + [d.tolist()]
            elif "dropped" in rec["result"]:
                fails = 0
                last_drop = d.copy()
            else:
                fails += 1
            if fails >= MAX_FAILS:
                say("Three misses in a row. Pausing the demo.", a.voice)
                json.dump(rec, open(log_path, "a"))
                open(log_path, "a").write("\n")
                break
        with open(log_path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        n += 1
        if a.dry_run and n >= a.max_cycles:
            break
    print("demo_loop done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
