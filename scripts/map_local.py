#!/usr/bin/env python3
"""Workspace map for the PICKER branch (vertical tool): where can the fingertip go, and how well does the arm get there?

--plan OUT.json  (laptop, no arm): for a grid of (r, theta) and heights, IK + joint windows + table clearance
                 (gen_movements.path_ok) for the point itself and for the straight hover -> grasp descent.
default          (ON THE JETSON, moves the arm): unpark, visit a serpentine grid of hover points with Arm.goto
                 (encoder feedback, path-checked, temperature-checked), log per point: target, measured fingertip
                 (FK of the read angles), final command offset (= sag the controller had to be pushed through),
                 iterations, servo temperatures; then park at REST and release. Stops early (safe_exit) above STOP_C.
                 Prints "POINT {...}" per point and "SUMMARY {...}" at the end (record_cycle.py collects it).

  laptop:  ~/venvs/realsense/bin/python scripts/map_local.py --plan data/perception/pickermap_<stamp>.json
  record:  ~/venvs/realsense/bin/python scripts/record_cycle.py map map_local.py --r 150 200 250 --th 0 30 60 90 120 --z 80
  hand tags: add --j6 -35 0 35 --dwell 1.5 (labelled holds "hold_r*_th*_j6*" per wrist angle)
  --face-camera LEAN: instead of the vertical tool, tilt the fingertips LEAN deg toward the RealSense (from the newest
                 camera<->arm calibration) and turn J6 so the back-of-hand face (flange (y - x) diagonal, tags 31/32) faces
                 the camera; holds at that J6 and +-30 (clipped to the window), labelled "hold_r*_th*_j6*".
"""
import argparse
import datetime
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402  (sets the joint windows)
import gen_movements as G  # noqa: E402
import kinematics as K  # noqa: E402

F.PICKER = True
STOP_C = 50          # stop the tour (safe_exit) when any servo reaches this (arm_local aborts at 60)
GRASP_Z, HOVER_Z = 13.0, 53.0   # picker grasp fingertip z (cycle_local) and its hover (grasp + 40)


def tip_floor(z):
    """Fingertip floor the real scripts use: grasp heights relax it to z - 8 (cycle_local: min(15, gz - 8))."""
    return min(G.TIP_MIN_GRASP, z - 8) if z < G.TIP_MIN else G.TIP_MIN


def feasible(r, th, z):
    """(ok, reason, q) for the fingertip at (r, th, z) on the picker branch."""
    try:
        q = F.pose(r, th, z)
    except ValueError:
        return False, "no IK", None
    if np.any(q < G.WINDOWS[:, 0]) or np.any(q > G.WINDOWS[:, 1]):
        return False, "outside windows", q
    ok, why = G.path_ok(q, q, tip_floor(z))
    return (True, "", q) if ok else (False, why, q)


def plan(out):
    rs, ths, zs = np.arange(90, 331, 10), np.arange(-105, 156, 5), [GRASP_Z, 33.0, HOVER_Z, 80.0, 120.0]
    t0, pts = time.time(), []
    for z in zs:
        for r in rs:
            for th in ths:
                ok, why, q = feasible(float(r), float(th), z)
                pts.append({"r": int(r), "th": int(th), "z": z, "ok": ok, "why": why,
                            "q": None if q is None else [round(float(v), 2) for v in q]})
    # descent hover -> grasp (what a pick actually needs)
    idx = {(p["r"], p["th"], p["z"]): p for p in pts}
    desc = []
    for r in rs:
        for th in ths:
            a, b = idx[(int(r), int(th), HOVER_Z)], idx[(int(r), int(th), GRASP_Z)]
            ok = a["ok"] and b["ok"] and G.path_ok(np.array(a["q"]), np.array(b["q"]), tip_floor(GRASP_Z))[0]
            desc.append({"r": int(r), "th": int(th), "ok": bool(ok)})
    res = {"created": datetime.datetime.now().isoformat(timespec="seconds"), "generator": "scripts/map_local.py --plan",
           "branch": "picker (far_side.PICKER, tool axis -z)", "windows": G.WINDOWS.tolist(),
           "theta_eq_J1_plus": round(float(G.REF_AZ - G.REF_J1), 2), "grasp_z": GRASP_Z, "hover_z": HOVER_Z,
           "points": pts, "descent": desc}
    json.dump(res, open(out, "w"))
    for z in zs:
        n = sum(p["ok"] for p in pts if p["z"] == z)
        print(f"z {z:5.1f}: {n}/{len(rs) * len(ths)} grid points feasible")
    print(f"descent hover z{HOVER_Z:.0f} -> grasp z{GRASP_Z:.0f}: {sum(d['ok'] for d in desc)}/{len(desc)} feasible")
    print(f"{time.time() - t0:.0f} s -> {out}")


def camera_pos():
    import glob
    cal = json.load(open(sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "cam_arm_calib_*.json")))[-1]))
    return -np.array(cal["R_base_to_cam"]).T @ np.array(cal["t_base_to_cam_mm"])


def face_pose(r, th, z, lean, C=None):
    """(score, q): fingertip at (r, th, z), tool leaning `lean` deg toward the camera, J6 turning the back-of-hand normal
    (flange (y - x)/sqrt2) toward it; score = cos(angle between that normal and the direction to the camera)."""
    C = camera_pos() if C is None else C
    tip = np.array([r * np.cos(np.radians(th)), r * np.sin(np.radians(th)), z])
    ch = (C - tip)[:2]
    ch = np.append(ch / np.linalg.norm(ch), 0.0)
    ax = np.sin(np.radians(lean)) * ch - np.cos(np.radians(lean)) * np.array([0, 0, 1.0])
    best = None
    for seed in ([th - 18, -40, -80, 0, 54, 20], [th - 18, -63, -32, 6, 3, 0], [th - 18, -20, -90, 20, 60, 0], [th - 18, -60, -40, -20, 80, 0]):
        q, err = K.ik_tool(tip, ax, seed, G.WINDOWS)
        if err > 0.5:
            continue
        sc = []
        for j6 in np.arange(G.WINDOWS[5, 0] + 1, G.WINDOWS[5, 1] - 1 + 1e-6, 1.0):
            qq = q.copy()
            qq[5] = j6
            Fl = K.fk_frames(qq)[-1]
            hand = Fl[:3, 3] + 0.5 * K.GRIPPER_LEN * Fl[:3, 2]
            n = (Fl[:3, 1] - Fl[:3, 0]) / np.sqrt(2)
            sc.append((float(n @ ((C - hand) / np.linalg.norm(C - hand))), float(j6)))
        s, j6 = max(sc)
        qq = q.copy()
        qq[5] = j6
        if G.path_ok(qq, qq, G.TIP_MIN)[0] and (best is None or s > best[0]):
            best = (s, qq)
    return best


def serpentine(rs, ths):
    out = []
    for i, r in enumerate(sorted(rs)):
        out += [(r, th) for th in (sorted(ths) if i % 2 == 0 else sorted(ths, reverse=True))]
    return out


def tour(a):
    from arm_local import Arm, Overheat  # noqa: E402  (pymycobot: Jetson only)
    S = {"start": datetime.datetime.now().isoformat(timespec="seconds"), "z": a.z, "points": [], "skipped": []}
    arm = Arm(a.log)
    t_start = time.time()
    try:
        t = arm.temps()
        S["temps_start"] = None if t is None else t.tolist()
        if t is not None and t.max() > STOP_C - 10:
            S["result"] = f"too warm to start {t.tolist()}"
            return 5
        if K.fingertip(arm.angles())[0][2] < 50:
            arm.unpark()
        for r, th in serpentine(a.r, a.th):
            x, y = r * np.cos(np.radians(th)), r * np.sin(np.radians(th))
            t0 = time.time()
            if a.face_camera is not None:
                fp = face_pose(r, th, a.z, a.face_camera)
                if fp is None or fp[0] < 0.6:
                    S["skipped"].append([r, th, "no camera-facing pose" if fp is None else f"facing score {fp[0]:.2f}"])
                    print(f"skip r{r:.0f} th{th:.0f}: {S['skipped'][-1][2]}", flush=True)
                    continue
                q0 = arm.move(fp[1], a.speed, f"map_r{r:.0f}_th{th:.0f}")
                tip, cmd = K.fingertip(q0)[0], np.array([x, y, a.z])
                j6s = sorted({float(np.clip(fp[1][5] + d, G.WINDOWS[5, 0] + 1, G.WINDOWS[5, 1] - 1)) for d in (-30, 0, 30)})
            else:
                ok, why, _ = feasible(r, th, a.z)
                if not ok:
                    S["skipped"].append([r, th, why])
                    print(f"skip r{r:.0f} th{th:.0f}: {why}", flush=True)
                    continue
                tip, cmd = arm.goto((x, y, a.z), tol=2.5, speed=a.speed, label=f"map_r{r:.0f}_th{th:.0f}")
                j6s = a.j6
            q = arm.angles()
            temps = arm.temps()
            p = {"r": r, "th": th, "target": [round(x, 1), round(y, 1), a.z], "tip": np.round(tip, 1).tolist(),
                 "err_mm": round(float(np.linalg.norm(tip - np.array([x, y, a.z]))), 1),
                 "cmd_offset": np.round(cmd - np.array([x, y, a.z]), 1).tolist(), "q": np.round(q, 2).tolist(),
                 "seconds": round(time.time() - t0, 1), "temps": None if temps is None else temps.tolist()}
            S["points"].append(p)
            print("POINT " + json.dumps(p), flush=True)
            if j6s:         # wrist sweep: show the hand tags to the camera; labelled holds for video alignment
                for j6 in j6s:
                    qh = arm.angles()
                    qh[5] = j6
                    lab = f"hold_r{r:.0f}_th{th:.0f}_j6{j6:+.0f}"
                    qh = arm.move(qh, 10, lab)
                    arm._w("hold_begin", lab, qh)
                    t1 = time.time()
                    while time.time() - t1 < a.dwell:
                        aa = arm.mc.get_angles()
                        if isinstance(aa, list) and len(aa) == 6:
                            arm._w("poll", lab, aa)
                        time.sleep(0.05)
                    arm._w("hold_end", lab, arm.angles())
            else:
                time.sleep(a.dwell)
            if temps is not None and temps.max() >= STOP_C:
                raise Overheat(f"servo temps {temps.tolist()} >= {STOP_C} C (tour stop)")
        arm.move(arm.H_near, 10, "map_home")
        arm.park_rest_from_home()
        S["result"] = "done"
        return 0
    except Overheat as e:
        S["result"] = f"overheat: {e}"
        arm.safe_exit("overheat")
        return 4
    except Exception as e:  # report, then leave the arm safe
        S["result"] = f"error: {type(e).__name__}: {e}"
        try:
            arm.safe_exit("error")
        except Exception as e2:
            S["safe_exit_error"] = f"{type(e2).__name__}: {e2}"
        return 1
    finally:
        S["seconds"] = round(time.time() - t_start, 1)
        t = arm.temps()
        S["temps_end"] = None if t is None else t.tolist()
        print("SUMMARY " + json.dumps(S), flush=True)
        arm.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", metavar="OUT.json", help="offline feasibility grid (no arm)")
    ap.add_argument("--r", nargs="+", type=float, default=[150, 200, 250])
    ap.add_argument("--th", nargs="+", type=float, default=[0, 30, 60, 90, 120])
    ap.add_argument("--z", type=float, default=80.0, help="hover fingertip z (mm above the table)")
    ap.add_argument("--speed", type=int, default=8)
    ap.add_argument("--dwell", type=float, default=1.0, help="seconds to hold each point (camera sees it settled)")
    ap.add_argument("--j6", nargs="*", type=float, default=[], help="at each point hold at these J6 values (hand-tag views)")
    ap.add_argument("--face-camera", type=float, metavar="LEAN", help="tilted holds facing the back of the hand to the camera")
    ap.add_argument("--log")
    a = ap.parse_args()
    if a.plan:
        plan(a.plan)
        return 0
    return tour(a)


if __name__ == "__main__":
    sys.exit(main())
