#!/usr/bin/env python3
"""Camera <-> arm-base calibration from a recorded calib tour (movements/calib_tour_pick_v1.json).

The arm held a tagged block (AprilTag 36h11, 20 mm) at several poses, pausing at steps labelled "hold_*".
For each hold: median joint angles (joints.csv) -> flange pose from the validated kinematics (controller frame);
RealSense color frames in that window (laptop clock, offset-corrected) -> tag detections (frames upscaled 2x for
small tags), PnP with the 20 mm size -> tag center in the camera frame (median over frames).
Tags that stay put across holds (blocks on the table) are excluded; the moving tag is the held one.

Solves jointly (alternating): camera->base rigid transform (R, t) AND the tag center's fixed offset o in the flange
frame (the block wasn't placed perfectly), minimizing || p_cam_i - (R p_base_i(o) + t) ||.
Reports per-hold residuals (mm). Writes config/cam_arm_calib_<stamp>.json.

Usage:
    ~/venvs/realsense/bin/python scripts/calibrate_cam_arm.py data/<stamp>_calib_tour_pick_v1
"""
import argparse
import csv
import datetime
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402

TAG = 0.020  # original block tags (IDs 1-4)
TAG_BY_ID = {i: 0.018 for i in (10, 11, 12, 13, 14, 15, 16, 17, 30, 31, 32, 33, 34)}  # P-touch labels, measured 18 mm (2026-09-26)
UPSCALE = 2.0
Z_TOP = 14.9   # mm, block-top plane in the controller frame (find_blocks.py)


def load_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def kabsch(A, B):
    """R, t minimizing ||R A + t - B|| (rows are points)."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def solve(pairs, iters=200):
    """Alternating: Kabsch for (R, t) given the tag offset o, then o by linear least squares."""
    Pc = np.array([p[2] for p in pairs])
    o = np.array([0, 0, K.GRIPPER_LEN])
    for _ in range(iters):
        Pb = np.array([F[:3, :3] @ o + F[:3, 3] for _, F, _, _, *_ in pairs])
        R, t = kabsch(Pb, Pc)
        A = np.vstack([R @ F[:3, :3] for _, F, _, _, *_ in pairs])
        b = np.concatenate([pc - t - R @ F[:3, 3] for (_, F, _, _, *_), pc in zip(pairs, Pc)])
        o_new = np.linalg.lstsq(A, b, rcond=None)[0]
        done = np.linalg.norm(o_new - o) < 1e-4
        o = o_new
        if done:
            break
    Pb = np.array([F[:3, :3] @ o + F[:3, 3] for _, F, _, _, *_ in pairs])
    R, t = kabsch(Pb, Pc)
    return R, t, o, np.linalg.norm((Pb @ R.T + t) - Pc, axis=1)


def look_point(F, o):
    """Place-and-look: the block was released with its tag at F o (flange frame offset o); on the table the tag centre
    keeps that xy and sits on the block-top plane z = Z_TOP."""
    p = F[:3, :3] @ o + F[:3, 3]
    return np.array([p[0], p[1], Z_TOP])


def refine_reproj(pairs, R, t, o, Kc, iters=100, looks=()):
    """Damped Gauss-Newton on x = (rotvec, t, o): pixel of R (F o_flange + p_F) + t vs the held tag's median center pixel,
    plus place-and-look pairs (F at release, tag pixel after the arm moved away) that share the same o."""
    x = np.concatenate([cv2.Rodrigues(R)[0].ravel(), t, o])

    def res(x):
        Rx = cv2.Rodrigues(x[:3].reshape(3, 1))[0]
        out = []
        for _, F, _, _, px in pairs:
            pc = Rx @ (F[:3, :3] @ x[6:9] + F[:3, 3]) + x[3:6]
            out.append(Kc[:2, :2] @ (pc[:2] / pc[2]) + Kc[:2, 2] - px)
        for _, F, px in looks:
            pc = Rx @ look_point(F, x[6:9]) + x[3:6]
            out.append(Kc[:2, :2] @ (pc[:2] / pc[2]) + Kc[:2, 2] - px)
        return np.concatenate(out)

    lam = 1e-3
    for _ in range(iters):
        r = res(x)
        J = np.array([(res(x + dx) - r) / 1e-5 for dx in np.eye(9) * 1e-5]).T
        A = J.T @ J
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -J.T @ r)
        if np.linalg.norm(res(x + step)) < np.linalg.norm(r):
            x, lam = x + step, lam / 3
        else:
            lam *= 5
        if np.linalg.norm(step) < 1e-8:
            break
    r = res(x).reshape(-1, 2)
    return cv2.Rodrigues(x[:3].reshape(3, 1))[0], x[3:6], x[6:9], np.linalg.norm(r, axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--no-looks", action="store_true", help="fit on holds only; report place-and-look points as validation")
    ap.add_argument("--reproj", action="store_true", help="refine on tag-center pixel reprojection (recommended far from the camera)")
    ap.add_argument("--static-px", type=float, default=12.0, help="px: tags this close across most holds are table tags")
    args = ap.parse_args()
    S = args.session
    meta = json.load(open(os.path.join(S, "metadata.json")))
    info = json.load(open(os.path.join(S, "realsense_info.json")))
    ci = info["color_intrinsics"]
    Kc = np.array([[ci["fx"], 0, ci["ppx"]], [0, ci["fy"], ci["ppy"]], [0, 0, 1.0]])
    dist = np.array(ci["coeffs"])
    o0, o1 = meta["clock_offset_start"]["offset_s"], meta["clock_offset_end"]["offset_s"]

    rows = load_csv(os.path.join(S, "jetson", "joints.csv"))
    for r in rows:  # arm_local.py logs the step name as "label"
        r.setdefault("step", r.get("label", ""))
    tj_all = np.array([float(r["t_host"]) for r in rows])
    to_laptop = lambda t: t - (o0 + (o1 - o0) * (t - tj_all[0]) / max(tj_all[-1] - tj_all[0], 1e-6))
    holds = {}
    for r in rows:
        if r["step"].startswith("hold_"):
            h = holds.setdefault(r["step"], {"t": [], "q": []})
            h["t"].append(float(r["t_host"]))
            if r["event"] == "poll":
                h["q"].append([float(r[f"j{k}"]) for k in range(1, 7)])
    frames = load_csv(os.path.join(S, "realsense_frames.csv"))
    tf = np.array([float(r["t_host"]) for r in frames])
    cap = cv2.VideoCapture(os.path.join(S, "realsense_color.mp4"))
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    def tag_obj(tid):
        half = TAG_BY_ID.get(int(tid), TAG) / 2
        return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)

    per_hold = {}
    for name, h in holds.items():
        if len(h["q"]) < 3:
            continue
        t0, t1 = to_laptop(min(h["t"])) + 0.4, to_laptop(max(h["t"]))
        idx = np.where((tf >= t0) & (tf <= t1))[0]
        dets = []
        for i in idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, img = cap.read()
            if not ok:
                continue
            g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
            corners, ids, _ = det.detectMarkers(g)
            if ids is None:
                continue
            for c, tid in zip(corners, ids.ravel()):
                px = c.reshape(4, 2) / UPSCALE
                ok, rvec, tvec = cv2.solvePnP(tag_obj(tid), px.astype(np.float32), Kc, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    dets.append((int(tid), px.mean(0), tvec.ravel()))
        per_hold[name] = {"q": np.median(np.array(h["q"]), 0), "dets": dets, "n_frames": len(idx)}
        print(f"{name:22s} frames {len(idx):3d}  detections {len(dets):3d}  ids {sorted({d[0] for d in dets})}")

    # Table tags sit at the same pixel in >= 2 different holds; the held tag is somewhere new at every hold.
    def is_static(name, d):
        return any(np.linalg.norm(e[1] - d[1]) < args.static_px
                   for m, v in per_hold.items() if m != name for e in v["dets"] if e[0] == d[0])
    pairs = []
    for name, v in per_hold.items():
        moving = [d for d in v["dets"] if not is_static(name, d)]
        ids = [d[0] for d in moving]
        if len(moving) < 3:
            print(f"  {name}: {len(moving)} moving-tag detections -> skipped")
            continue
        tid = max(set(ids), key=ids.count)
        mv = [d for d in moving if d[0] == tid]
        med = np.median(np.array([d[1] for d in mv]), 0)
        keep = [d for d in mv if np.linalg.norm(d[1] - med) < 15]
        if len(keep) < 3:
            print(f"  {name}: tag {tid} scattered -> skipped")
            continue
        p_cam = np.median(np.array([d[2] for d in keep]), 0) * 1000  # mm
        F = K.fk_frames(v["q"])[-1]
        pairs.append((name, F, p_cam, len(keep), med))
        print(f"  {name}: held tag id {tid}, {len(keep)} detections")
    print(f"usable holds: {len(pairs)}")
    if len(pairs) < 4:
        sys.exit("not enough usable holds (need >= 4)")

    R, t, o, res = solve(pairs)
    loo = []
    for i in range(len(pairs)):
        Ri, ti, oi, _ = solve(pairs[:i] + pairs[i + 1:])
        _, F, pc, _, _ = pairs[i]
        loo.append(float(np.linalg.norm(Ri @ (F[:3, :3] @ oi + F[:3, 3]) + ti - pc)))
    for (name, _, pc, n, _px), r in zip(pairs, res):
        print(f"  {name:22s} n={n:3d}  residual {r:6.1f} mm   cam {np.round(pc, 0).tolist()}")
    print(f"leave-one-out prediction errors (mm): {[round(e, 1) for e in loo]}  "
          f"RMS {np.sqrt(np.mean(np.square(loo))):.1f}, max {max(loo):.1f}")
    print(f"RMS {np.sqrt((res ** 2).mean()):.1f} mm, max {res.max():.1f} mm; tag offset in flange frame {np.round(o, 1).tolist()} mm "
          f"(|o| {np.linalg.norm(o):.0f} mm; gripper length {K.GRIPPER_LEN:.0f})")
    method = "alternating Kabsch + linear tag offset; PnP 20 mm AprilTag, frames upscaled 2x"
    # place-and-look windows (tour_local.py --placelook): 'put' event (label put_i) + look_i hold window
    looks = []
    puts = {r["step"]: np.array([float(r[f"j{k}"]) for k in range(1, 7)]) for r in rows if r["event"] == "put"}
    for name, qp in sorted(puts.items()):
        i = name.split("_")[1]
        win = [float(r["t_host"]) for r in rows if r["step"] == f"look_{i}" and r["event"] in ("hold_begin", "hold_end")]
        if len(win) < 2:
            continue
        Fp = K.fk_frames(qp)[-1]
        pred_c = R @ look_point(Fp, o) + t
        pred = Kc[:2, :2] @ (pred_c[:2] / pred_c[2]) + Kc[:2, 2]
        idx = np.where((tf >= to_laptop(win[0]) + 0.3) & (tf <= to_laptop(win[1])))[0]
        pxs = []
        for fi in idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(fi))
            ok, img = cap.read()
            if not ok:
                continue
            g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
            corners, ids, _ = det.detectMarkers(g)
            if ids is None:
                continue
            cands = [c.reshape(4, 2).mean(0) / UPSCALE for c in corners]
            dists = [np.linalg.norm(c - pred) for c in cands]
            if min(dists) < 120:
                pxs.append(cands[int(np.argmin(dists))])
        if len(pxs) >= 3:
            px = np.median(np.array(pxs), 0)
            looks.append((f"look_{i}", Fp, px))
            print(f"  look_{i}: {len(pxs)} detections, pixel {np.round(px, 1).tolist()}, {np.linalg.norm(px - pred):.0f} px from the pre-fit prediction")
        else:
            print(f"  look_{i}: no tag near the prediction ({len(pxs)} detections) -> skipped")
    if args.reproj:
        # Refine on PIXELS (tag-center reprojection): PnP range noise (~+-2 cm at 1 m) no longer enters.
        R, t, o, px_res = refine_reproj(pairs, R, t, o, Kc, looks=[] if args.no_looks else looks)
        if looks and args.no_looks:
            for nm, Fp, px in looks:
                pc = R @ look_point(Fp, o) + t
                e = np.linalg.norm(Kc[:2, :2] @ (pc[:2] / pc[2]) + Kc[:2, 2] - px)
                print(f"  VALIDATION {nm}: {e:5.1f} px (~{e * pc[2] / Kc[0, 0]:.0f} mm lateral)")
        elif looks:
            _, _, _, allres = refine_reproj(pairs, R, t, o, Kc, iters=0, looks=looks)
            for (nm, *_), r in zip(looks, allres[len(pairs):]):
                print(f"  reproj {nm:22s} {r:5.1f} px")
            px_res = px_res[:len(pairs)]
        loo_px, loo_mm = [], []
        for i in range(len(pairs)):
            Ri, ti, oi, _ = refine_reproj(pairs[:i] + pairs[i + 1:], R, t, o, Kc)
            _, F, _, _, px = pairs[i]
            pc = Ri @ (F[:3, :3] @ oi + F[:3, 3]) + ti
            e = float(np.linalg.norm(Kc[:2, :2] @ (pc[:2] / pc[2]) + Kc[:2, 2] - px))
            loo_px.append(e)
            loo_mm.append(e * pc[2] / Kc[0, 0])
        for (name, *_), r in zip(pairs, px_res):
            print(f"  reproj {name:22s} {r:5.1f} px")
        res = np.array([r * (R @ (p[1][:3, :3] @ o + p[1][:3, 3]) + t)[2] / Kc[0, 0] for p, r in zip(pairs, px_res)])
        loo = loo_mm
        print(f"REPROJ: RMS {np.sqrt(np.mean(np.square(px_res))):.2f} px ({np.sqrt((res ** 2).mean()):.1f} mm lateral); "
              f"LOO {np.round(loo_px, 1).tolist()} px -> RMS {np.sqrt(np.mean(np.square(loo_mm))):.1f} mm, max {max(loo_mm):.1f} mm; "
              f"tag offset {np.round(o, 1).tolist()}")
        method = "Kabsch init, then tag-center REPROJECTION refinement (R, t, tag offset); errors in mm are lateral (px * Z / f)"
    cam_in_base = -R.T @ t
    print(f"camera position in the arm-base frame: {np.round(cam_in_base, 0).tolist()} mm")
    out = {"created": datetime.datetime.now().isoformat(timespec="seconds"), "session": os.path.basename(S.rstrip('/')),
           "method": method,
           "R_base_to_cam": R.tolist(), "t_base_to_cam_mm": t.tolist(), "camera_in_base_mm": cam_in_base.tolist(),
           "tag_offset_flange_mm": o.tolist(), "rms_mm": float(np.sqrt((res ** 2).mean())), "max_mm": float(res.max()),
           "residuals_mm": {p[0]: float(r) for p, r in zip(pairs, res)},
           "loo_errors_mm": {p[0]: e for p, e in zip(pairs, loo)}, "loo_rms_mm": float(np.sqrt(np.mean(np.square(loo)))), "n_holds": len(pairs),
           "intrinsics": ci, "note": "p_cam = R p_base + t (mm); controller (get_coords) frame for base"}
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config",
                        f"cam_arm_calib_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    json.dump(out, open(path, "w"), indent=1)
    print(f"saved {os.path.relpath(path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
