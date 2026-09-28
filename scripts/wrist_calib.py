#!/usr/bin/env python3
"""Calibrate the WRIST camera (intrinsics f, cx, cy, k1 + hand-eye T_cam<-flange) from a wrist scan, using blocks whose
arm-frame positions are known (placed by the arm; camera-verified) as targets.

Targets: tag id -> (center xy arm frame, tag rotation in the arm frame from the RealSense PnP, top z = 14.9).
Observations: each wrist frame's detections of those ids (4 corners each), with the measured joint angles -> flange pose.
Model: pixel = distort(K * T_cam_flange * T_flange_arm(q) * X). Solved by damped Gauss-Newton (numeric Jacobian),
initialized from per-view PnP with f=500. Outliers (> 8 px mean) dropped and re-solved. Writes config/wrist_cam_<stamp>.json.

Usage: ~/venvs/realsense/bin/python scripts/wrist_calib.py SCAN_DIR TARGETS.json
TARGETS.json: {"1": {"center": [x, y], "R_arm": [[3x3]]}, "3": {...}}
  or, per target, "yaw_deg" (RealSense top-tag yaw mod 90) instead of "R_arm": the 4 rotations yaw + k*90 are all tried
  and the lowest-RMS one is kept. Optional "files": [...] restricts a target to those scan frames (tag IDs repeat
  across blocks, so a target is only valid in frames where that block is the one carrying the ID).
"""
import datetime
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402

Z_TOP = 14.9
H = 10.0  # half tag size mm
CORNERS = np.array([[-H, H, 0], [H, H, 0], [H, -H, 0], [-H, -H, 0]])


def rodrigues(v):
    return cv2.Rodrigues(np.asarray(v, float).reshape(3, 1))[0]


def project(Pc, p):
    f, cx, cy, k1 = p
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
    r2 = x * x + y * y
    d = 1 + k1 * r2
    return np.stack([f * x * d + cx, f * y * d + cy], 1)


def main():
    scan, tfile = sys.argv[1], sys.argv[2]
    F_PRIOR = float(sys.argv[3]) if len(sys.argv) > 3 else 520.0   # 640 px webcam, ~63 deg HFOV
    PRIOR_W = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5  # prior weight (px-equivalent per unit deviation / sigma); keeps f, cx, cy, k1 physical
    targets = {int(k): v for k, v in json.load(open(tfile)).items()}
    yaw_ids = [tid for tid, v in targets.items() if "R_arm" not in v]
    if len(yaw_ids) > 1:
        sys.exit("only one yaw_deg target supported (4 rotations are tried for it)")
    best = None
    for k90 in (range(4) if yaw_ids else [0]):
        res = calibrate(scan, targets, yaw_ids, k90, F_PRIOR, PRIOR_W)
        if res is not None:
            print(f"  yaw option k={k90}: RMS {res['rms_px']:.2f} px, f {res['f']:.0f}")
            if best is None or res["rms_px"] < best["rms_px"]:
                best = res
    if best is None:
        sys.exit("not enough observations")
    out = best
    print(f"intrinsics f {out['f']:.1f} px  cx {out['cx']:.1f} cy {out['cy']:.1f} k1 {out['k1']:.3f}; hand-eye t_cam<-flange "
          f"{np.round(out['t_cam_flange_mm'], 1).tolist()} mm")
    print(f"reprojection RMS {out['rms_px']:.2f} px over {out['n_views']} views ({out['n_views'] * 4} corners); dropped {out['dropped']}")
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", f"wrist_cam_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    json.dump(out, open(path, "w"), indent=1)
    print("saved", os.path.relpath(path))
    return 0


def calibrate(scan, targets, yaw_ids, k90, F_PRIOR, PRIOR_W):
    tgt3d = {}
    for tid, v in targets.items():
        if tid in yaw_ids:
            a = np.radians(v["yaw_deg"] + 90 * k90)
            R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
        else:
            R = np.array(v["R_arm"])
        c = np.array([v["center"][0], v["center"][1], Z_TOP])
        tgt3d[tid] = c + (R @ CORNERS.T).T
    sc = json.load(open(os.path.join(scan, "scan.json")))
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    obs = []  # (flange T (4x4), X (4x3), uv (4x2), file)
    for r in sc["records"]:
        im = cv2.imread(os.path.join(scan, r["file"]))
        g = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
        c, ids, _ = det.detectMarkers(g)
        if ids is None:
            continue
        Tf = K.fk_frames(r["angles"])[-1]
        for cc, tid in zip(c, ids.ravel()):
            if int(tid) in tgt3d and ("files" not in targets[int(tid)] or r["file"] in targets[int(tid)]["files"]):
                obs.append((Tf, tgt3d[int(tid)], cc.reshape(4, 2) / 1.5, r["file"], int(tid)))
    if len(obs) < 4:
        return None

    # init: per-view PnP (f=500) -> camera pose in arm frame -> T_cam_flange guesses
    K0 = np.array([[F_PRIOR, 0, 320], [0, F_PRIOR, 240], [0, 0, 1.0]])
    guesses = []
    for Tf, X, uv, fn, tid in obs:
        ok, rv, tv = cv2.solvePnP(X.astype(np.float32), uv.astype(np.float32), K0, None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        T_cam_arm = np.eye(4)
        T_cam_arm[:3, :3], T_cam_arm[:3, 3] = rodrigues(rv), tv.ravel()     # arm -> cam
        T_cam_fl = T_cam_arm @ Tf                                            # flange -> cam
        guesses.append(T_cam_fl)
    # robust init: the median translation, rotation of the guess closest to it
    ts = np.array([g[:3, 3] for g in guesses])
    i0 = int(np.argmin(np.linalg.norm(ts - np.median(ts, 0), axis=1)))
    R0, t0 = guesses[i0][:3, :3], guesses[i0][:3, 3]
    x = np.concatenate([[F_PRIOR, 320.0, 240.0, 0.0], cv2.Rodrigues(R0)[0].ravel(), t0])

    def residuals(x, use):
        p = x[:4]
        R, t = rodrigues(x[4:7]), x[7:10]
        res = []
        for k in use:
            Tf, X, uv, _, _ = obs[k]
            Xf = (np.linalg.inv(Tf) @ np.c_[X, np.ones(4)].T).T[:, :3]   # arm -> flange frame
            Pc = (R @ Xf.T).T + t
            res.append((project(Pc, p) - uv).ravel())
        prior = [PRIOR_W * (p[0] - F_PRIOR) / 30.0 * 10, PRIOR_W * (p[1] - 320) / 30.0 * 10,
                 PRIOR_W * (p[2] - 240) / 30.0 * 10, PRIOR_W * p[3] / 0.1 * 10]
        return np.concatenate(res + [np.array(prior)])

    def solve(x, use, iters=200):
        lam = 1e-2
        for _ in range(iters):
            r = residuals(x, use)
            J = np.zeros((r.size, x.size))
            for j in range(x.size):
                dx = np.zeros_like(x)
                dx[j] = 1e-4 * max(1.0, abs(x[j]))
                J[:, j] = (residuals(x + dx, use) - r) / dx[j]
            step = np.linalg.solve(J.T @ J + lam * np.diag(np.diag(J.T @ J) + 1e-9), -J.T @ r)
            if np.linalg.norm(residuals(x + step, use)) < np.linalg.norm(r):
                x, lam = x + step, lam / 3
            else:
                lam *= 5
            if np.linalg.norm(step) < 1e-7:
                break
        return x

    use = list(range(len(obs)))
    x = solve(x, use)
    per = [np.abs(residuals(x, [k])[:-4]).reshape(-1, 2) for k in use]
    per_mean = [float(np.linalg.norm(p, axis=1).mean()) for p in per]
    print(f"  k={k90}: per-view mean px before outlier removal {np.round(per_mean, 1).tolist()}")
    keep = [k for k, m in zip(use, per_mean) if m <= 8.0]
    if len(keep) < 4:
        keep = use   # too few inliers: report the all-views fit (high RMS -> reject) instead of NaN
    if len(keep) < len(use) and len(keep) >= 4:
        print(f"dropping {len(use) - len(keep)} outlier views: {[obs[k][3] for k in use if k not in keep]}")
        x = solve(x, keep)
    r = residuals(x, keep)[:-4].reshape(-1, 2)
    rms = float(np.sqrt((np.linalg.norm(r, axis=1) ** 2).mean()))
    f, cx, cy, k1 = x[:4]
    return {"created": datetime.datetime.now().isoformat(timespec="seconds"), "scan": scan, "targets": targets, "yaw_k90": k90,
            "f": f, "cx": cx, "cy": cy, "k1": k1, "R_cam_flange": rodrigues(x[4:7]).tolist(), "t_cam_flange_mm": x[7:10].tolist(),
            "rms_px": rms, "n_views": len(keep), "n_obs": len(obs), "dropped": [obs[k][3] for k in use if k not in keep],
            "note": "p_cam = R p_flange + t; distortion x_d = x (1 + k1 r^2)"}


if __name__ == "__main__":
    sys.exit(main())
