#!/usr/bin/env python3
"""Camera-truth hand position from the gripper AprilTags (IDs 31/32, 18 mm) in a recorded map_local.py --j6 tour.

For every labelled hold (joint log label "hold_*"): RealSense frames inside the hold (Jetson -> laptop clock via the
session's start/end clock offsets, first 0.3 s skipped), the hand tags detected in a crop around the FK-predicted hand
(upscaled), PnP (IPPE_SQUARE, 18 mm) -> tag centre in the camera frame -> arm frame with the camera<->arm calibration.
Model: tag centre = flange position + flange rotation @ o_id (o_id = the tag's unknown fixed offset in the flange frame,
one per tag ID, least squares over all holds). Residual per hold = measured - model = the camera-truth position error
that a constant tag offset cannot explain (arm sag/tracking + calibration error, ~2-3 mm for the calibration alone).

  ~/venvs/realsense/bin/python scripts/handtag_fit.py data/<session> [--calib config/cam_arm_calib_*.json]
Writes <session>/handtag_fit.json and prints a per-point table.
"""
import argparse
import csv
import datetime
import glob
import json
import os
import re
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kinematics as K  # noqa: E402

HAND_IDS = (31, 32, 33, 34)
TAG_M = 0.018
H = TAG_M / 2
OBJ = np.array([[-H, H, 0], [H, H, 0], [H, -H, 0], [-H, -H, 0]], np.float32)
CROP, UP, MIN_FRAMES = 140, 3.0, 3   # crop half-size px, upscale, frames needed for a hold measurement


def load_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--calib", default=sorted(glob.glob(os.path.join(os.path.dirname(__file__), "..", "config", "cam_arm_calib_*.json")))[-1])
    a = ap.parse_args()
    S = a.session
    meta = json.load(open(os.path.join(S, "metadata.json")))
    ci = json.load(open(os.path.join(S, "realsense_info.json")))["color_intrinsics"]
    Kc = np.array([[ci["fx"], 0, ci["ppx"]], [0, ci["fy"], ci["ppy"]], [0, 0, 1.0]])
    dist = np.array(ci["coeffs"])
    cal = json.load(open(a.calib))
    R, t = np.array(cal["R_base_to_cam"]), np.array(cal["t_base_to_cam_mm"])
    o0, o1 = meta["clock_offset_start"]["offset_s"], meta["clock_offset_end"]["offset_s"]

    rows = load_csv(os.path.join(S, "jetson", "joints.csv"))
    tj = np.array([float(r["t_host"]) for r in rows])
    to_laptop = lambda x: x - (o0 + (o1 - o0) * (x - tj[0]) / max(tj[-1] - tj[0], 1e-6))
    holds = {}
    for r in rows:
        if r["label"].startswith("hold_") and r["event"] in ("hold_begin", "poll", "hold_end"):
            h = holds.setdefault(r["label"], {"t": [], "q": []})
            h["t"].append(float(r["t_host"]))
            if r["j1"]:
                h["q"].append([float(r[f"j{k}"]) for k in range(1, 7)])
    frames = load_csv(os.path.join(S, "realsense_frames.csv"))
    tf = np.array([float(r["t_host"]) for r in frames])
    cap = cv2.VideoCapture(os.path.join(S, "realsense_color.mp4"))
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())

    meas = []   # (label, id, p_arm(3), q_mean(6), n_frames)
    for lab, h in holds.items():
        if len(h["q"]) < 3:
            continue
        q = np.mean(np.array(h["q"]), 0)
        Fl = K.fk_frames(q)[-1]
        hand = Fl[:3, 3] + 0.5 * K.GRIPPER_LEN * Fl[:3, 2]          # middle of the gripper body: crop centre
        qc = R @ hand + t
        u0, v0 = Kc[0, 0] * qc[0] / qc[2] + Kc[0, 2], Kc[1, 1] * qc[1] / qc[2] + Kc[1, 2]
        t0, t1 = to_laptop(min(h["t"])) + 0.3, to_laptop(max(h["t"]))
        per = {}
        for i in np.where((tf >= t0) & (tf <= t1))[0]:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, img = cap.read()
            if not ok:
                continue
            x0, y0 = int(max(u0 - CROP, 0)), int(max(v0 - CROP, 0))
            g = cv2.cvtColor(img[y0:int(v0 + CROP), x0:int(u0 + CROP)], cv2.COLOR_BGR2GRAY)
            if g.size == 0:
                continue
            corners, ids, _ = det.detectMarkers(cv2.resize(g, None, fx=UP, fy=UP, interpolation=cv2.INTER_CUBIC))
            if ids is None:
                continue
            for c, tid in zip(corners, ids.ravel()):
                if int(tid) not in HAND_IDS:
                    continue
                px = (c.reshape(4, 2) / UP + [x0, y0]).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(OBJ, px, Kc, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    Rt, _ = cv2.Rodrigues(rvec)
                    # tag centre (arm frame, mm) and tag-plane normal in the FLANGE frame (sign: toward the camera)
                    n = R.T @ Rt[:, 2]
                    pc = R.T @ (tvec.ravel() * 1000 - t)
                    if n @ (-R.T @ t - pc) < 0:
                        n = -n
                    per.setdefault(int(tid), []).append(np.concatenate([pc, Fl[:3, :3].T @ n]))
        for tid, ps in per.items():
            if len(ps) >= MIN_FRAMES:
                ps = np.array(ps)
                nf = np.median(ps[:, 3:], 0)
                meas.append((lab, tid, np.median(ps[:, :3], 0), q, len(ps), float(np.linalg.norm(ps[:, :3].std(0))), nf / np.linalg.norm(nf)))
        print(f"{lab:26s} frames {int(((tf >= t0) & (tf <= t1)).sum()):3d}  hand tags {({k: len(v) for k, v in per.items()})}", flush=True)

    out = {"session": os.path.basename(os.path.normpath(S)), "created": datetime.datetime.now().isoformat(timespec="seconds"),
           "generator": "scripts/handtag_fit.py", "calib": os.path.basename(a.calib), "tag_size_m": TAG_M, "fits": {}}
    for tid in HAND_IDS:
        m = [x for x in meas if x[1] == tid]
        if len(m) < 4:
            print(f"tag {tid}: {len(m)} holds with a measurement (need >= 4) -> no fit")
            continue
        A = np.vstack([K.fk_frames(x[3])[-1][:3, :3] for x in m])
        b = np.concatenate([x[2] - K.fk_frames(x[3])[-1][:3, 3] for x in m])
        o, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = (b - A @ o).reshape(-1, 3)
        err = np.linalg.norm(res, axis=1)
        nf = np.mean([x[6] for x in m], 0)
        nf /= np.linalg.norm(nf)
        spread = float(np.degrees(np.max([np.arccos(np.clip(x[6] @ nf, -1, 1)) for x in m])))
        out["fits"][str(tid)] = {"offset_flange_mm": np.round(o, 1).tolist(), "n_holds": len(m),
                                 "face_normal_flange": np.round(nf, 3).tolist(),
                                 "face_normal_azimuth_in_flange_xy_deg": round(float(np.degrees(np.arctan2(nf[1], nf[0]))), 1),
                                 "face_normal_tilt_from_flange_xy_deg": round(float(np.degrees(np.arcsin(np.clip(nf[2], -1, 1)))), 1),
                                 "face_normal_max_spread_deg": round(spread, 1),
                                 "rms_mm": round(float(np.sqrt((err ** 2).mean())), 1), "max_mm": round(float(err.max()), 1),
                                 "holds": [{"label": x[0], "frames": x[4], "frame_scatter_mm": round(x[5], 1),
                                            "residual_mm": np.round(rr, 1).tolist(), "err_mm": round(float(e), 1),
                                            "face_normal_flange": np.round(x[6], 3).tolist()}
                                           for x, rr, e in zip(m, res, err)]}
        fo = out["fits"][str(tid)]
        print(f"\ntag {tid}: offset in flange frame {np.round(o, 1).tolist()} mm, {len(m)} holds, "
              f"RMS {fo['rms_mm']} mm, max {fo['max_mm']} mm")
        print(f"  face normal (flange frame) {fo['face_normal_flange']}: azimuth in flange xy {fo['face_normal_azimuth_in_flange_xy_deg']} deg, "
              f"tilt {fo['face_normal_tilt_from_flange_xy_deg']} deg, spread over holds <= {fo['face_normal_max_spread_deg']} deg")
        for x, rr, e in zip(m, res, err):
            rt = re.match(r"hold_r(\d+)_th(-?\d+)_j6([+-]\d+)", x[0])
            print(f"  r {rt.group(1):>3} th {rt.group(2):>4} j6 {rt.group(3):>4}: err {e:5.1f} mm  residual xyz {np.round(rr, 1).tolist()}"
                  f"  ({x[4]} frames, scatter {x[5]:.1f})  normal az {np.degrees(np.arctan2(x[6][1], x[6][0])):6.1f} tilt {np.degrees(np.arcsin(x[6][2])):5.1f}")
    json.dump(out, open(os.path.join(S, "handtag_fit.json"), "w"), indent=1)
    print(f"\n-> {os.path.join(S, 'handtag_fit.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
