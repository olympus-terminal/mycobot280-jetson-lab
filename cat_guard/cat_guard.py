#!/usr/bin/env python3
"""Cat guard.

Cat guard: detect anything TALL in the arm's work area with the RealSense depth, and ask it (robot voice) to leave.

Reference: median depth of N frames of the empty work area with the arm PARKED (`cat_guard.py ref`).
Check: median of a few current frames; intrusion = pixels inside the work-area mask that are > THRESH_MM closer to the
camera than the reference (a 30 mm block barely changes depth at this viewing angle; a cat or a hand changes it a lot),
largest connected blob >= MIN_PX. Work-area mask: the table disc of radius R_MM around the arm base, from the
camera<->arm calibration, plus the space up to 350 mm above it; each changed pixel is deprojected with the current depth
and kept only if its 3-D point lies inside that cylinder (the 2-D outline alone also covers the room behind the table).

  cat_guard.py ref                 save data/perception/catguard_ref.npz
  cat_guard.py check [--speak]     exit 0 clear, 1 intrusion (speaks if --speak)
  cat_guard.py wait [--timeout 180]  loop: check, warn, wait 10 s, until clear (exit 0) or timeout (exit 1)
"""
import argparse
import datetime
import glob
import json
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")  # repo scripts/ (paths below are relative to it)
REF = os.path.join(HERE, "..", "data", "perception", "catguard_ref.npz")
LOG = os.path.join(HERE, "..", "data", "perception", "catguard_log.jsonl")
THRESH_MM, MIN_PX, R_MM = 45.0, 1500, 300.0
Z_LO_MM, Z_HI_MM = -15.0, 335.0  # table level .. 350 mm above it (arm base frame)
LINES = ["Attention. Feline detected in the work area. Remove yourself from the area.",
         "Cat. You have ten seconds to comply.",
         "This is your final warning. Step away from the blocks."]


def depth_frames(n):
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    try:
        for _ in range(15):
            pipe.wait_for_frames()
        ds, color = [], None
        for _ in range(n):
            fs = align.process(pipe.wait_for_frames())
            ds.append(np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale * 1000)
            color = np.asanyarray(fs.get_color_frame().get_data()).copy()
        intr = fs.get_color_frame().profile.as_video_stream_profile().intrinsics
    finally:
        pipe.stop()
    d = np.stack(ds)
    d[d == 0] = np.nan
    return np.nanmedian(d, 0), color, intr


def load_calib():
    cal = json.load(open(sorted(glob.glob(os.path.join(HERE, "..", "config", "cam_arm_calib_*.json")))[-1]))
    return np.array(cal["R_base_to_cam"]), np.array(cal["t_base_to_cam_mm"])


def work_mask(intr, shape):
    """2-D outline of the work cylinder (cheap pre-filter; includes everything BEHIND it along the rays)."""
    R, t = load_calib()
    mask = np.zeros(shape, np.uint8)
    for z in (Z_LO_MM, Z_HI_MM):
        pts = []
        for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
            q = R @ np.array([R_MM * np.cos(a), R_MM * np.sin(a), z]) + t
            pts.append([intr.fx * q[0] / q[2] + intr.ppx, intr.fy * q[1] / q[2] + intr.ppy])
        cv2.fillPoly(mask, [np.array(pts, np.int32)], 255)
    return mask > 0


def in_work_volume(d, intr, cand):
    """cand pixels whose CURRENT 3-D point (colour-aligned depth, mm) lies in the cylinder r <= R_MM, Z_LO..Z_HI (arm base frame)."""
    R, t = load_calib()
    v, u = np.nonzero(cand)
    z = d[v, u]
    q = np.stack([(u - intr.ppx) / intr.fx * z, (v - intr.ppy) / intr.fy * z, z], 1)
    p = (q - t) @ R  # = R.T @ (q - t) per row: camera -> arm base
    ok = (np.hypot(p[:, 0], p[:, 1]) <= R_MM) & (p[:, 2] >= Z_LO_MM) & (p[:, 2] <= Z_HI_MM)
    out = np.zeros_like(cand)
    out[v[ok], u[ok]] = True
    return out


def speak(text):
    subprocess.run(["espeak-ng", "-v", "en-us+m3", "-p", "18", "-s", "135", "-a", "160", text], capture_output=True)


def check(speak_level=None):
    ref = np.load(REF)["depth"]
    d, color, intr = depth_frames(5)
    m = work_mask(intr, d.shape)
    closer = (ref - d) > THRESH_MM
    closer &= m & np.isfinite(d) & np.isfinite(ref)
    n_2d = int(closer.sum())
    closer = in_work_volume(d, intr, closer)
    n, lab, st, _ = cv2.connectedComponentsWithStats(closer.astype(np.uint8))
    big = int(st[1:, cv2.CC_STAT_AREA].max()) if n > 1 else 0
    intr_found = big >= MIN_PX
    rec = {"t": datetime.datetime.now().isoformat(timespec="seconds"), "largest_blob_px": big, "intrusion": intr_found,
           "closer_px_2d": n_2d, "closer_px_3d": int(closer.sum())}
    if intr_found:
        k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
        x, y, w, h = st[k, :4]
        rec["bbox"] = [int(x), int(y), int(w), int(h)]
        snap = os.path.join(HERE, "..", "data", "perception", f"catguard_{datetime.datetime.now():%Y%m%d_%H%M%S}.jpg")
        cv2.rectangle(color, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 3)
        cv2.imwrite(snap, color)
        np.savez_compressed(snap.replace(".jpg", "_depth.npz"), depth=d.astype(np.float16))  # replay/tuning
        rec["snapshot"] = os.path.basename(snap)
        if speak_level is not None:
            speak(LINES[min(speak_level, len(LINES) - 1)])
    with open(LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
    return intr_found, rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["ref", "check", "wait"])
    ap.add_argument("--speak", action="store_true")
    ap.add_argument("--timeout", type=float, default=180)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(REF), exist_ok=True)
    if a.cmd == "ref":
        d, color, intr = depth_frames(15)
        np.savez_compressed(REF, depth=d)
        cv2.imwrite(REF.replace(".npz", ".jpg"), color)
        print(f"reference saved ({np.isfinite(d).mean() * 100:.0f}% valid depth)")
        return 0
    if a.cmd == "check":
        found, rec = check(0 if a.speak else None)
        print(json.dumps(rec))
        return 1 if found else 0
    t0, level = time.time(), 0
    while time.time() - t0 < a.timeout:
        found, rec = check(level)
        print(json.dumps(rec), flush=True)
        if not found:
            if level:
                speak("Area clear. Thank you for your cooperation.")
            return 0
        level += 1
        time.sleep(10)
    return 1


if __name__ == "__main__":
    sys.exit(main())
