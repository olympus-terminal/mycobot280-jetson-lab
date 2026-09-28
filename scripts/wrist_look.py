#!/usr/bin/env python3
"""Grasp look (JETSON, system python3 with OpenCV aruco): wait for a wrist-camera frame newer than --after (written by
capture_wrist.py --latest), detect AprilTag 36h11 tags, optionally save the frame, print one JSON line.

2026-09-27: data collection for hand-camera grasp alignment. The camera is fixed to the hand, so a block that ends up
between the fingers appears at a fixed place in the image at the hover; these looks (with the grasp outcome in the cycle
summary) give that reference.

Usage: python3 scripts/wrist_look.py --after T [--timeout 3] [--save /tmp/x.jpg] [--latest /dev/shm/wrist_latest.jpg]
Output JSON: {"ok", "t_frame", "frame", "age_s", "tags": [{"id", "center": [x, y], "size_px"}], "saved"}
"""
import argparse
import json
import os
import shutil
import time

import cv2
import numpy as np

HAND_IDS = {30, 31, 32, 33, 34}   # base plate / gripper tags: never blocks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--after", type=float, required=True, help="unix time: use only a frame taken after this")
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--latest", default="/dev/shm/wrist_latest.jpg")
    ap.add_argument("--save")
    a = ap.parse_args()
    out = {"ok": False, "tags": []}
    t_end = time.time() + a.timeout
    while time.time() < t_end:
        try:
            t_f, n = open(a.latest + ".t").read().split()
            t_f = float(t_f)
        except (OSError, ValueError):
            t_f = 0.0
        if t_f > a.after:
            img = cv2.imread(a.latest)
            if img is not None:
                break
        time.sleep(0.05)
    else:
        out["error"] = "no fresh frame"
        print(json.dumps(out), flush=True)
        return 1
    out.update({"ok": True, "t_frame": t_f, "frame": int(n), "age_s": round(time.time() - t_f, 3)})
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if ids is not None:
        for c, tid in zip(corners, ids.ravel()):
            if int(tid) in HAND_IDS:
                continue
            p = c.reshape(4, 2)
            out["tags"].append({"id": int(tid), "center": np.round(p.mean(0), 1).tolist(),
                                "size_px": round(float(np.mean(np.linalg.norm(p - np.roll(p, 1, 0), axis=1))), 1)})
    if a.save:
        os.makedirs(os.path.dirname(a.save), exist_ok=True)
        shutil.copyfile(a.latest, a.save)
        out["saved"] = a.save
    print(json.dumps(out), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
