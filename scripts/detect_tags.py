#!/usr/bin/env python3
"""Detect AprilTag 36h11 markers (on the blocks) with the RealSense and measure them in 3D from depth.

For each tag: pixel corners, 3D corners in the CAMERA frame (m; depth aligned to color, median over a
small window), 3D center, mean edge length (a measurement of the printed tag size), and the plane normal.
No tag size is assumed. Saves nothing unless --json is given.

Usage (on the laptop, camera attached):
    ~/venvs/realsense/bin/python scripts/detect_tags.py [--frames 15] [--json out.json] [--image out.png]
"""
import argparse
import datetime
import json
import sys

import cv2
import numpy as np
import pyrealsense2 as rs

W, H, FPS = 1920, 1080, 30
TAG_SIZE_M = 0.020  # black square edge, measured by the user 2026-09-25 (blocks are 30 mm; original tags IDs 1-4)
# P-touch PT-P710BT labels (assets/ptouch_tags_36h11_20260925_202148): black square measured 18 mm by the user 2026-09-26
TAG_SIZE_BY_ID = {i: 0.018 for i in (10, 11, 12, 13, 14, 15, 16, 17, 30, 31, 32, 33, 34)}


def tag_size_m(tag_id):
    return TAG_SIZE_BY_ID.get(int(tag_id), TAG_SIZE_M)


def depth_at(depth_m, u, v, r=3):
    """Median of valid depth in a (2r+1)^2 window around pixel (u, v)."""
    h, w = depth_m.shape
    u0, u1 = max(0, int(round(u)) - r), min(w, int(round(u)) + r + 1)
    v0, v1 = max(0, int(round(v)) - r), min(h, int(round(v)) + r + 1)
    win = depth_m[v0:v1, u0:u1]
    win = win[win > 0]
    return float(np.median(win)) if win.size else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=15, help="frames to average depth over")
    ap.add_argument("--json")
    ap.add_argument("--image")
    args = ap.parse_args()

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, FPS)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    try:
        for _ in range(30):
            pipe.wait_for_frames()
        depths, color = [], None
        for _ in range(args.frames):
            fs = align.process(pipe.wait_for_frames())
            color = np.asanyarray(fs.get_color_frame().get_data()).copy()
            depths.append(np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale)
        intr = fs.get_color_frame().profile.as_video_stream_profile().intrinsics
    finally:
        pipe.stop()
    d = np.stack(depths)
    d[d == 0] = np.nan
    with np.errstate(all="ignore"), __import__("warnings").catch_warnings():
        __import__("warnings").simplefilter("ignore", RuntimeWarning)
        depth_m = np.nan_to_num(np.nanmedian(d, axis=0))

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    out = {"time": datetime.datetime.now().isoformat(timespec="seconds"), "pyrealsense2": rs.__version__,
           "opencv": cv2.__version__, "color": [W, H],
           "intrinsics": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy, "model": str(intr.model),
                          "coeffs": list(intr.coeffs)},
           "tags": []}
    print(f"detect_tags.py  {out['time']}  intrinsics fx {intr.fx:.1f} fy {intr.fy:.1f} pp ({intr.ppx:.1f}, {intr.ppy:.1f})")
    if ids is None:
        print("no tags found")
    else:
        for c, i in zip(corners, ids.flatten()):
            px = c.reshape(4, 2)
            pts = []
            for u, v in px:
                z = depth_at(depth_m, u, v)
                pts.append(rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], z) if z > 0 else [np.nan] * 3)
            pts = np.array(pts)
            edges = [np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]
            n = np.cross(pts[1] - pts[0], pts[3] - pts[0])
            n = n / np.linalg.norm(n) if np.all(np.isfinite(n)) and np.linalg.norm(n) > 0 else n
            tag = {"id": int(i), "pixel_center": px.mean(0).round(1).tolist(), "corners_px": px.round(1).tolist(),
                   "corners_cam_m": np.round(pts, 4).tolist(), "center_cam_m": np.round(np.nanmean(pts, 0), 4).tolist(),
                   "edge_mm": [round(float(e) * 1000, 1) for e in edges], "normal_cam": np.round(n, 3).tolist()}
            out["tags"].append(tag)
            print(f"tag {tag['id']}: px {tag['pixel_center']}  center_cam {tag['center_cam_m']} m  "
                  f"edges {tag['edge_mm']} mm  normal {tag['normal_cam']}")
            # PnP from the color corners with the known tag size (IPPE_SQUARE corner order: TL, TR, BR, BL)
            h = tag_size_m(i) / 2
            obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
            K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
            ok, rvec, tvec = cv2.solvePnP(obj, px.astype(np.float32), K, np.array(intr.coeffs), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                R, _ = cv2.Rodrigues(rvec)
                tag["pnp_center_cam_m"] = np.round(tvec.ravel(), 4).tolist()
                tag["pnp_normal_cam"] = np.round(R[:, 2], 3).tolist()
                tag["pnp_minus_depth_mm"] = np.round((tvec.ravel() - np.nanmean(pts, 0)) * 1000, 1).tolist()
                print(f"        PnP center {tag['pnp_center_cam_m']} m  normal {tag['pnp_normal_cam']}  PnP-depth {tag['pnp_minus_depth_mm']} mm")
            cv2.polylines(color, [px.astype(int)], True, (0, 0, 255), 2)
            cv2.putText(color, str(tag["id"]), tuple(px[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if args.json:
        json.dump(out, open(args.json, "w"), indent=1)
    if args.image:
        cv2.imwrite(args.image, color)
    return 0


if __name__ == "__main__":
    sys.exit(main())
