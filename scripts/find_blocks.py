#!/usr/bin/env python3
"""Find the 30 mm tagged blocks on the table and express them in the ARM (controller) frame.

RealSense 1920x1080 color, N frames; AprilTag 36h11 (20 mm) per frame; tags clustered by pixel position (IDs repeat
across blocks); per cluster the median PnP pose -> arm frame via the camera<->arm calibration
(p_cam = R p_base + t, mm). For each block: the tag face center, the face normal in the arm frame, the block center
(= face center - 15 mm along the outward normal), the yaw of the tag edges about the arm z axis (mod 90 deg) when the
tag is on the top face, the radius/azimuth from the base, and whether the working-tilt grasp is reachable (IK).

Usage (laptop, camera attached):
    ~/venvs/realsense/bin/python scripts/find_blocks.py [--frames 30] [--calib config/cam_arm_calib_*.json] [--json out.json]
"""
import argparse
import datetime
import glob
import json
import os
import sys

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_movements as G  # noqa: E402

TAG = 0.020  # original block tags (IDs 1-4)
TAG_BY_ID = {i: 0.018 for i in (10, 11, 12, 13, 14, 15, 16, 17, 30, 31, 32, 33, 34)}  # P-touch labels, measured 18 mm (2026-09-26)


def tag_obj(tid):
    h = TAG_BY_ID.get(int(tid), TAG) / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
BLOCK = 30.0
Z_TOP = 14.9  # mm, arm frame: top face of a block resting on the table (tag center at the verified 2026-09-25 pick)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--calib", default=sorted(glob.glob(os.path.join(os.path.dirname(__file__), "..", "config", "cam_arm_calib_*.json")))[-1])
    ap.add_argument("--json")
    ap.add_argument("--image")
    args = ap.parse_args()
    cal = json.load(open(args.calib))
    Rbc, tbc = np.array(cal["R_base_to_cam"]), np.array(cal["t_base_to_cam_mm"])

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
    Kc = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.0]])
    dist = np.array(intr.coeffs)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    dets, img = [], None
    try:
        for _ in range(30):
            pipe.wait_for_frames()
        for _ in range(args.frames):
            img = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data()).copy()
            g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            corners, ids, _ = det.detectMarkers(g)
            if ids is None:
                continue
            for c, tid in zip(corners, ids.ravel()):
                px = c.reshape(4, 2) / 1.5
                # 2026-09-26: IPPE has two mirror solutions for a small, oblique tag; the lower-error one flipped several TOP
                # tags to a ~57 deg "side" normal (n_z ~0.5, same pixel/plane position as a TOP read minutes earlier).
                # Blocks rest on the table: take the near-vertical solution when its error is within 2x (+0.5 px) of the best.
                n_sol, rvecs, tvecs, errs = cv2.solvePnPGeneric(tag_obj(tid), px.astype(np.float32), Kc, dist,
                                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not n_sol:
                    continue
                errs = np.asarray(errs).ravel()
                k = int(np.argmin(errs))
                for j in range(n_sol):
                    Rj, _ = cv2.Rodrigues(rvecs[j])
                    if j != k and abs((Rbc.T @ Rj[:, 2])[2]) > 0.9 and errs[j] <= 2 * errs[k] + 0.5:
                        k = j
                Rt, _ = cv2.Rodrigues(rvecs[k])
                dets.append({"id": int(tid), "px": px.mean(0), "corners": px, "t": tvecs[k].ravel() * 1000, "R": Rt,
                             "pnp_err": [round(float(e), 3) for e in errs]})
    finally:
        pipe.stop()

    # cluster detections by pixel center
    clusters = []
    for d in dets:
        for cl in clusters:
            if np.linalg.norm(cl[0]["px"] - d["px"]) < 15:
                cl.append(d)
                break
        else:
            clusters.append([d])
    blocks = []
    for cl in clusters:
        if len(cl) < max(3, args.frames // 4):
            continue
        t_cam = np.median(np.array([d["t"] for d in cl]), 0)
        Rm = cl[len(cl) // 2]["R"]
        n_cam, x_cam = Rm[:, 2], Rm[:, 0]
        face = Rbc.T @ (t_cam - tbc)
        n = Rbc.T @ n_cam
        xa = Rbc.T @ x_cam
        # PnP normal points out of the tag's front (toward the camera side of the face); make it point away from the block:
        # for a top face that means +z. Only the sign matters; top-face tags have |n_z| ~ 1.
        top = abs(n[2]) > 0.8
        n_out = n * np.sign(n[2]) if top else n * (1 if np.dot(n, face - np.array([face[0], face[1], 0])) >= 0 else -1)
        center = face - (BLOCK / 2) * n_out
        yaw = float(np.degrees(np.arctan2(xa[1], xa[0])) % 90) if top else None
        # Ray-plane: tag-center pixel ray intersected with the block-top plane (removes PnP range noise, ~+-2 cm at 1 m)
        pxc = np.median(np.array([d["px"] for d in cl]), 0)
        ray_c = np.linalg.solve(Kc, np.array([pxc[0], pxc[1], 1.0]))
        ray_b = Rbc.T @ ray_c
        cam_b = -Rbc.T @ tbc
        s_ = (Z_TOP - cam_b[2]) / ray_b[2]
        plane_xy = (cam_b + s_ * ray_b)[:2]
        if top:
            center = np.array([plane_xy[0], plane_xy[1], Z_TOP - BLOCK / 2])
        side_xy = side_yaw = None
        if not top and np.linalg.norm(n_out[:2]) > 0.3:
            # 2026-09-26 (dice roll of side-tag blocks): the tag ray cut at mid-block height, then half a block back along
            # the HORIZONTAL face normal, pointed toward the camera (the tag is seen, so its face looks at the camera;
            # the PnP normal of side tags tilts up to ~30 deg)
            s2 = (Z_TOP - BLOCK / 2 - cam_b[2]) / ray_b[2]
            fxy = (cam_b + s2 * ray_b)[:2]
            nh = n_out[:2] / np.linalg.norm(n_out[:2])
            if np.dot(nh, cam_b[:2] - fxy) < 0:
                nh = -nh
            side_xy = fxy - (BLOCK / 2) * nh
            side_yaw = float(np.degrees(np.arctan2(nh[1], nh[0])) % 90)
        r, az = float(np.hypot(center[0], center[1])), float(np.degrees(np.arctan2(center[1], center[0])))
        blocks.append({"id": cl[0]["id"], "n_frames": len(cl), "pixel": np.round(cl[0]["px"], 1).tolist(),
                       "face_center_mm": np.round(face, 1).tolist(), "face_normal": np.round(n_out, 3).tolist(),
                       "top_face_tag": bool(top), "block_center_mm": np.round(center, 1).tolist(),
                       "pnp_face_center_mm": np.round(face, 1).tolist(), "plane_xy_mm": np.round(plane_xy, 1).tolist(),
                       "yaw_deg_mod90": None if yaw is None else round(yaw, 1), "r_mm": round(r, 1), "az_deg": round(az, 1),
                       "side_center_xy_mm": None if side_xy is None else np.round(side_xy, 1).tolist(),
                       "side_yaw_deg_mod90": None if side_yaw is None else round(side_yaw, 1)})
        if img is not None:
            cv2.polylines(img, [cl[0]["corners"].astype(int)], True, (0, 0, 255), 2)
            cv2.putText(img, f"{len(blocks) - 1}", tuple(cl[0]["corners"][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    tops = [b["face_center_mm"][2] for b in blocks if b["top_face_tag"]]
    table_z = float(np.median(tops) - BLOCK) if tops else None
    print(f"find_blocks.py {datetime.datetime.now().isoformat(timespec='seconds')}  calib {os.path.basename(args.calib)}  "
          f"{len(dets)} detections in {args.frames} frames")
    for k, b in enumerate(blocks):
        print(f"[{k}] tag {b['id']:2d} {'TOP ' if b['top_face_tag'] else 'SIDE'} center {b['block_center_mm']} mm  "
              f"r {b['r_mm']:.0f} az {b['az_deg']:.0f}  yaw {b['yaw_deg_mod90']}  frames {b['n_frames']}")
    print(f"table z estimate from top faces: {table_z if table_z is None else round(table_z, 1)} mm "
          f"(n={len(tops)}; model assumption was {0.0})")
    if args.json:
        json.dump({"created": datetime.datetime.now().isoformat(timespec="seconds"), "calib": os.path.basename(args.calib),
                   "table_z_mm": table_z, "blocks": blocks}, open(args.json, "w"), indent=1)
    if args.image and img is not None:
        cv2.imwrite(args.image, img)
    return 0


if __name__ == "__main__":
    sys.exit(main())
