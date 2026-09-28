#!/usr/bin/env python3
"""Pixel-exact AprilTag 36h11 PNGs for a Brother P-touch Cube PT-P710BT (180 dpi, 128-dot head, 24 mm tape).

Each PNG is 128 px tall = the full print head (~18.06 mm): the black 8x8-cell tag square is 128x128 px (16 dots/cell),
with QUIET_CELLS white cells left and right (the tape's own margins give the white above/below). Pure black/white,
no scaling: print at 100 % (image height 128 px = 128 dots). One PNG per ID, plus a strip of all IDs for one print.

  make_ptouch_tags_<ts>.py [--ids 10-17,30] [--copies 6] [--out assets/ptouch_tags_36h11_<ts>]
"""
import argparse
import datetime
import json
import os
import sys

import cv2
import numpy as np

HEAD_DOTS, DPI, CELLS, QUIET_CELLS = 128, 180, 8, 2  # 36h11 = 6x6 data + 1-cell black border each side


def parse_ids(s):
    out = []
    for part in s.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def tag_png(dic, i):
    px = HEAD_DOTS // CELLS
    tag = cv2.aruco.generateImageMarker(dic, i, HEAD_DOTS, borderBits=1)
    q = np.full((HEAD_DOTS, QUIET_CELLS * px), 255, np.uint8)
    return np.hstack([q, tag, q])


def main():
    ts = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default="10-17,30")
    ap.add_argument("--copies", type=int, default=0, help="also write one strip per ID with N copies (one block's six faces: 6)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets",
                                                  f"ptouch_tags_36h11_{ts}"))
    a = ap.parse_args()
    ids = parse_ids(a.ids)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
    os.makedirs(a.out, exist_ok=True)
    imgs = []
    for i in ids:
        img = tag_png(dic, i)
        assert img.shape[0] == HEAD_DOTS and set(np.unique(img)) <= {0, 255}, "not pixel-exact"
        _, got, _ = det.detectMarkers(cv2.copyMakeBorder(img, 32, 32, 0, 0, cv2.BORDER_CONSTANT, value=255))
        assert got is not None and [int(g) for g in got.ravel()] == [i], f"self-check failed for id {i}: {got}"
        cv2.imwrite(os.path.join(a.out, f"tag36h11_id{i:03d}_{HEAD_DOTS}px.png"), img)
        imgs.append(img)
    cv2.imwrite(os.path.join(a.out, f"strip_ids_{a.ids.replace(',', '_')}_{HEAD_DOTS}px.png"), np.hstack(imgs))
    if a.copies > 0:
        for i, img in zip(ids, imgs):
            cv2.imwrite(os.path.join(a.out, f"strip_id{i:03d}_x{a.copies}_{HEAD_DOTS}px.png"), np.hstack([img] * a.copies))
    mm = HEAD_DOTS / DPI * 25.4
    info = {"generated": ts, "script": os.path.basename(__file__), "opencv": cv2.__version__, "python": sys.version.split()[0],
            "family": "tag36h11 (cv2.aruco.DICT_APRILTAG_36h11)", "ids": ids, "printer": "Brother PT-P710BT, 180 dpi, 24 mm tape",
            "black_square_px": HEAD_DOTS, "dots_per_cell": HEAD_DOTS // CELLS, "quiet_cells_left_right": QUIET_CELLS,
            "black_square_mm_nominal": round(mm, 2), "copies_per_id_strip": a.copies, "note": "measure the printed black square and set TAG_SIZE_M to it"}
    json.dump(info, open(os.path.join(a.out, "info.json"), "w"), indent=1)
    print(f"{len(ids)} tags -> {os.path.abspath(a.out)}  (black square {HEAD_DOTS} px = {mm:.2f} mm nominal)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
