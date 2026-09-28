#!/usr/bin/env python3
"""Draft promo cut (LAPTOP) from REAL recorded demo cycles: RealSense colour video of successful picker pick-and-place
cycles, cropped to the table, countdown trimmed, sped up, captioned, concatenated. No synthetic footage.

Usage: ~/venvs/realsense/bin/python scripts/make_promo_cut_20260926_171000.py [--speed 4] [--dim-bg [--pip]] [SESSION ...]

--dim-bg (2026-09-26 19:30): dim + blur the room behind the table using the RECORDED depth (aligned to colour): each pixel is
deprojected into the arm frame (camera<->arm calibration of that date); kept = valid depth, z > -45 mm (tabletop and above),
x > -230 mm (drops the PC case / cables behind the base; the arm never works there), r < 900 mm. Frames without a depth image
(09-26 07:11 clips: HDD drops, gaps <= 13 frames) use the union of the nearest earlier and later masks, dilated. The pixels
of the arm/table/blocks are not altered.
Output: data/promo/promo_draft_<stamp>.mp4 + promo_draft_<stamp>.json (sources, their summary.json result, params, git hash).

Default sessions (2026-09-26 17:10 selection): the picker demo cycles after the grasp fix that report "placed" and, in a
12-frame scan per clip, show no person in view (20260925_201905_cycle_sup4 has the user in frame; 20260926_150550 shows a
leg at the bottom left) - excluded.
"""
import argparse
import datetime
import json
import os
import subprocess
import sys

import cv2
import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DEFAULT = ["20260925_202737_cycle_sup5", "20260925_202938_cycle_demo_20260925_202931_000",
           "20260925_203232_cycle_demo_20260925_202931_001", "20260925_203616_cycle_demo_20260925_202931_002",
           "20260925_203830_cycle_demo_20260925_203823_000", "20260926_071136_cycle_demo_20260926_071129_000",
           "20260926_071401_cycle_demo_20260926_071129_001"]
CROP = "1030:580:150:0"      # w:h:x:y on the 1280x720 RealSense colour: the table, arm raised included (~16:9)
TRIM_START = 6.0             # s: the spoken countdown before the first move
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
CAL_OLD = "config/cam_arm_calib_20260925_115724.json"            # camera mount of 2026-09-25 ~11:00
CAL_NEW = "config/cam_arm_calib_20260926_071021_rotfix.json"     # + 0.5 deg rotation after the 09-26 replug
KEEP_Z, KEEP_X, KEEP_R = -45.0, -230.0, 900.0
CROP_XYWH = (150, 0, 1030, 580)
STATIC_FRAC, STATIC_FEATHER = 0.8, 81      # steady vignette (2026-09-26 20:05)
DYN_EMA, DYN_FEATHER = 0.6, 41
BG_SIGMA, BG_GAIN = 10, 0.35


def keep_mask(depth, R, t, rx, ry):
    z = depth.astype(np.float32)
    B = (np.stack([rx * z, ry * z, z], -1).reshape(-1, 3) - t) @ R          # arm frame: R^T (p_cam - t), mm
    k = ((z.ravel() > 0) & (B[:, 2] > KEEP_Z) & (B[:, 0] > KEEP_X) & (np.hypot(B[:, 0], B[:, 1]) < KEEP_R))
    k = k.reshape(z.shape).astype(np.uint8) * 255
    k = cv2.morphologyEx(k, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return cv2.morphologyEx(k, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


def render_dimmed(session, idx, speed, label, writer, pip=False, dim=True):
    """Frames of one session -> writer (ffmpeg stdin), background dimmed from the recorded depth.
    2026-09-26 20:05 (user: the per-frame mask looked "strobe-like and twitchy"): STEADY version =
      static part: pixels kept in >= STATIC_FRAC of the clip's depth frames (table, base, blocks at rest), computed once,
      feathered STATIC_FEATHER px -> never flickers;
      dynamic part: only kept pixels OUTSIDE the static part (the arm over the background), EMA over output frames
      (DYN_EMA), feathered DYN_FEATHER px;
      alpha = max(static, dynamic); background = blur(sigma BG_SIGMA) * BG_GAIN."""
    d = os.path.join(REPO, "data", session)
    cal = json.load(open(os.path.join(REPO, CAL_NEW if session >= "20260926_0710" else CAL_OLD)))
    R, t = np.array(cal["R_base_to_cam"]), np.array(cal["t_base_to_cam_mm"])
    I = json.load(open(os.path.join(d, "realsense_info.json")))["color_intrinsics"]
    u, v = np.meshgrid(np.arange(1280), np.arange(720))
    rx, ry = (u - I["ppx"]) / I["fx"], (v - I["ppy"]) / I["fy"]
    have = np.array(sorted(int(f[:6]) for f in os.listdir(os.path.join(d, "realsense_depth")) if f.endswith(".png")))
    dep = lambda i: cv2.imread(os.path.join(d, "realsense_depth", f"{i:06d}.png"), -1)
    # pass 1: static mask from every ~10th depth frame (skipped with dim=False: plain crop, 2026-09-27 user: no blur)
    acc = None
    if not dim:
        have = have[:0]
    sample = have[:: max(1, len(have) // 120)]
    for q in sample:
        k = keep_mask(dep(q), R, t, rx, ry) > 0
        acc = k.astype(np.float32) if acc is None else acc + k
    if acc is None:     # dim=False: everything kept
        acc, sample = np.ones((720, 1280), np.float32), [0]
    static_hard = (acc / len(sample) >= STATIC_FRAC).astype(np.uint8) * 255
    static_hard = cv2.morphologyEx(static_hard, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    static_soft = cv2.GaussianBlur(static_hard.astype(np.float32) / 255, (STATIC_FEATHER, STATIC_FEATHER), 0)
    outside = cv2.dilate(static_hard, np.ones((9, 9), np.uint8)) == 0
    cap = cv2.VideoCapture(os.path.join(d, "realsense_color.mp4"))
    step = max(1, int(round(speed)))
    x0, y0, w, h = CROP_XYWH
    stats = {"frames": 0, "depth_direct": 0, "depth_interp": 0, "static_px_frac": round(float(static_hard.mean() / 255), 3)}
    # --pip (2026-09-26 20:50): wrist camera inset, synced by timestamps (laptop = jetson - clock offset)
    wr = None
    if pip and os.path.exists(os.path.join(d, "jetson", "wrist.mp4")):
        import csv
        off = json.load(open(os.path.join(d, "metadata.json")))["clock_offset_start"]["offset_s"]
        w_t = np.array([float(r["t_host"]) for r in csv.DictReader(open(os.path.join(d, "jetson", "wrist_frames.csv")))]) - off
        rs_t = np.array([float(r["t_host"]) for r in csv.DictReader(open(os.path.join(d, "realsense_frames.csv")))])
        wr, w_idx, w_img = cv2.VideoCapture(os.path.join(d, "jetson", "wrist.mp4")), -1, None
        stats["pip"] = {"wrist_frames": len(w_t), "clock_offset_s": off}
    dyn_ema, i = None, -1
    while True:
        ok, img = cap.read()      # sequential read (seeking an MP4 per frame is slow)
        i += 1
        if not ok:
            break
        if i < int(TRIM_START * 30) or (i - int(TRIM_START * 30)) % step:
            continue
        if not dim:
            k = np.full((720, 1280), 255, np.uint8)
        elif os.path.exists(os.path.join(d, "realsense_depth", f"{i:06d}.png")):
            k = keep_mask(dep(i), R, t, rx, ry)
            stats["depth_direct"] += 1
        else:
            j = np.searchsorted(have, i)
            nb = [have[x] for x in (j - 1, j) if 0 <= x < len(have)]
            k = cv2.dilate(np.max([keep_mask(dep(q), R, t, rx, ry) for q in nb], 0), np.ones((15, 15), np.uint8))
            stats["depth_interp"] += 1
        dyn = ((k > 0) & outside).astype(np.float32)
        dyn_ema = dyn if dyn_ema is None else DYN_EMA * dyn_ema + (1 - DYN_EMA) * dyn
        a = np.maximum(static_soft, cv2.GaussianBlur(dyn_ema, (DYN_FEATHER, DYN_FEATHER), 0))[..., None]
        out = (img * a + cv2.GaussianBlur(img, (0, 0), BG_SIGMA) * BG_GAIN * (1 - a)).astype(np.uint8)
        out = cv2.resize(out[y0:y0 + h, x0:x0 + w], (1280, 720), interpolation=cv2.INTER_AREA)
        if wr is not None and i < len(rs_t):
            want = int(np.searchsorted(w_t, rs_t[i], side="right")) - 1     # latest wrist frame at or before this frame
            while w_idx < want:
                ok_w, fw = wr.read()
                if not ok_w:
                    break
                w_idx, w_img = w_idx + 1, fw
            if want >= 0 and w_img is not None:
                pw, ph, px0, py0 = 384, 288, 20, 20      # top LEFT: dark floor in every frame; top right hid the raised arm (20:57)
                out[py0 - 3:py0 + ph + 3, px0 - 3:px0 + pw + 3] = 255
                out[py0:py0 + ph, px0:px0 + pw] = cv2.resize(w_img, (pw, ph), interpolation=cv2.INTER_AREA)
                cv2.putText(out, "hand camera", (px0 + 8, py0 + ph - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(out, "hand camera", (px0 + 8, py0 + ph - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(out, (16, 720 - 58), (16 + 16 + 13 * len(label), 720 - 22), (0, 0, 0), -1)
        cv2.putText(out, label, (24, 720 - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(out.tobytes())
        stats["frames"] += 1
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="*", default=DEFAULT)
    ap.add_argument("--speed", type=float, default=4.0)
    ap.add_argument("--crop", default=CROP, help="w:h:x:y on the 1280x720 colour (plain path), e.g. 800:450:260:40 (tighter, 09-27)")
    ap.add_argument("--dim-bg", action="store_true", help="dim the room behind the table from the recorded depth")
    ap.add_argument("--pip", action="store_true", help="wrist camera inset (top left), synced by timestamps (with or without --dim-bg)")
    a = ap.parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(REPO, "data", "promo")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"promo_draft_{stamp}.mp4")
    srcs, inputs, filters = [], [], []
    for i, s in enumerate(a.sessions):
        d = os.path.join(REPO, "data", s)
        vid = os.path.join(d, "realsense_color.mp4")
        if not os.path.exists(vid):
            sys.exit(f"missing {vid}")
        summ = json.load(open(os.path.join(d, "summary.json")))
        srcs.append({"session": s, "result": summ.get("result"), "seconds": summ.get("seconds")})
        inputs += ["-ss", str(TRIM_START), "-i", vid]
        cap = f"real footage  |  {a.speed:g}x speed  |  move {i + 1}/{len(a.sessions)}"
        filters.append(f"[{i}:v]crop={a.crop},scale=-2:720,setpts=PTS/{a.speed},fps=30,"
                       f"drawtext=fontfile={FONT}:text='{cap}':x=24:y=h-48:fontsize=26:fontcolor=white:"
                       f"box=1:boxcolor=black@0.45:boxborderw=8[v{i}]")
    if a.dim_bg or a.pip:
        out = out.replace(".mp4", "_dimbg.mp4" if a.dim_bg else "_plain.mp4")
        ff = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                               "-s", "1280x720", "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", out],
                              stdin=subprocess.PIPE)
        for i, s in enumerate(a.sessions):
            lab = f"real footage | {a.speed:g}x speed | " + ("background dimmed | " if a.dim_bg else "") + f"move {i + 1}/{len(a.sessions)}"
            st = render_dimmed(s, i, a.speed, lab, ff.stdin, pip=a.pip, dim=a.dim_bg)
            srcs[i]["render"] = st
            print(s, st, flush=True)
        ff.stdin.close()
        if ff.wait():
            sys.exit("ffmpeg failed")
    else:
        run_filter_graph(inputs, filters, srcs, out)
    git = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    json.dump({"created": stamp, "script": os.path.basename(__file__), "git": git, "synthetic": False,
               "speed": a.speed, "crop": a.crop, "trim_start_s": TRIM_START, "dim_bg": a.dim_bg,
               "dim_bg_rule": {"z_gt_mm": KEEP_Z, "x_gt_mm": KEEP_X, "r_lt_mm": KEEP_R, "static_frac": STATIC_FRAC,
                               "static_feather": STATIC_FEATHER, "dyn_ema": DYN_EMA, "dyn_feather": DYN_FEATHER,
                               "bg_sigma": BG_SIGMA, "bg_gain": BG_GAIN} if a.dim_bg else None,
               "sources": srcs},
              open(out.replace(".mp4", ".json"), "w"), indent=1)
    print(out)


def run_filter_graph(inputs, filters, srcs, out):
    fc = ";".join(filters) + ";" + "".join(f"[v{i}]" for i in range(len(srcs))) + f"concat=n={len(srcs)}:v=1:a=0[out]"
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y"] + inputs + \
          ["-filter_complex", fc, "-map", "[out]", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.exit(r.stderr)


if __name__ == "__main__":
    main()
