#!/usr/bin/env python3
"""Cut Cosmos conditioning clips from recorded sessions (REAL footage) around each session's fastest motion.

For each session: joint log (Jetson clock) -> laptop clock via the offset measured at the session start and end
(linear interpolation); peak joint speed -> window of --seconds centred on it; RealSense frames resampled to exactly
--fps by nearest timestamp; crop x range (--crop-x) to exclude people at the frame edge; H.264 MP4 via ffmpeg.
Writes <out>/<name>.mp4 and <out>/<name>.json (provenance: source session, frame indices, times, crop, offsets).

Usage:
    ~/venvs/realsense/bin/python scripts/make_cosmos_clips.py data/20260924_2044*_unpark ... --out data/cosmos_clips/batch1
"""
import argparse
import csv
import json
import os
import subprocess
import sys

import cv2
import numpy as np


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--crop-x", nargs=2, type=int, default=[320, 1280], help="x0 x1 in the 1280-wide frame")
    ap.add_argument("--size", nargs=2, type=int, default=[960, 720], help="output W H")
    ap.add_argument("--prefix", default="mycobot_b1")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    for sess in args.sessions:
        meta = json.load(open(os.path.join(sess, "metadata.json")))
        mvname = json.load(open(os.path.join(sess, "movement.json")))["name"]
        o0, o1 = meta["clock_offset_start"]["offset_s"], meta["clock_offset_end"]["offset_s"]
        joints = [r for r in load_csv(os.path.join(sess, "jetson", "joints.csv")) if r["j1"] != ""]
        tj = np.array([float(r["t_host"]) for r in joints])
        q = np.array([[float(r[f"j{k}"]) for k in range(1, 7)] for r in joints])
        # jetson -> laptop time, offset linearly interpolated over the joint log span
        w = (tj - tj[0]) / max(tj[-1] - tj[0], 1e-6)
        tl = tj - (o0 + (o1 - o0) * w)
        speed = np.r_[0, np.abs(np.diff(q, axis=0)).max(axis=1) / np.maximum(np.diff(tl), 1e-3)]
        k = 0
        # Window = the step with the largest commanded change. (Joint polling misses most of each move:
        # pymycobot blocks ~1.5 s re-sending unacknowledged commands, so peak logged speed is unreliable.)
        steps = [s for s in json.load(open(os.path.join(sess, "movement.json")))["steps"] if "angles" in s]
        big = int(np.argmax([0] + [np.abs(np.subtract(b["angles"], a["angles"])).max() for a, b in zip(steps, steps[1:])]))
        begins = [r for r in load_csv(os.path.join(sess, "jetson", "joints.csv")) if r["event"] == "step_begin"]
        ang_begins = [r for r in begins if r["step"] in {st["label"] for st in steps}]
        tb = float(ang_begins[big]["t_host"])
        tb_l = tb - (o0 + (o1 - o0) * (tb - tj[0]) / max(tj[-1] - tj[0], 1e-6))
        t_mid = tb_l + args.seconds / 2 - 0.2
        frames = load_csv(os.path.join(sess, "realsense_frames.csv"))
        tf = np.array([float(r["t_host"]) for r in frames])
        t0 = max(t_mid - args.seconds / 2, tf[0] + 2.0)  # skip the first 2 s (auto-exposure)
        t0 = min(t0, tf[-1] - args.seconds)
        targets = t0 + np.arange(int(round(args.seconds * args.fps))) / args.fps
        idx = np.searchsorted(tf, targets).clip(1, len(tf) - 1)
        idx = np.where(np.abs(tf[idx - 1] - targets) < np.abs(tf[idx] - targets), idx - 1, idx)
        cap = cv2.VideoCapture(os.path.join(sess, "realsense_color.mp4"))
        name = f"{args.prefix}_{mvname}_{args.fps}fps"
        tmp = os.path.join(args.out, name + "_raw.mp4")
        vw = cv2.VideoWriter(tmp, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, tuple(args.size))
        cache = {}
        for i in idx:
            if i not in cache:
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
                ok, f = cap.read()
                if not ok:
                    sys.exit(f"{sess}: cannot read frame {i}")
                cache[i] = cv2.resize(f[:, args.crop_x[0]:args.crop_x[1]], tuple(args.size), interpolation=cv2.INTER_AREA)
            vw.write(cache[i])
        vw.release()
        final = os.path.join(args.out, name + ".mp4")
        r = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", tmp, "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-crf", "16", "-r", str(args.fps), final], capture_output=True, text=True)
        if r.returncode:
            sys.exit(f"ffmpeg failed: {r.stderr}")
        os.remove(tmp)
        prov = {"clip": os.path.basename(final), "synthetic": False, "source_session": os.path.basename(sess.rstrip("/")),
                "movement": mvname, "source_video": "realsense_color.mp4", "frame_indices": idx.tolist(),
                "t_start_laptop": round(float(t0), 4), "seconds": args.seconds, "fps": args.fps,
                "crop_x": args.crop_x, "size": args.size, "window_step": steps[big]["label"], "window_rule": "starts 0.2 s before the begin of the largest commanded step",
                "clock_offset_start_s": o0, "clock_offset_end_s": o1, "generator": "scripts/make_cosmos_clips.py",
                "note": "crop excludes the person at the left frame edge (sweep_j1/j2); first 2 s skipped (auto-exposure)"}
        json.dump(prov, open(os.path.join(args.out, name + ".json"), "w"), indent=1)
        print(f"{name}: frames {idx[0]}..{idx[-1]} ({len(set(idx.tolist()))} unique), window = step {big} '{steps[big]['label']}'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
