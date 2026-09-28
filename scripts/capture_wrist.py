#!/usr/bin/env python3
"""Record the wrist camera (UVC, /dev/video0) to MP4 plus a per-frame timestamp CSV.

Runs until --seconds elapse or SIGTERM/SIGINT (the recording orchestrator sends SIGTERM).
Uses the Jetson's system OpenCV (python3, not the mycobot venv).

Output: <out>/wrist.mp4, <out>/wrist_frames.csv (frame, t_host), <out>/wrist_info.json

Usage (on the Jetson):
    /usr/bin/python3 scripts/capture_wrist.py --out /tmp/session --seconds 10
"""
import argparse
import csv
import json
import os
import signal
import sys
import time

import cv2

STOP = False


def _stop(*_):
    global STOP
    STOP = True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=0, help="0 = until SIGTERM")
    ap.add_argument("--latest", default="/dev/shm/wrist_latest.jpg",
                    help="also keep the latest frame here (+ '.t' = 'time frame'); '' disables")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    os.makedirs(args.out, exist_ok=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    t0 = time.time()
    ok, frame = cap.read()
    while not ok and time.time() - t0 < 5.0:   # 2026-09-26: the first frame after a cold open can fail/take ~1.5 s
        time.sleep(0.2)
        ok, frame = cap.read()
    if not ok:
        sys.exit(f"cannot read from {args.device}")
    h, w = frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS) or args.fps
    fourcc = "".join(chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8 * k) & 0xFF) for k in range(4))
    writer = cv2.VideoWriter(os.path.join(args.out, "wrist.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    info = {"device": args.device, "width": w, "height": h, "fps_reported": fps, "fourcc": fourcc,
            "opencv": cv2.__version__, "started": time.time()}
    print(f"capture_wrist: {w}x{h} @ {fps} ({fourcc})", flush=True)

    n, t0 = 0, time.time()
    with open(os.path.join(args.out, "wrist_frames.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["frame", "t_host"])
        while not STOP and (args.seconds <= 0 or time.time() - t0 < args.seconds):
            if n > 0:
                ok, frame = cap.read()
                if not ok:
                    print("read failed; stopping", flush=True)
                    break
            t_f = time.time()
            wr.writerow([n, f"{t_f:.4f}"])
            writer.write(frame)
            if args.latest and n % 2 == 0:   # 2026-09-27: latest frame for the grasp look (cycle_local), atomic replace
                cv2.imwrite(args.latest + ".tmp.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                os.replace(args.latest + ".tmp.jpg", args.latest)
                with open(args.latest + ".t.tmp", "w") as ft:
                    ft.write(f"{t_f:.4f} {n}")
                os.replace(args.latest + ".t.tmp", args.latest + ".t")
            n += 1
    writer.release()
    cap.release()
    info.update({"frames": n, "stopped": time.time(), "fps_measured": round(n / max(time.time() - t0, 1e-6), 2)})
    json.dump(info, open(os.path.join(args.out, "wrist_info.json"), "w"), indent=1)
    print(f"capture_wrist: {n} frames, {info['fps_measured']} fps", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
