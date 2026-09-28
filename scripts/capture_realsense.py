#!/usr/bin/env python3
"""Record the RealSense D435i: color MP4 + 16-bit depth PNGs (aligned to color) + per-frame timestamps.

Runs until --seconds elapse or SIGTERM/SIGINT (the recording orchestrator sends SIGTERM).

Output in --out:
  realsense_color.mp4          color video (mp4v)
  realsense_depth/NNNNNN.png   uint16 depth in mm-units of depth_scale (see realsense_info.json), aligned to color
  realsense_frames.csv         frame, t_host (time.time()), color_ts_ms, depth_ts_ms (device clock), color_frame_no
  realsense_info.json          intrinsics, depth scale, versions, measured fps

Rough live view (not recorded data): about every PREVIEW_S the colour frame is shrunk to 320x180 and handed to a nice-19
daemon thread that writes /dev/shm/realsense_preview.jpg (JPEG q40, atomic replace) plus a self-refreshing
/dev/shm/realsense_preview.html (open in a browser). The capture loop never waits for it: one-slot hand-off, frames
dropped when the thread is busy, any preview error disables the preview and never touches the recording. --no-preview turns it off.

Usage (on the laptop):
    ~/venvs/realsense/bin/python scripts/capture_realsense.py --out data/test --seconds 5
"""
import argparse
import csv
import json
import os
import queue
import signal
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

STOP = False
PREVIEW_S = 0.5
PREVIEW_JPG = "/dev/shm/realsense_preview.jpg"
PREVIEW_HTML = "/dev/shm/realsense_preview.html"


def _stop(*_):
    global STOP
    STOP = True


def depth_writer(q, folder):
    while True:
        item = q.get()
        if item is None:
            return
        n, d = item
        cv2.imwrite(os.path.join(folder, f"{n:06d}.png"), d, [cv2.IMWRITE_PNG_COMPRESSION, 1])


def preview_writer(q, label):
    """Low-priority writer for the rough live view; exits quietly on any error."""
    try:
        os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 19)  # this thread only
        with open(PREVIEW_HTML, "w") as f:
            f.write('<meta http-equiv="refresh" content="1"><body style="margin:0;background:#111">'
                    '<img src="realsense_preview.jpg" style="width:100%;max-width:960px"></body>')
        while True:
            img = q.get()
            if img is None:
                return
            cv2.putText(img, time.strftime("%H:%M:%S ") + label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 40])
            if ok:
                with open(PREVIEW_JPG + ".tmp", "wb") as f:
                    f.write(buf.tobytes())
                os.replace(PREVIEW_JPG + ".tmp", PREVIEW_JPG)
    except Exception as e:
        print(f"capture_realsense: preview disabled ({e})", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=0, help="0 = until SIGTERM")
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--no-preview", action="store_true", help="don't write the rough live view to /dev/shm")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    os.makedirs(args.out, exist_ok=True)
    dfolder = os.path.join(args.out, "realsense_depth")
    if not args.no_depth:
        os.makedirs(dfolder, exist_ok=True)

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    if not args.no_depth:
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, args.fps)
    prof = pipe.start(cfg)
    dev = prof.get_device()
    align = rs.align(rs.stream.color)
    cintr = prof.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
    info = {"device": dev.get_info(rs.camera_info.name), "serial": dev.get_info(rs.camera_info.serial_number),
            "firmware": dev.get_info(rs.camera_info.firmware_version), "pyrealsense2": rs.__version__,
            "opencv": cv2.__version__, "color": [args.width, args.height, args.fps],
            "color_intrinsics": {"fx": cintr.fx, "fy": cintr.fy, "ppx": cintr.ppx, "ppy": cintr.ppy,
                                 "model": str(cintr.model), "coeffs": list(cintr.coeffs)},
            "depth_aligned_to_color": not args.no_depth,
            "depth_scale_m": dev.first_depth_sensor().get_depth_scale() if not args.no_depth else None,
            "started": time.time()}
    writer = cv2.VideoWriter(os.path.join(args.out, "realsense_color.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (args.width, args.height))
    q = queue.Queue(maxsize=120)
    th = threading.Thread(target=depth_writer, args=(q, dfolder), daemon=True)
    th.start()
    pq, pth, t_prev = None, None, 0.0
    if not args.no_preview:
        pq = queue.Queue(maxsize=1)
        pth = threading.Thread(target=preview_writer, args=(pq, "REC " + os.path.basename(os.path.normpath(args.out))),
                               daemon=True)
        pth.start()
    dropped = 0
    n, t0 = 0, time.time()
    # written now AND at the end: pipe.stop() sometimes hangs and the recorder then kills this process (2026-09-25)
    json.dump(info, open(os.path.join(args.out, "realsense_info.json"), "w"), indent=1)
    print(f"capture_realsense: {info['device']} {info['serial']} color {args.width}x{args.height}@{args.fps}", flush=True)
    try:
        with open(os.path.join(args.out, "realsense_frames.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["frame", "t_host", "color_ts_ms", "depth_ts_ms", "color_frame_no"])
            while not STOP and (args.seconds <= 0 or time.time() - t0 < args.seconds):
                fs = pipe.wait_for_frames()
                t_host = time.time()
                if not args.no_depth:
                    fs = align.process(fs)
                c = fs.get_color_frame()
                d = fs.get_depth_frame() if not args.no_depth else None
                if not c:
                    continue
                writer.write(np.asanyarray(c.get_data()))
                if pq is not None and t_host - t_prev >= PREVIEW_S:
                    t_prev = t_host
                    try:
                        if pth.is_alive():
                            pq.put_nowait(cv2.resize(np.asanyarray(c.get_data()), (320, 180), interpolation=cv2.INTER_AREA))
                        else:
                            pq = None
                    except Exception:  # Full (thread busy) or anything else: skip this preview frame
                        pass
                if d:
                    try:
                        q.put_nowait((n, np.asanyarray(d.get_data()).copy()))
                    except queue.Full:
                        dropped += 1
                wr.writerow([n, f"{t_host:.4f}", f"{c.get_timestamp():.3f}", f"{d.get_timestamp():.3f}" if d else "",
                             c.get_frame_number()])
                n += 1
    finally:
        info.update({"frames": n, "stopped_capturing": time.time()})
        json.dump(info, open(os.path.join(args.out, "realsense_info.json"), "w"), indent=1)   # before the stop that can hang
        pipe.stop()
        writer.release()
        q.put(None)
        th.join()
        if pq is not None:
            try:
                pq.put_nowait(None)
            except Exception:
                pass
    info.update({"frames": n, "depth_frames_dropped": dropped, "stopped": time.time(),
                 "fps_measured": round(n / max(time.time() - t0, 1e-6), 2)})
    json.dump(info, open(os.path.join(args.out, "realsense_info.json"), "w"), indent=1)
    print(f"capture_realsense: {n} frames, {info['fps_measured']} fps, depth dropped {dropped}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
