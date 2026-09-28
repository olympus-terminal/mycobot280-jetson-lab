#!/usr/bin/env python3
"""Stream check for the RealSense D435i: device info, depth + color frame rates, depth sanity, IMU.

Saves nothing; prints measurements only.

Usage (on the host the camera is plugged into, currently the laptop):
    ~/venvs/realsense/bin/python scripts/probe_realsense.py [--seconds 5] [--imu]
"""
import argparse
import datetime
import platform
import sys
import time

import numpy as np
import pyrealsense2 as rs

DEPTH = (848, 480, 30)   # D435 recommended depth resolution
COLOR = (1280, 720, 30)


def device_info(dev):
    keys = ["name", "serial_number", "asic_serial_number", "firmware_version", "usb_type_descriptor", "product_line"]
    info = {}
    for k in keys:
        attr = getattr(rs.camera_info, k, None)
        if attr is not None and dev.supports(attr):
            info[k] = dev.get_info(attr)
    return info


def stream_test(seconds):
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, DEPTH[0], DEPTH[1], rs.format.z16, DEPTH[2])
    cfg.enable_stream(rs.stream.color, COLOR[0], COLOR[1], rs.format.bgr8, COLOR[2])
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    try:
        for _ in range(30):  # let auto-exposure settle
            pipe.wait_for_frames()
        n, t0 = 0, time.time()
        depth_frame = None
        while time.time() - t0 < seconds:
            fs = pipe.wait_for_frames()
            depth_frame, color_frame = fs.get_depth_frame(), fs.get_color_frame()
            if depth_frame and color_frame:
                n += 1
        elapsed = time.time() - t0
        d = np.asanyarray(depth_frame.get_data()).astype(np.float32) * scale
        h, w = d.shape
        centre = d[h // 2 - 5:h // 2 + 5, w // 2 - 5:w // 2 + 5]
        centre = centre[centre > 0]
        print(f"depth {DEPTH[0]}x{DEPTH[1]}@{DEPTH[2]} + color {COLOR[0]}x{COLOR[1]}@{COLOR[2]}")
        print(f"framesets: {n} in {elapsed:.1f} s = {n / elapsed:.1f} fps")
        print(f"depth scale: {scale} m/unit")
        print(f"valid depth pixels (last frame): {100 * np.count_nonzero(d) / d.size:.1f}%")
        print(f"centre 10x10 median depth: {np.median(centre):.3f} m" if centre.size else "centre 10x10: no valid depth")
    finally:
        pipe.stop()


def imu_test():
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)
    try:
        pipe.start(cfg)
    except RuntimeError as exc:
        print(f"IMU: cannot start ({exc})")
        print("IMU: install Intel's udev rules (99-realsense-libusb.rules), then replug the camera")
        return
    try:
        got = {}
        t0 = time.time()
        while len(got) < 2 and time.time() - t0 < 5:
            for f in pipe.wait_for_frames():
                m = f.as_motion_frame()
                if m:
                    v = m.get_motion_data()
                    got[m.get_profile().stream_name()] = (round(v.x, 3), round(v.y, 3), round(v.z, 3))
        for k, v in got.items():
            print(f"{k}: {v}")
        if "Accel" in got:
            print(f"|accel| = {np.linalg.norm(got['Accel']):.2f} m/s^2 (~9.81 at rest)")
    finally:
        pipe.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=5)
    ap.add_argument("--imu", action="store_true", help="also read the IMU (needs hidraw access)")
    args = ap.parse_args()

    print(f"probe_realsense.py  {datetime.datetime.now().isoformat(timespec='seconds')}")
    print(f"python {platform.python_version()}  pyrealsense2 {rs.__version__}  host {platform.node()}")
    devs = rs.context().query_devices()
    if len(devs) == 0:
        print("no RealSense device found")
        return 1
    for k, v in device_info(devs[0]).items():
        print(f"{k}: {v}")
    stream_test(args.seconds)
    if args.imu:
        imu_test()
    return 0


if __name__ == "__main__":
    sys.exit(main())
