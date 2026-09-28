#!/usr/bin/env python3
"""Precise fingertip positioning on the far side with joint-encoder feedback (laptop orchestrates the Jetson).

The arm lands ~5-13 mm off a commanded working-tilt pose (servo sag under load). This commands the pose for a target
fingertip point (x, y, z in the controller frame), reads the measured joint angles, computes the measured fingertip
(validated FK), shifts the command by the error, and repeats (max N) until |error| <= tol mm.
Moves are sent as tiny movement files through run_movement.py (all its safety checks apply; start_pose = the current
measured angles, so only small corrections are ever sent).

Usage (library):  import precise; precise.goto((x, y, z), speed=6) -> (measured_tip, cmd_xyz, angles)
CLI:              python scripts/precise.py X Y Z [--tol 2] [--speed 6]
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402
import gen_movements as G  # noqa: E402
import kinematics as K  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
JET = os.environ.get("JETSON_HOST", "nvidia@jetson-orin.local")   # ssh target of the Jetson (set JETSON_HOST)
SSH = ["ssh", "-o", "BatchMode=yes"]
PY = "~/venvs/mycobot/bin/python"


def angles():
    code = ("from pymycobot import MyCobot280\nimport time\nmc=MyCobot280('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0','1000000')\n"
            "for _ in range(5):\n a=mc.get_angles()\n if isinstance(a,list) and len(a)==6: print(*a); break\n time.sleep(0.1)")
    out = subprocess.run(SSH + [JET, f"{PY} -c \"{code}\""], capture_output=True, text=True, timeout=30).stdout.split()
    return np.array([float(v) for v in out])


def pose_xyz(p):
    r, th = float(np.hypot(p[0], p[1])), float(np.degrees(np.arctan2(p[1], p[0])))
    return F.pose(r, th, float(p[2]))


def move_to(q_cmd, speed, label="precise"):
    cur = angles()
    steps = [G.A(cur, label="from"), G.A(q_cmd, speed, label)]
    ok, why = G.path_ok(cur, np.array(q_cmd), G.TIP_MIN_GRASP)
    if not ok:
        raise RuntimeError(f"path check failed: {why}")
    mv = {"name": "precise_step", "description": "precise.py correction step", "created": "2026-09-25",
          "start_pose": [round(float(v), 2) for v in cur], "units": "degrees", "steps": steps}
    path = os.path.join(REPO, "movements", "far", "precise_step.json")
    json.dump(mv, open(path, "w"), indent=1)
    subprocess.run(["rsync", "-a", path, f"{JET}:~/orin-mycobot/movements/far/"], check=True, capture_output=True)
    r = subprocess.run(SSH + [JET, f"cd ~/orin-mycobot && {PY} scripts/run_movement.py movements/far/precise_step.json"],
                       capture_output=True, text=True, timeout=120)
    if "refusing" in r.stdout or "STOP" in r.stdout or r.returncode:
        raise RuntimeError(f"runner: {r.stdout[-400:]} {r.stderr[-200:]}")


def goto(target, tol=2.0, speed=6, iters=4, verbose=True):
    target = np.asarray(target, float)
    cmd = target.copy()
    for k in range(iters):
        move_to(pose_xyz(cmd), speed)
        q = angles()
        tip = K.fingertip(q)[0]
        err = target - tip
        if verbose:
            print(f"  iter {k}: cmd {np.round(cmd, 1).tolist()} -> measured tip {np.round(tip, 1).tolist()}  err {np.round(err, 1).tolist()} (|{np.linalg.norm(err):.1f}| mm)")
        if np.linalg.norm(err) <= tol:
            break
        cmd = cmd + err
    return tip, cmd, q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xyz", nargs=3, type=float)
    ap.add_argument("--tol", type=float, default=2.0)
    ap.add_argument("--speed", type=int, default=6)
    a = ap.parse_args()
    tip, cmd, q = goto(a.xyz, a.tol, a.speed)
    print(json.dumps({"measured_tip": np.round(tip, 1).tolist(), "final_cmd": np.round(cmd, 1).tolist(), "angles": np.round(q, 2).tolist()}))
    return 0


def gripper(state):
    """state 0 open / 1 close; returns the gripper value after 3 s."""
    code = ("from pymycobot import MyCobot280\nimport time\nmc=MyCobot280('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0','1000000')\n"
            f"mc.set_gripper_state({int(state)},30); time.sleep(3); print(mc.get_gripper_value())")
    out = subprocess.run(SSH + [JET, f"{PY} -c \"{code}\""], capture_output=True, text=True, timeout=30).stdout.split()
    return int(out[-1]) if out else None


def probe(x, y, z_start=60.0, z_end=25.0, step=5.0, verbose=True):
    """Touch probe with the (closed) gripper: hover at (x, y, z_start) with encoder feedback, then step down keeping the
    xy correction; contact = a commanded 5 mm step moves the measured tip < 2 mm. Returns ('contact'|'free', measured z,
    measured xy). Lifts back to z_start afterwards."""
    tip, cmd, q = goto((x, y, z_start), tol=2.0, verbose=False)
    off = cmd - np.array([x, y, z_start])
    prev = tip[2]
    result = ("free", tip[2], tip[:2])
    z = z_start
    while z - step >= z_end - 1e-6:
        z -= step
        move_to(pose_xyz(np.array([x, y, z]) + off), 5, "probe_down")
        t = K.fingertip(angles())[0]
        if verbose:
            print(f"    probe ({x:.0f},{y:.0f}) cmd z {z:.0f}: measured z {t[2]:.1f} (moved {prev - t[2]:.1f})")
        if prev - t[2] < 2.0:
            result = ("contact", t[2], t[:2])
            break
        prev = t[2]
        result = ("free", t[2], t[:2])
    move_to(pose_xyz(np.array([x, y, z_start]) + off), 6, "probe_up")
    return result


if __name__ == "__main__":
    sys.exit(main())
