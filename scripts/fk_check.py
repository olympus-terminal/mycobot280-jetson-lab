#!/usr/bin/env python3
"""Check the official URDF's forward kinematics against real (angles, get_coords) pairs from the arm.

The URDF angle convention may differ from the controller's (per-joint sign and/or 90° offsets),
so this searches sign x offset combinations for J2..J6 (J1 fixed at identity), plus a constant
base translation, and reports the best fit's position error per pose.

Usage (laptop or Jetson, needs numpy):
    python scripts/fk_check.py [--urdf models/mycobot_280_arduino/mycobot_280_arduino.urdf] [--pairs config/pose_pairs_20260924.csv]
"""
import argparse
import csv
import itertools
import xml.etree.ElementTree as ET

import numpy as np

CHAIN = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
         "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = np.cos(q), np.sin(q), 1 - np.cos(q)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def load_chain(path):
    root = ET.parse(path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    chain = []
    for name in CHAIN:
        j = joints[name]
        o = j.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz").split()])
        rpy = [float(v) for v in o.get("rpy").split()]
        axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rpy_matrix(*rpy), xyz
        chain.append((name, T, axis))
    return chain


def fk(chain, q_rad):
    T = np.eye(4)
    for (name, To, axis), q in zip(chain, q_rad):
        R = np.eye(4)
        R[:3, :3] = axis_angle(axis, q)
        T = T @ To @ R
    return T  # flange frame in the URDF root (joint1 link) frame


def load_pairs(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(line for line in f if not line.startswith("#"))]
    ang = np.array([[float(r[f"j{i}"]) for i in range(1, 7)] for r in rows])
    xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rows])
    return [r["label"] for r in rows], ang, xyz


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default="models/mycobot_280_arduino/mycobot_280_arduino.urdf")
    ap.add_argument("--pairs", default="config/pose_pairs_20260924.csv")
    args = ap.parse_args()

    chain = load_chain(args.urdf)
    labels, ang, xyz = load_pairs(args.pairs)
    print(f"URDF: {args.urdf}\npairs: {args.pairs} ({len(labels)} poses)")

    results = []
    options = [(s, o) for s in (1, -1) for o in (0, 90, -90, 180)]
    for combo in itertools.product(options, repeat=5):
        signs = np.array([1] + [c[0] for c in combo])
        offs = np.array([0] + [c[1] for c in combo])
        q = np.radians(ang * signs + offs)
        p = np.array([fk(chain, qi)[:3, 3] * 1000 for qi in q])  # mm
        # allow a rotation of the base about z by 0/90/180/270 and a constant translation
        for k in range(4):
            Rz = axis_angle(np.array([0, 0, 1.0]), k * np.pi / 2)
            pk = p @ Rz.T
            t = (xyz - pk).mean(axis=0)
            err = np.linalg.norm(pk + t - xyz, axis=1)
            results.append((np.sqrt((err ** 2).mean()), signs, offs, k * 90, t, err))
    results.sort(key=lambda r: r[0])
    for rms, signs, offs, rot, t, err in results[:3]:
        print(f"\nRMS {rms:.1f} mm  signs {signs.tolist()}  offsets {offs.tolist()} deg  base rot {rot}  base shift {np.round(t, 1).tolist()} mm")
        for lab, e in zip(labels, err):
            print(f"   {lab:22s} {e:6.1f} mm")


if __name__ == "__main__":
    main()
