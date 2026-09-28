#!/usr/bin/env python3
"""Kinematics for the myCobot 280 Arduino in the CONTROLLER frame (the frame of get_coords).

Built on the official URDF, validated 2026-09-24 against 7 real poses (scripts/fk_check.py):
position 1.0 mm RMS, orientation 0.0 deg, controller angles used directly, controller frame = URDF root
rotated 270 deg about z, shifted BASE_SHIFT mm. The tool (gripper) axis is the flange z-axis.

IK is numerical (damped least squares, several seeds), for "fingertip at point P, gripper pointing
straight down", within firmware limits and an extra per-joint safe window (e.g. J5, to avoid the
wrist/gripper self-collision seen 2026-09-24). Plans are checked before anything moves:
fingertip and every joint origin above the table plus a margin.

Usage (plan only, no motion):
    python scripts/kinematics.py --fingertip 9 180 100 --seed 95 0 0 0 89 -1
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_check import axis_angle, load_chain  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "models", "mycobot_280_arduino", "mycobot_280_arduino.urdf")
BASE_ROT = axis_angle(np.array([0, 0, 1.0]), np.radians(270))
BASE_SHIFT = np.array([-2.6, 1.2, -0.1])  # mm, from fk_check.py fit
GRIPPER_LEN = 100.0  # mm, flange face to fingertips (user measurement 2026-09-24)
FW_LIMITS = np.array([[-168, 168], [-135, 135], [-150, 150], [-145, 145], [-165, 165], [-180, 180]], float)
SAFE = FW_LIMITS.copy()
SAFE[4] = [80, 140]  # J5: 88.85 and 138-139 were collision-free; 44 hit the gripper protrusion
TABLE_Z = 0.0  # mm, controller frame; conservative planning floor for joint frames (see HAND_FLOOR)
HAND_FLOOR = 5.0  # mm: fingertip-point z where the hand touches the table at the working tilt (touch test 2026-09-25)
BLOCK = 30.0      # mm cube; grasp/place fingertip z = HAND_FLOOR + 15 + 30*level

_CHAIN = load_chain(URDF)


def fk_frames(q_deg):
    """4x4 transforms (controller frame, mm) of each joint origin and the flange, for angles in deg."""
    frames, T = [], np.eye(4)
    for (name, To, axis), q in zip(_CHAIN, np.radians(q_deg)):
        R = np.eye(4)
        R[:3, :3] = axis_angle(axis, q)
        T = T @ To
        frames.append(T.copy())
        T = T @ R
    frames.append(T.copy())  # flange
    out = []
    for F in frames:
        G = np.eye(4)
        G[:3, :3] = BASE_ROT @ F[:3, :3]
        G[:3, 3] = BASE_ROT @ (F[:3, 3] * 1000) + BASE_SHIFT
        out.append(G)
    return out


def fingertip(q_deg):
    F = fk_frames(q_deg)[-1]
    return F[:3, 3] + GRIPPER_LEN * F[:3, 2], F[:3, 2]


def _residual(q, target_tip, down):
    tip, z = fingertip(q)
    return np.concatenate([tip - target_tip, 100.0 * (z - down)])


def ik_down(target_tip, seed, bounds=SAFE, iters=300):
    """Fingertip at target_tip (mm), tool axis pointing -z. Returns (q_deg, residual_norm)."""
    down = np.array([0, 0, -1.0])
    best = None
    rng_seeds = [np.array(seed, float)]
    for dj in ([0, 20, -20, 0, 0, 0], [0, -20, 20, 0, 0, 0], [0, 30, 30, 30, 0, 0], [0, -30, -30, -30, 0, 0]):
        rng_seeds.append(np.array(seed, float) + dj)
    for q in rng_seeds:
        q = np.clip(q, bounds[:, 0], bounds[:, 1])
        lam = 1.0
        for _ in range(iters):
            r = _residual(q, target_tip, down)
            J = np.zeros((6, 6))
            for k in range(6):
                dq = np.zeros(6)
                dq[k] = 1e-3
                J[:, k] = (_residual(q + dq, target_tip, down) - r) / 1e-3
            step = np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ r)
            qn = np.clip(q + step, bounds[:, 0], bounds[:, 1])
            if np.linalg.norm(_residual(qn, target_tip, down)) < np.linalg.norm(r):
                q, lam = qn, max(lam / 2, 1e-4)
            else:
                lam *= 4
            if np.linalg.norm(r) < 1e-3:
                break
        err = np.linalg.norm(_residual(q, target_tip, down))
        # prefer small residual, then small motion from the seed
        key = (round(err, 1), np.abs(q - seed).sum())
        if best is None or key < best[0]:
            best = (key, q, err)
    return best[1], best[2]


def ik_tool(target_tip, tool_axis, seed, bounds, iters=300):
    """Fingertip at target_tip (mm) with the tool axis along tool_axis (unit). Returns (q_deg, residual_norm)."""
    target_tip, tool_axis = np.asarray(target_tip, float), np.asarray(tool_axis, float)
    q = np.clip(np.asarray(seed, float), bounds[:, 0], bounds[:, 1])

    def res(q):
        tip, z = fingertip(q)
        return np.concatenate([tip - target_tip, 100.0 * (z - tool_axis)])

    lam = 1.0
    for _ in range(iters):
        r = res(q)
        if np.linalg.norm(r) < 1e-3:
            break
        J = np.array([(res(q + np.eye(6)[k] * 1e-3) - r) / 1e-3 for k in range(6)]).T
        qn = np.clip(q + np.linalg.solve(J.T @ J + lam * np.eye(6), -J.T @ r), bounds[:, 0], bounds[:, 1])
        if np.linalg.norm(res(qn)) < np.linalg.norm(r):
            q, lam = qn, max(lam / 2, 1e-4)
        else:
            lam *= 4
    return q, float(np.linalg.norm(res(q)))


def check_plan(q_deg, margin=20.0):
    """Return a list of problems (empty = OK): joint limits, table clearance of every joint origin and the fingertip."""
    problems = []
    for k, (v, (lo, hi)) in enumerate(zip(q_deg, SAFE), 1):
        if not lo <= v <= hi:
            problems.append(f"J{k}={v:.1f} outside safe window {lo}..{hi}")
    for k, F in enumerate(fk_frames(q_deg)[1:], 2):
        if F[2, 3] < TABLE_Z + margin:
            problems.append(f"joint/flange frame {k} z={F[2, 3]:.0f} mm below table+{margin:.0f}")
    tip, _ = fingertip(q_deg)
    if tip[2] < TABLE_Z + 5:
        problems.append(f"fingertip z={tip[2]:.0f} mm below table+5")
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fingertip", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    ap.add_argument("--seed", nargs=6, type=float, required=True)
    args = ap.parse_args()
    q, err = ik_down(np.array(args.fingertip), np.array(args.seed))
    tip, z = fingertip(q)
    print(f"target fingertip {args.fingertip} (gripper down)")
    print(f"solution angles  {np.round(q, 2).tolist()}  residual {err:.2f}")
    print(f"fingertip        {np.round(tip, 1).tolist()}  tool axis {np.round(z, 3).tolist()}")
    print(f"flange coords    {np.round(fk_frames(q)[-1][:3, 3], 1).tolist()}")
    probs = check_plan(q)
    print("checks:", "OK" if not probs else probs)
    return 0 if not probs and err < 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
