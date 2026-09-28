#!/usr/bin/env python3
"""One pyramid cycle, run ON THE JETSON (scripts/arm_local.py): from REST -> far side -> find block by touch near a
camera estimate -> grasp -> place in slot -> back to REST (parked, released). Prints a JSON summary line "SUMMARY {...}".

Usage (Jetson): ~/venvs/mycobot/bin/python scripts/cycle_local.py EST_X EST_Y SLOT_R SLOT_TH LEVEL --log FILE.csv
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402
import kinematics as K  # noqa: E402
from arm_local import Arm, Overheat, j6_for_yaw, j6_picker, pose_xyz  # noqa: E402

START_MAX_C = 52   # 2026-09-27: was 45 (would refuse demo_loop starts at J5 46-50); servo cutoff 70, ABORT_C 60
ON_BLOCK = (26.0, 48.0)
GRASP_Z = K.HAND_FLOOR + 15
G_TIP_MIN = 15.0   # gen_movements.TIP_MIN_GRASP


def wrist_look(label, log_path, arm, tip):
    """2026-09-27: hand-camera look (data for grasp alignment): a wrist frame taken after the arm settled, tag
    detections, the measured fingertip and J6. The frame is saved next to the wrist video (record_cycle syncs it)."""
    time.sleep(0.4)
    t0 = time.time()
    save = log_path.replace("_joints.csv", "_wrist") + f"/look_{label}.jpg" if log_path and log_path.endswith("_joints.csv") else None
    cmd = ["/usr/bin/python3", os.path.join(os.path.dirname(os.path.abspath(__file__)), "wrist_look.py"), "--after", f"{t0:.3f}"]
    try:
        r = subprocess.run(cmd + (["--save", save] if save else []), capture_output=True, text=True, timeout=8)
        d = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception as e:  # never let a look break the cycle
        d = {"ok": False, "error": f"{type(e).__name__}: {e}"}
    d.update({"label": label, "tip": np.round(tip, 1).tolist(), "j6": round(float(arm.angles()[5]), 1)})
    print(f"  look {label}: {[(t['id'], t['center']) for t in d.get('tags', [])]}", flush=True)
    return d


def jaw_world(xy, z, j6, jaw_xy, reach=None):
    """World-xy offset of the jaw centre from the model fingertip at the pose reaching (xy, z) with J6 = j6."""
    q = pose_xyz(np.array([xy[0], xy[1], z]), reach)
    if j6 is not None:
        q[5] = j6
    Fl = K.fk_frames(q)[-1]
    return jaw_xy[0] * Fl[:2, 0] + jaw_xy[1] * Fl[:2, 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ex", type=float)
    ap.add_argument("ey", type=float)
    ap.add_argument("sr", type=float)
    ap.add_argument("sth", type=float)
    ap.add_argument("level", type=int)
    ap.add_argument("--picker", action="store_true", help="vertical-tool 'picker' IK branch (far_side.PICKER)")
    ap.add_argument("--log")
    ap.add_argument("--bias", nargs=2, type=float, default=[0.0, 0.0], help="xy mm added to the camera estimate (learned)")
    ap.add_argument("--max-attempts", type=int, default=13)
    ap.add_argument("--side", choices=["far", "near"], default="far")
    ap.add_argument("--jaw-t", type=float, default=0.0,
                    help="mm: the real jaw centre sits this far on the -tangential side of the model fingertip point "
                         "(2026-09-25: ~15; every missed grasp pushed the block +tangential). Grasp aims at est + jaw_t*v, "
                         "place aims at slot + jaw_t*v so the BLOCK lands on the slot")
    ap.add_argument("--jaw-xy", nargs=2, type=float, default=[0.0, 0.0],
                    help="picker: jaw centre in the FLANGE frame (x, y mm) relative to the model fingertip "
                         "(2026-09-25 20:12 guided grasp: ~(+7, +18)); grasp aims tip = block - offset, place tip = slot - offset")
    ap.add_argument("--place-bias-rt", nargs=2, type=float, default=[0.0, 0.0],
                    help="mm (radial, tangential) added to the slot aim: measured systematic placement error correction")
    ap.add_argument("--release-to", type=float, default=0.0, help="place: open only to this gripper value (e.g. 50) instead of fully")
    ap.add_argument("--grasp-z", type=float, help="fingertip-point z for the working-tilt grasp (default HAND_FLOOR + 15 = 20)")
    ap.add_argument("--yaw", type=float, help="block top-tag yaw (arm frame, deg mod 90, find_blocks): align the fingers via J6")
    ap.add_argument("--drop-z", type=float, default=0.0,
                    help="dice roll (2026-09-26, user: 'lift up and drop like a dice'): carry the block to the slot at this "
                         "tip height and snap the gripper open there instead of placing it")
    ap.add_argument("--place-j6", type=float, default=None,
                    help="2026-09-27: J6 during the place (default: IK's 0) -> the block is set down rotated (variation)")
    ap.add_argument("--release-speed", type=int, default=30,
                    help="gripper speed for the partial release (--release-to); 2026-09-27: 10 for stacking")
    ap.add_argument("--square-grip-max", type=int, default=0,
                    help="stacking: re-seat the block (set down, open, close) until the grip value is <= this (0 = off)")
    ap.add_argument("--place-only", action="store_true",
                    help="2026-09-27: skip the grasp (the block is already held, e.g. after --no-place + an in-hand measurement)")
    ap.add_argument("--wrist-look", action="store_true",
                    help="2026-09-27: at each grasp attempt, save/analyse a hand-camera frame at the hover and before closing")
    ap.add_argument("--no-place", action="store_true", help="keep holding the block at home after the grasp (for a calibration tour)")
    a = ap.parse_args()
    if a.picker:
        import far_side as _F
        _F.PICKER = True
    S = {"start": datetime.datetime.now().isoformat(timespec="seconds"), "estimate": [a.ex, a.ey], "slot": [a.sr, a.sth, a.level],
         "probes": []}
    arm = Arm(a.log)
    t_start = time.time()
    try:
        t = arm.temps()
        S["temps_start"] = None if t is None else t.tolist()
        if t is not None and t.max() > START_MAX_C and not a.place_only:
            S["result"] = f"too warm to start {t.tolist()}"
            return 5
        # --- from REST (released) to far home
        q = arm.angles()
        if K.fingertip(q)[0][2] < 50:
            arm.unpark()
        q = arm.angles()
        if a.side == "near":
            pass
        elif abs(q[0] - arm.H_far[0]) < 25 and K.fingertip(q)[0][2] > 50:
            arm.move(arm.H_far, 10, "to_far_home")          # already raised over the far side
        else:
            s_near = np.array([70.8, -32.0, -33.6, -49.8, 50.6, 20.0])
            arm.move(s_near, 10, "to_far_raise")
            s_far = s_near.copy()
            s_far[0] = arm.H_far[0]
            arm.move(s_far, 12, "to_far_rotate")
            arm.move(arm.H_far, 10, "to_far_home")

        if a.place_only:
            # 2026-09-27: the block is already held (a previous --no-place run + a camera in-hand measurement): place it
            g, val, j6 = None, None, None
            S["place_only"] = True
            S["grasp_xy"] = [a.ex, a.ey]   # the caller passes the earlier grasp aim: the put-back spot on error
        else:
            est = np.array([a.ex, a.ey]) + np.array(a.bias)
            u = est / np.linalg.norm(est)
            v = np.array([-u[1], u[0]])
            est = est + a.jaw_t * v
            # Grasp search: probe-by-grasping. Candidates (radial, tangential) mm around the (bias-corrected) estimate.
            cands = [(0, 0), (8, 0), (-8, 0), (0, 12), (0, -12), (8, 12), (8, -12), (-8, 12), (-8, -12), (16, 0), (-16, 0), (0, 24), (0, -24)]
            val, g = None, None
            S["attempts"] = []
            for dr, dt in cands[: a.max_attempts]:
                g = est + dr * u + dt * v
                # IK branch for the whole attempt: far-reach (hand floor 21, ~20 mm sag) beyond REACH_R, or wherever the
                # working tilt can't reach every height of the attempt (e.g. r 236 at z 80)
                reach = None
                for br in ([False] if a.picker else [True] if np.hypot(g[0], g[1]) > F.REACH_R else [False, True]):
                    gz = F.GRASP_Z_REACH if br else (GRASP_Z if a.grasp_z is None else a.grasp_z)
                    try:
                        for zz in (gz, gz + 20, gz + 40, gz + 60):
                            pose_xyz(np.array([g[0], g[1], zz]), br)
                        reach = br
                        break
                    except ValueError:
                        continue
                if reach is None:
                    print(f"attempt off {dr:+d},{dt:+d}: unreachable, skipped", flush=True)
                    continue
                j6 = None
                if a.yaw is not None:
                    j6, mis = j6_picker((g[0], g[1], gz), a.yaw) if a.picker else j6_for_yaw((g[0], g[1], gz), a.yaw, reach)
                    print(f"  yaw {a.yaw:.0f}: J6 {j6:.0f} (residual misalignment {mis:+.0f} deg)", flush=True)
                if a.picker and any(a.jaw_xy):     # aim the jaw centre (not the model fingertip) at the block
                    blk = g.copy()
                    for _ in range(2):
                        g = blk - jaw_world(g, gz, j6, a.jaw_xy)
                    print(f"  jaw offset: tip aim {np.round(g, 1).tolist()} for block {np.round(blk, 1).tolist()}", flush=True)
                    try:                              # the jaw offset can push the aim out of reach: re-check every height
                        for zz in (gz, gz + 20, gz + 40, gz + 60):
                            pose_xyz(np.array([g[0], g[1], zz]), reach)
                    except ValueError:
                        print(f"attempt off {dr:+d},{dt:+d}: jaw-adjusted aim unreachable, skipped", flush=True)
                        continue
                arm.gripper(0)
                zh = gz + 40
                t_h, c_h = arm.goto((g[0], g[1], zh), tol=3.0, label="gs_hover", reach=reach, j6=j6)
                k_att = len(S["attempts"])
                if a.wrist_look:
                    S.setdefault("wrist_looks", []).append(wrist_look(f"a{k_att}_hover", a.log, arm, t_h))
                t_m, c_m = arm.goto((g[0], g[1], gz + 20), tol=3.0, label="gs_mid", reach=reach, init_off=c_h - np.array([g[0], g[1], zh]), j6=j6)
                tip, _ = arm.goto((g[0], g[1], gz), tol=3.0, speed=5, label="gs_low", iters=2, reach=reach,
                                  init_off=c_m - np.array([g[0], g[1], gz + 20]), j6=j6, tip_min=min(G_TIP_MIN, gz - 8),
                                  stop_above=8.0)
                if a.wrist_look:
                    S["wrist_looks"].append(wrist_look(f"a{k_att}_low", a.log, arm, tip))
                blocked = tip[2] > gz + 10    # a finger landed on the block top
                val = None
                if not blocked:
                    # the open fingertips reach lower than closed ones: at the grasp height they can drag on the table and
                    # fail to close (reads ~100). Lift a little and close again.
                    for dz in (0.0, 4.0, 8.0):
                        if dz:
                            tip, _ = arm.goto((g[0], g[1], gz + dz), tol=3.0, speed=4, label="gs_relift", iters=1, reach=reach, j6=j6)
                        val = arm.gripper(1)
                        if val is not None and val < 85:
                            break
                        print(f"  close had no effect at z {gz + dz:.0f} (value {val}): lifting", flush=True)
                S["attempts"].append({"off": [dr, dt], "xy": np.round(g, 1).tolist(), "tip": np.round(tip, 1).tolist(), "blocked": bool(blocked), "value": val})
                print(f"attempt off {dr:+d},{dt:+d} at {np.round(g, 1).tolist()}: tip z {tip[2]:.1f} {'BLOCKED' if blocked else f'gripper {val}'}", flush=True)
                arm.goto((g[0], g[1], gz + 60), tol=4, label="gs_lift", iters=2, reach=reach, j6=j6)
                if val and 12 < val < 70:
                    break
            if not (val and 12 < val < 70):
                S["result"] = "grasp search failed"
                arm.safe_exit("grasp search failed")
                return 3
            S["grasp_xy"] = np.round(g, 1).tolist()
            if a.level > 0 and a.square_grip_max and val > a.square_grip_max:
                # 2026-09-27 tower 4: grip 37 (a slightly skewed hold) -> the top block twisted off at the release; tower 2 (29)
                # stood. Re-seat: set it down where it was grasped, open, close again (the fingers square it up), check.
                for k in range(2):
                    tip, _ = arm.goto((g[0], g[1], gz + 2.0), tol=3.0, speed=4, iters=1, j6=j6, label="reseat_down",
                                      tip_min=min(G_TIP_MIN, gz - 8), stop_above=8.0)   # 18:16: default floor 15 tripped
                    arm.gripper_to(70)
                    val = arm.gripper(1)
                    print(f"  re-seat {k}: grip {val}", flush=True)
                    S.setdefault("reseat", []).append(val)
                    if val is not None and 12 < val <= a.square_grip_max:
                        break
                if not (val is not None and 12 < val < 70):
                    S["result"] = "grasp lost while re-seating"
                    arm.safe_exit("re-seat lost the block")
                    return 3
                if val > a.square_grip_max:
                    # 18:22: two re-seats left it at 41 and the skewed block knocked the tower down -> never stack a skewed
                    # grip: set it down where it is, release, park
                    arm.gripper_to(70)
                    arm.goto((g[0], g[1], gz + 60), tol=4, label="skew_clear", iters=1, j6=j6)
                    S["result"] = f"grip not square ({val}) - not stacked"
                    arm.safe_exit("grip not square")
                    return 6
                arm.goto((g[0], g[1], gz + 60), tol=4, label="gs_lift", iters=2, j6=j6)
        if a.no_place:
            arm.move(arm.H_near if a.side == "near" else arm.H_far, 10, "hold_home")
            S["result"] = "holding at home"
            return 0
        if g is not None:
            S["learned_bias"] = np.round(g - np.array([a.ex, a.ey]), 1).tolist()
        slot = np.array([a.sr * np.cos(np.radians(a.sth)), a.sr * np.sin(np.radians(a.sth))])
        us = slot / np.linalg.norm(slot)
        slot = slot + a.place_bias_rt[0] * us + a.place_bias_rt[1] * np.array([-us[1], us[0]])
        slot = slot + a.jaw_t * np.array([-us[1], us[0]])
        if a.picker and any(a.jaw_xy):         # the block lands at the jaw centre: put that on the slot
            sl = slot.copy()
            for _ in range(2):
                slot = sl - jaw_world(slot, 30.0, a.place_j6, a.jaw_xy)
        # place height = the grasp height (+2 mm) + 30 per level: the block sits in the fingers as it was grasped
        zp = F.zgrasp(a.level, place=True) if a.grasp_z is None else a.grasp_z + 2.0 + K.BLOCK * a.level
        S["slot_tip_target"] = np.round(slot, 1).tolist()
        zh = max(80.0, zp + 40)
        j1_slot = pose_xyz(np.array([slot[0], slot[1], zh]))[0]
        if abs(j1_slot - arm.angles()[0]) > 60:     # other side of the base: cross over raised
            s_raise = np.array([70.8, -32.0, -33.6, -49.8, 50.6, 20.0])
            s_raise[0] = arm.angles()[0]
            arm.move(s_raise, 10, "cross_raise")
            s_raise[0] = j1_slot
            arm.move(s_raise, 12, "cross_rotate")
        # 2026-09-26: carry each stage's converged sag correction into the next (as the grasp path does): fewer
        # 1-2 s correction steps ("twitching") on the way down
        if a.drop_z > 0:
            tip, _ = arm.goto((slot[0], slot[1], a.drop_z), tol=6.0, iters=2, label="drop_hover", j6=a.place_j6)
            S["release_value"] = arm.gripper_to(100, speed=100)
            S["dropped_from_z"] = a.drop_z
        else:
            _, c_h = arm.goto((slot[0], slot[1], zh), tol=2.5, label="slot_hover", j6=a.place_j6)
            # path floor: the default TIP_MIN_GRASP (15) was sized for grasp z 13; at grasp z 5 (2026-09-26 20:18) the
            # near stage (zp + 12 = 19) dips below it (20:31 path-check error). zp - 5 (09-27 10:11: zp - 2 still tripped at 4 mm) keeps the held block's bottom
            # (~20 mm below the fingertip) at or above the table
            _, c_n = arm.goto((slot[0], slot[1], zp + 12), tol=2.5, speed=5, label="slot_near", j6=a.place_j6,
                              init_off=c_h - np.array([slot[0], slot[1], zh]), tip_min=min(G_TIP_MIN, zp - 5))
            tip, _ = arm.goto((slot[0], slot[1], zp), tol=2.0, speed=4, label="slot_place", j6=a.place_j6, tip_min=min(G_TIP_MIN, zp - 16),   # 09-27 10:30: -12 tripped at -6 on the far side (command includes the sag; measured tip fine)
                              init_off=c_n - np.array([slot[0], slot[1], zp + 12]),
                              stop_above=8.0)   # 09-27 16:40: far side θ -60: measured tip stuck at z 20 (target 7) while
                                                # the sag loop pushed the command to the -9 floor -> release where it rests
            S["release_value"] = arm.gripper_to(a.release_to, speed=a.release_speed) if a.release_to > 0 else arm.gripper(0)
        S["placed_tip"] = np.round(tip, 1).tolist()
        if a.drop_z <= 0:     # after a drop the hand is already above the block: straight home
            if a.level > 0:   # 09-27 tower test 1: the released top block got nudged off -> first a slow straight lift
                arm.goto((slot[0], slot[1], zp + 25), tol=4, speed=3, iters=1, label="slot_lift_slow", j6=a.place_j6)
            arm.goto((slot[0], slot[1], zh), tol=4, label="slot_clear", j6=a.place_j6)
        if arm.angles()[0] < 0:
            arm.move(arm.H_far, 10, "home_after_place")
            arm.park_near()
        else:
            arm.move(arm.H_near, 10, "home_after_place")
            arm.park_rest_from_home()
        S["result"] = "dropped" if a.drop_z > 0 else "placed"
        return 0
    except Overheat as e:
        S["result"] = f"overheat: {e}"
        arm.safe_exit("overheat")
        return 4
    except Exception as e:  # report, then try to leave the arm safe
        S["result"] = f"error: {type(e).__name__}: {e}"
        if "grasp_xy" in S and "release_value" not in S and a.grasp_z is not None:
            # 2026-09-26: an error while HOLDING a block used to end in release_all at REST -> the block fell out and
            # usually landed picture-up (lost to the camera). Put it back where it was grasped (reachable by definition),
            # same J6, then the normal safe exit. Any failure here falls through to safe_exit as before.
            try:
                gx, gy = S["grasp_xy"]
                j6h = float(arm.angles()[5])
                arm.goto((gx, gy, 80.0), tol=5.0, iters=2, label="putback_hover", j6=j6h)
                arm.goto((gx, gy, a.grasp_z + 2.0), tol=4.0, iters=2, speed=4, label="putback_place", j6=j6h,
                         tip_min=min(G_TIP_MIN, a.grasp_z - 10.0))
                S["putback_release"] = arm.gripper_to(55)
                arm.goto((gx, gy, 80.0), tol=6.0, iters=1, label="putback_clear", j6=j6h)
                S["put_back"] = True
            except Exception as e3:
                S["put_back_error"] = f"{type(e3).__name__}: {e3}"
        try:
            arm.safe_exit("error")
        except Exception as e2:
            S["safe_exit_error"] = f"{type(e2).__name__}: {e2}"
        return 1
    finally:
        S["seconds"] = round(time.time() - t_start, 1)
        t = arm.temps()
        S["temps_end"] = None if t is None else t.tolist()
        print("SUMMARY " + json.dumps(S), flush=True)
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
