#!/usr/bin/env python3
"""Fast, local (runs ON THE JETSON) arm control for precise far-side work: one persistent serial connection.

Why: running each small step through ssh + a fresh serial open (which resets the Arduino, +1.5 s) made a touch probe
take ~50 s, and J5 overheated (56 C) before a cycle could finish. Here a step is ~1 s.

Safety (every move): firmware limits, far-side windows (far_side.G.WINDOWS: J1 -122..100, J5 50..110, J6 -10..35),
joint-space path check against the table (gen_movements.path_ok, grasp minimum), temperature (per-joint min of 3 reads)
> ABORT_C -> safe_exit() (lift, far home, raised rotation to the near side, near home, REST, release).
Joint readings are logged (t_host, event, label, j1..j6) to --log.
"""
import csv
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import far_side as F  # noqa: E402  (sets far-side windows)
import gen_movements as G  # noqa: E402
import kinematics as K  # noqa: E402
from pymycobot import MyCobot280  # noqa: E402

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ABORT_C = 62   # 2026-09-27: was 60; servo cutoff (reg 13) 70 C; J5 rises < 2 C per move step, ~10 C (max 15) per cycle
TOL_SCALE = np.array([1, 1, 1, 1, 2, 1])
REST = np.array([63.89, -105.11, -31.11, 38.93, 97.55, 34.89])
S_NEAR = np.array([70.8, -32.0, -33.6, -49.8, 50.6, 20.0])


class Overheat(RuntimeError):
    pass


class Hangup(RuntimeError):
    """The controlling ssh session died (SIGHUP) or the script was told to stop (SIGTERM). Raised inside the running
    move so the script's normal error path runs safe_exit() (2026-09-25: an interrupted laptop command killed a script
    mid-motion and left the arm hovering powered for ~15 min, J5 63 C)."""


def _raise_hangup(signum, frame):
    signal.signal(signal.SIGHUP, signal.SIG_IGN)   # the exit itself must not be interrupted again
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise Hangup(f"signal {signum}")


def _say(msg):
    """print that survives a dead ssh pipe (after SIGHUP stdout is gone)."""
    try:
        print(msg, flush=True)
    except (BrokenPipeError, OSError):
        pass


class Arm:
    def __init__(self, log_path=None):
        self.mc = MyCobot280(PORT, "1000000")
        self.lims = np.array([(self.mc.get_joint_min_angle(j), self.mc.get_joint_max_angle(j)) for j in range(1, 7)], float)
        self.logf = open(log_path, "w", newline="") if log_path else None
        self.log = csv.writer(self.logf) if self.logf else None
        if self.log:
            self.log.writerow(["t_host", "event", "label", "j1", "j2", "j3", "j4", "j5", "j6"])
        signal.signal(signal.SIGHUP, _raise_hangup)
        signal.signal(signal.SIGTERM, _raise_hangup)
        self.H_far = F.pose(180, F.FAR_THETA, 80)
        self.H_near = G.solve_rz(180, 80)[0]

    # --- low level -------------------------------------------------------------------------------------------
    def _w(self, event, label="", a=None):
        if self.log:
            self.log.writerow([f"{time.time():.4f}", event, label] + (list(a) if a is not None else [""] * 6))

    def angles(self):
        for _ in range(8):
            a = self.mc.get_angles()
            if isinstance(a, list) and len(a) == 6:
                self._w("read", "", a)
                return np.array(a, float)
            time.sleep(0.05)
        raise RuntimeError("no angle reading")

    def temps(self):
        r = [t for t in (self.mc.get_servo_temps() for _ in range(3)) if isinstance(t, list) and len(t) == 6]
        return np.min(np.array(r), 0) if r else None

    def gripper(self, state):
        """Open (0) / close (1). A command can be lost on the serial line (2026-09-25 16:03: 'close' read back 103 =
        still open, and a loose check counted it as holding): if the reading shows no effect, resend (up to 2x)."""
        v = None
        for k in range(2):
            self.mc.set_gripper_state(int(state), 30)
            time.sleep(2.2)
            v = self.mc.get_gripper_value()
            took = (v is not None and v >= 0) and ((state and v < 85) or (not state and v > 80))
            self._w("gripper", f"{'close' if state else 'open'}={v}" + ("" if took else f" (no effect, try {k + 1})"))
            if took:
                break
        return v

    def gripper_to(self, value, speed=30):
        """Open/close the gripper to a partial value (0 closed .. ~100 open): a place next to other blocks opens only
        as far as needed to let go, so the fingers don't shove the neighbours."""
        self.mc.set_gripper_value(int(value), int(speed))
        time.sleep(2.0 * max(1.0, 30.0 / max(int(speed), 1)))   # 2026-09-27: slower speeds need longer (speed 10 -> 6 s)
        v = self.mc.get_gripper_value()
        self._w("gripper", f"to{int(value)}={v}")
        return v

    def move(self, q, speed=8, label="", tip_min=None, check_heat=True, check_path=True, timeout=10.0, tol=3.0):
        q = np.asarray(q, float)
        if np.any(q < self.lims[:, 0]) or np.any(q > self.lims[:, 1]):
            raise ValueError(f"{label}: outside firmware limits {np.round(q, 1).tolist()}")
        cur = self.angles()
        if check_path:
            # measured start may sag slightly outside a window (e.g. J5 49.7 vs 50): check from the clamped start
            cur_c = np.clip(cur, G.WINDOWS[:, 0], G.WINDOWS[:, 1])
            if np.abs(cur_c - cur).max() > 4:
                raise ValueError(f"{label}: current pose {np.round(cur, 1).tolist()} is >4 deg outside the windows")
            # judge the path, not the start: the minimum is lowered to whatever the start already is (sag can put it there)
            tmin = G.TIP_MIN_GRASP if tip_min is None else tip_min
            tmin = min(tmin, K.fingertip(cur_c)[0][2] - 1.0)
            ok, why = G.path_ok(cur_c, q, tmin)
            if not ok:
                raise ValueError(f"{label}: path check failed: {why}")
        if check_heat:
            t = self.temps()
            if t is not None and t.max() > ABORT_C:
                raise Overheat(f"servo temps {t.tolist()} > {ABORT_C} C before {label}")
        self._w("send", label, q)
        self.mc.send_angles([round(float(v), 2) for v in q], int(speed), _async=True)
        t0 = last_t = time.time()
        last = None
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            a = self.mc.get_angles()
            if not (isinstance(a, list) and len(a) == 6):
                continue
            a = np.array(a, float)
            self._w("poll", label, a)
            now = time.time()
            if last is None or np.abs(a - last).max() > 0.2:
                last, last_t = a, now
                continue
            near = np.all(np.abs(a - q) <= tol * TOL_SCALE)
            if (near and now - last_t >= 0.4) or (now - t0 >= 0.8 and now - last_t >= 1.2):
                break
        return self.angles()

    # --- precise --------------------------------------------------------------------------------------------------
    def goto(self, xyz, tol=2.0, speed=6, iters=4, label="goto", reach=None, init_off=None, j6=None, tip_min=None,
             stop_above=None):
        """Encoder-feedback move of the fingertip to xyz. init_off: start from target + init_off (the sag correction a
        previous goto converged to, e.g. hover -> lower: on the far-reach branch the arm sags ~20 mm)."""
        target = np.asarray(xyz, float)
        if reach is None:   # choose the IK branch once, from the target (sag corrections must not flip it)
            reach = bool(np.hypot(target[0], target[1]) > F.REACH_R)
        cmd = target.copy() if init_off is None else target + np.asarray(init_off, float)
        tip = None
        for k in range(iters):
            try:
                qc = pose_xyz(cmd, reach)
            except ValueError:
                # the sag correction pushed the command out of reach: shrink the correction until reachable
                for frac in (0.75, 0.5, 0.25, 0.0):
                    try:
                        qc = pose_xyz(target + frac * (cmd - target), reach)
                        break
                    except ValueError:
                        continue
                else:
                    raise
            if j6 is not None:      # J6 rotates about the tool axis: the fingertip point does not move
                qc = qc.copy()
                qc[5] = j6
            q = self.move(qc, speed, f"{label}#{k}", tip_min=tip_min)
            tip = K.fingertip(q)[0]
            err = target - tip
            if np.linalg.norm(err) <= tol:
                break
            if stop_above is not None and tip[2] > target[2] + stop_above:
                break   # 2026-09-27: resting on something (a finger on the block top): don't push the command deeper
            cmd = cmd + err
        return tip, cmd

    def probe(self, x, y, z_start=60.0, z_end=12.0, step=5.0, reach=None, tip_min=None):
        """Closed-gripper touch probe. Returns ('contact'|'free'|'unreachable', measured z).

        Contact = the measured fingertip stops descending (< 2 mm for a `step` command) AND the commanded fingertip is
        below the measured one by > 3 mm. In free air gravity always keeps the arm BELOW its command (sag; ~20 mm on the
        far-reach branch), and the first steps down are absorbed by that sag, so a stall alone is not contact
        (false contacts at z 54 in the 10:30 reach touch test). z / z_end are target heights; the command carries the
        hover's sag offset. tip_min: commanded-path fingertip floor (default: z_end + offset - 1)."""
        if reach is None:
            reach = bool(np.hypot(x, y) > F.REACH_R)
        tip, cmd = self.goto((x, y, z_start), tol=2.5, label="probe_hover", reach=reach)
        off = cmd - np.array([x, y, z_start])
        tmin = min(G.TIP_MIN_GRASP, z_end + off[2] - 1.0) if tip_min is None else tip_min
        prev, z, res = tip[2], z_start, ("free", tip[2])
        while z - step >= z_end - 1e-6:
            z -= step
            try:
                qz = pose_xyz(np.array([x, y, z]) + off, reach)
            except ValueError:          # below the branch's reach at this radius: stop, report the last height
                res = ("unreachable", prev)
                break
            q = self.move(qz, 5, "probe_down", tip_min=tmin)
            t = K.fingertip(q)[0][2]
            zc = K.fingertip(qz)[0][2]
            self._w("probe", f"z={z:.1f} cmd_tip={zc:.1f} meas_tip={t:.1f}")
            if prev - t < 2.0 and zc < t - 3.0:
                res = ("contact", t)
                break
            prev, res = min(prev, t), ("free", t)
        self.move(pose_xyz(np.array([x, y, z_start]) + off, reach), 6, "probe_up")
        return res

    # --- safe exit / parking ----------------------------------------------------------------------------------
    def safe_exit(self, reason=""):
        """Lift, far home, raised rotation, near home, REST, release. Ignores the heat check (this is the cooling action)."""
        _say(f"SAFE EXIT ({reason})")
        q = self.into_windows("exit_wrist")
        tip = K.fingertip(q)[0]
        if tip[2] < 75:
            try:
                self.move(pose_xyz(np.array([tip[0], tip[1], 85.0])), 8, "exit_lift", check_heat=False)
            except ValueError:            # no IK straight above (edge of reach): lift the shoulder instead (2026-09-26)
                up = self.angles()
                up[1] = min(up[1] + 15, 90)
                self.move(up, 6, "exit_lift_j2", check_heat=False, check_path=False)
                self.into_windows("exit_wrist2")
        if self.angles()[0] < 0:
            self.move(self.H_far, 10, "exit_far_home", check_heat=False)
            self.park_near()
        else:
            self.move(self.H_near, 10, "exit_near_home", check_heat=False)
            self.park_rest_from_home()

    def into_windows(self, label="into_windows"):
        """If the wrist relaxed outside the joint windows (after a release: e.g. J6 40 > 35, J4 66), move ONLY J4-J6
        back inside (1 deg margin), unchecked, slowly. Arm/shoulder joints are left alone."""
        q = self.angles()
        tgt = q.copy()
        tgt[3:] = np.clip(q[3:], G.WINDOWS[3:, 0] + 1, G.WINDOWS[3:, 1] - 1)
        if np.abs(tgt - q).max() > 0.5:
            self.move(tgt, 5, label, check_heat=False, check_path=False)
        return self.angles()

    def unpark(self):
        """From REST (released; the wrist may have relaxed): lift the shoulder 20 deg, wrist back into the windows,
        then near home (path-checked)."""
        lift = self.angles()
        lift[1] += 20
        self.move(lift, 8, "unpark_lift", check_path=False)
        self.into_windows("unpark_wrist")
        self.move(self.H_near, 10, "unpark_home")

    def park_rest_from_home(self):
        pre = REST.copy()
        pre[1] += 20
        self.move(pre, 10, "park_pre_rest", check_heat=False)
        self.move(REST, 5, "park_rest", check_heat=False, check_path=False, tol=12)
        self.mc.release_all_servos()
        self._w("release", "park")
        _say("parked at REST and released")

    def far_raised(self):
        """From REST (released) or anywhere near: lift, near home, raise, rotate to J1 -100 (raised). Holds there."""
        q = self.angles()
        if K.fingertip(q)[0][2] < 50:
            self.unpark()
        self.move(S_NEAR, 10, "raise")
        s_far = S_NEAR.copy()
        s_far[0] = -100.0
        self.move(s_far, 12, "rotate_far_raised")

    def park_near(self):
        s_far = S_NEAR.copy()
        s_far[0] = self.angles()[0]
        self.move(s_far, 12, "park_raise", check_heat=False)
        self.move(S_NEAR, 12, "park_rotate", check_heat=False)
        self.move(self.H_near, 12, "park_home", check_heat=False)
        pre = REST.copy()
        pre[1] += 20
        self.move(pre, 10, "park_pre_rest", check_heat=False)
        self.move(REST, 5, "park_rest", check_heat=False, check_path=False, tol=12)
        self.mc.release_all_servos()
        self._w("release", "park")
        _say("parked at REST and released")

    def close(self):
        if self.logf:
            self.logf.close()
        self.mc._serial_port.close()


def j6_for_yaw(xyz, yaw_deg, reach=None, j6_ref=20.0):
    """J6 that squares the fingers to the nearest face of a block whose top-tag edges are at yaw_deg (arm frame, mod 90).
    Reference (empirical): at J6 = j6_ref the fingers grip radially-square blocks well (first-attempt picks of blocks
    <= 22 deg off radial, 2026-09-25). The closing direction is taken as flange x rotated so that it is exactly radial
    at j6_ref; J6 is searched over its window. Returns (j6, residual misalignment deg)."""
    xyz = np.asarray(xyz, float)
    q = pose_xyz(xyz, reach)
    az = np.degrees(np.arctan2(xyz[1], xyz[0]))

    def xang(j6):
        qq = q.copy()
        qq[5] = j6
        a = K.fk_frames(qq)[-1][:3, 0]
        return np.degrees(np.arctan2(a[1], a[0]))
    off = xang(j6_ref) - az
    best = None
    for j6 in np.arange(G.WINDOWS[5, 0] + 1, G.WINDOWS[5, 1] - 1 + 1e-6, 1.0):
        mis = (xang(j6) - off - yaw_deg + 45) % 90 - 45
        if best is None or abs(mis) < abs(best[1]):
            best = (float(j6), float(mis))
    return best


PICKER_J6_OFFSET = 45.0


def j6_picker(xyz, yaw_deg, axis=1):
    """Picker branch (vertical tool): J6 turns the fingers about the vertical. Closing axis = flange y (axis=1; from the
    14:40 touch-test frame, to be confirmed by grasps). Returns (j6, residual misalignment deg) over the J6 window."""
    q = pose_xyz(np.asarray(xyz, float))
    best = None
    for j6 in np.arange(G.WINDOWS[5, 0] + 1, G.WINDOWS[5, 1] - 1 + 1e-6, 1.0):
        qq = q.copy()
        qq[5] = j6
        a = K.fk_frames(qq)[-1][:3, axis]
        # +PICKER_J6_OFFSET: 18:06 supervised grasp came in exactly 45 deg off ("the worst angle", user)
        mis = (np.degrees(np.arctan2(a[1], a[0])) + PICKER_J6_OFFSET - yaw_deg + 45) % 90 - 45
        if best is None or abs(mis) < abs(best[1]):
            best = (float(j6), float(mis))
    return best


def pose_xyz(p, reach=None):
    r, th = float(np.hypot(p[0], p[1])), float(np.degrees(np.arctan2(p[1], p[0])))
    return F.pose(r, th, float(p[2]), reach)
