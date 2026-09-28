#!/usr/bin/env python3
"""Record one session: run a movement file on the arm while recording RealSense (laptop), wrist camera
and joint angles (Jetson), then gather everything into one timestamped folder with metadata.

Runs on the LAPTOP. Steps:
  1. syncs scripts/, movements/, models/, config/ to the Jetson (rsync)  -> the Jetson runs this commit's code
  2. measures the laptop<->Jetson clock offset (ping-pong over one ssh connection; best of N)
  3. starts capture_realsense.py (laptop) and capture_wrist.py (Jetson, background)
  4. runs run_movement.py --log on the Jetson (foreground; its safety checks apply)
  5. stops both captures, copies the Jetson files back, re-measures the clock offset, writes metadata.json

Output: data/YYYYMMDD_HHMMSS_<movement>/ with realsense_*, jetson/{joints.csv, wrist.mp4, ...}, runner.log,
movement.json (exact copy), metadata.json.

SUPERVISED USE ONLY when the movement moves the arm (a person at the arm).

Usage:
    ~/venvs/realsense/bin/python scripts/record_session.py movements/idle_5s.json
    ~/venvs/realsense/bin/python scripts/record_session.py movements/pick_test_v1.json --delay 15 --note "block placed by hand"
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
JETSON = os.environ.get("JETSON_HOST", "nvidia@jetson-orin.local")   # ssh target of the Jetson (set JETSON_HOST)
REMOTE_REPO = "~/orin-mycobot"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
PY_ARM = "~/venvs/mycobot/bin/python"
PY_SYS = "/usr/bin/python3"


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def say(text):
    """Speak on the laptop (Bluetooth speaker); never fails the caller."""
    try:
        subprocess.run(["espeak-ng", "-v", "en-us+m3", "-p", "18", "-s", "140", "-a", "190", text], capture_output=True, timeout=20)
    except Exception:
        pass


VOICE_SINK = os.environ.get("VOICE_SINK", "")   # optional PipeWire sink name for the voice clips (e.g. a Bluetooth speaker)
VOICE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "voice")   # optional <key>.wav clips (not included)


def play(key, fallback_text=None):
    """Play a pre-rendered voice clip VOICE_DIR/<key>.wav (optional, not included in this repo);
    espeak-ng fallback if the clip is missing or playback fails."""
    if os.environ.get("ROBOT_VOICE", "1") == "0":
        return
    wav = os.path.join(VOICE_DIR, f"{key}.wav")
    if os.path.exists(wav):
        try:
            if VOICE_SINK and subprocess.run(["pw-play", "--target", VOICE_SINK, wav], capture_output=True, timeout=20).returncode == 0:
                return
            if subprocess.run(["pw-play", wav], capture_output=True, timeout=20).returncode == 0:   # speaker gone: default
                return
        except Exception:
            pass
    if fallback_text:
        say(fallback_text)


def countdown():
    """User request 2026-09-25 17:30: announce every arm motion - "T minus 5 and counting ... 2 ... 1"; motion starts after "1"."""
    if os.environ.get("ROBOT_VOICE", "1") == "0":     # user 2026-09-25 20:40: voice off for now (ROBOT_VOICE=0)
        time.sleep(1.0)
        return
    t0 = time.time()
    play("tminus5", "T minus 5 and counting.")
    while time.time() - t0 < 3.2:
        time.sleep(0.1)
    play("two", "2.")
    while time.time() - t0 < 4.3:
        time.sleep(0.1)
    play("one", "1.")


def remote(cmd, **kw):
    return sh(SSH + [JETSON, cmd], **kw)


def clock_offset(n=15):
    """Jetson time minus laptop time (s), from the exchange with the smallest round trip."""
    p = subprocess.Popen(SSH + [JETSON, PY_SYS + " -u -c \"import sys,time\nfor l in sys.stdin: print(repr(time.time()), flush=True)\""],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    best = None
    try:
        for _ in range(n):
            t0 = time.time()
            p.stdin.write("x\n")
            p.stdin.flush()
            tj = float(p.stdout.readline())
            t1 = time.time()
            rtt = t1 - t0
            if best is None or rtt < best[1]:
                best = (tj - (t0 + t1) / 2, rtt)
    finally:
        p.stdin.close()
        p.wait(timeout=5)
    return {"offset_s": round(best[0], 5), "rtt_s": round(best[1], 5), "samples": n}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("movement")
    ap.add_argument("--delay", type=float, default=0, help="seconds before the arm starts (person walks to the arm)")
    ap.add_argument("--note", default="")
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--data", default=os.path.join(REPO, "data"))
    args = ap.parse_args()

    mv_raw = open(args.movement, "rb").read()
    mv = json.loads(mv_raw)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(args.data, f"{stamp}_{mv['name']}")
    os.makedirs(out)
    remote_dir = f"/tmp/session_{stamp}"
    print(f"session {out}", flush=True)

    git = {"commit": sh(["git", "-C", REPO, "rev-parse", "HEAD"]).stdout.strip(),
           "dirty": bool(sh(["git", "-C", REPO, "status", "--porcelain"]).stdout.strip())}
    r = sh(["rsync", "-a", "--delete", "scripts", "movements", "models", "config", f"{JETSON}:{REMOTE_REPO}/"], cwd=REPO)
    if r.returncode:
        sys.exit(f"rsync failed: {r.stderr}")
    off_start = clock_offset()
    print(f"clock offset (jetson - laptop) {off_start['offset_s'] * 1000:.1f} ms, rtt {off_start['rtt_s'] * 1000:.1f} ms", flush=True)

    shutil.copyfile(args.movement, os.path.join(out, "movement.json"))
    meta = {"session": os.path.basename(out), "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": args.note, "movement_file": os.path.relpath(os.path.abspath(args.movement), REPO),
            "movement_sha256": hashlib.sha256(mv_raw).hexdigest(), "git": git, "clock_offset_start": off_start,
            "delay_s": args.delay, "host_laptop": os.uname().nodename,
            "synthetic": False, "flags": {"cat_or_person_in_workspace": None}}

    rs_cmd = [sys.executable, os.path.join(HERE, "capture_realsense.py"), "--out", out] + (["--no-depth"] if args.no_depth else [])
    rs_log = open(os.path.join(out, "capture_realsense.log"), "w")
    rs_proc = subprocess.Popen(rs_cmd, stdout=rs_log, stderr=subprocess.STDOUT)
    try:
        return _record(args, out, remote_dir, meta, rs_proc, rs_log)
    finally:
        if rs_proc.poll() is None:  # never leave the camera recording (e.g. after an exception)
            rs_proc.send_signal(signal.SIGTERM)
            rs_proc.wait(timeout=30)
        remote(f"pkill -TERM -f 'capture_wrist.py --out {remote_dir}'", timeout=20)


def _record(args, out, remote_dir, meta, rs_proc, rs_log):
    r = remote(f"mkdir -p {remote_dir} && cd {REMOTE_REPO} && {{ nohup {PY_SYS} scripts/capture_wrist.py --out {remote_dir} "
               f"> {remote_dir}/capture_wrist.log 2>&1 < /dev/null & echo $!; }}", timeout=20)
    wrist_pid = r.stdout.strip()
    if not wrist_pid.isdigit():
        rs_proc.send_signal(signal.SIGTERM)
        sys.exit(f"wrist capture did not start: {r.stdout!r} {r.stderr!r}")
    print(f"recording (wrist pid {wrist_pid})", flush=True)
    time.sleep(2.0)  # let both cameras start streaming before the arm moves
    meta["t_runner_start_laptop"] = time.time()
    runner = remote(f"cd {REMOTE_REPO} && {PY_ARM} scripts/run_movement.py {args.movement} --delay {args.delay} "
                    f"--log {remote_dir}/joints.csv")
    meta["t_runner_end_laptop"] = time.time()
    open(os.path.join(out, "runner.log"), "w").write(runner.stdout + runner.stderr)
    print(runner.stdout.strip())
    time.sleep(1.0)
    remote(f"kill -TERM {wrist_pid}; sleep 1.5")
    rs_proc.send_signal(signal.SIGTERM)
    rs_proc.wait(timeout=30)
    rs_log.close()
    os.makedirs(os.path.join(out, "jetson"))
    sh(["rsync", "-a", f"{JETSON}:{remote_dir}/", os.path.join(out, "jetson") + "/"])
    remote(f"rm -rf {remote_dir}")
    meta["clock_offset_end"] = clock_offset()
    meta["runner_exit"] = runner.returncode
    for name in ("realsense_info.json", os.path.join("jetson", "wrist_info.json")):
        p = os.path.join(out, name)
        if os.path.exists(p):
            meta[os.path.basename(name).replace(".json", "")] = json.load(open(p))
    meta["timestamps"] = ("t_host columns: laptop time.time() in realsense_frames.csv; JETSON time.time() in "
                          "jetson/joints.csv and jetson/wrist_frames.csv. laptop_time = jetson_time - clock_offset.offset_s. "
                          "MP4 header fps may differ from the real rate: use the frame CSVs.")
    json.dump(meta, open(os.path.join(out, "metadata.json"), "w"), indent=1)
    print(f"done: {out}  (runner exit {runner.returncode}, offset end {meta['clock_offset_end']['offset_s'] * 1000:.1f} ms)")
    return runner.returncode


if __name__ == "__main__":
    sys.exit(main())
