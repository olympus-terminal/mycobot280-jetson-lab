#!/usr/bin/env python3
"""Record a pyramid cycle as a session: RealSense capture (laptop) while scripts/cycle_local.py runs on the Jetson.

Output: data/YYYYMMDD_HHMMSS_cycle_<tag>/ with realsense_*, jetson/joints.csv, cycle.log, summary.json, metadata.json
Usage: ~/venvs/realsense/bin/python scripts/record_cycle.py TAG EST_X EST_Y SLOT_R SLOT_TH LEVEL
"""
import datetime
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import record_session as RS  # noqa: E402


def main():
    tag, args = sys.argv[1], sys.argv[2:]
    script = "cycle_local.py"
    if args and args[0].endswith(".py"):
        script, args = args[0], args[1:]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(RS.REPO, "data", f"{stamp}_cycle_{tag}")
    os.makedirs(os.path.join(out, "jetson"))
    r = RS.sh(["rsync", "-a", "--delete", "scripts", "movements", "models", "config", f"{RS.JETSON}:{RS.REMOTE_REPO}/"], cwd=RS.REPO)
    if r.returncode:
        sys.exit(f"rsync failed: {r.stderr}")
    meta = {"session": os.path.basename(out), "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "kind": "local arm script on the Jetson", "script": script, "args": args, "synthetic": False,
            "git": {"commit": RS.sh(["git", "-C", RS.REPO, "rev-parse", "HEAD"]).stdout.strip(),
                    "dirty": bool(RS.sh(["git", "-C", RS.REPO, "status", "--porcelain"]).stdout.strip())},
            "clock_offset_start": RS.clock_offset(), "flags": {"cat_or_person_in_workspace": None}}
    # capture to the internal NVMe, move to data/ after stop: the data drive is a USB spinning HDD and depth PNGs + MP4 on
    # it stalled the capture (2026-09-26 13:33: 20.9 fps, 1069 depth frames dropped; diagnosis by the cat-guard-d9 session)
    rec_tmp = os.path.join(os.path.expanduser("~"), "rec_tmp", os.path.basename(out))
    os.makedirs(rec_tmp, exist_ok=True)
    rs_log = open(os.path.join(out, "capture_realsense.log"), "w")
    rs = subprocess.Popen([sys.executable, os.path.join(HERE, "capture_realsense.py"), "--out", rec_tmp], stdout=rs_log, stderr=subprocess.STDOUT)
    remote_log = f"/tmp/cycle_{stamp}_joints.csv"
    # 2026-09-26: wrist camera too (as record_session does); a failed start only warns
    wrist_dir = f"/tmp/cycle_{stamp}_wrist"
    w = RS.remote(f"mkdir -p {wrist_dir} && cd {RS.REMOTE_REPO} && {{ nohup {RS.PY_SYS} scripts/capture_wrist.py --out {wrist_dir} "
                  f"> {wrist_dir}/capture_wrist.log 2>&1 < /dev/null & echo $!; }}", timeout=20)
    wrist_pid = w.stdout.strip() if w.stdout.strip().isdigit() else None
    if wrist_pid is None:
        print(f"WARNING: wrist capture did not start: {w.stdout!r} {w.stderr!r}")
    meta["wrist_capture"] = bool(wrist_pid)
    try:
        time.sleep(2.0 if wrist_pid else 1.0)
        RS.countdown()      # spoken "T minus 5 ... 2 ... 1" right before the arm moves
        res = RS.remote(f"cd {RS.REMOTE_REPO} && {RS.PY_SYS if script.startswith('wrist_') else RS.PY_ARM} scripts/{script} {' '.join(args)} --log {remote_log}", timeout=1500)
    finally:
        if wrist_pid:
            RS.remote(f"kill -TERM {wrist_pid}; sleep 1.5", timeout=20)
        rs.send_signal(signal.SIGTERM)
        try:
            rs.wait(timeout=30)
        except subprocess.TimeoutExpired:   # RealSense pipeline.stop() can hang: never let it kill the session record
            rs.kill()
            rs.wait(timeout=10)
            print("WARNING: RealSense capture had to be killed; its video may be incomplete")
        rs_log.close()
        mv = RS.sh(["rsync", "-a", "--remove-source-files", rec_tmp + "/", out + "/"])
        if mv.returncode == 0:
            shutil.rmtree(rec_tmp, ignore_errors=True)
        else:
            print(f"WARNING: could not move the capture from {rec_tmp} to {out}: {mv.stderr.strip()}")
    open(os.path.join(out, "cycle.log"), "w").write(res.stdout + res.stderr)
    RS.sh(["rsync", "-a", f"{RS.JETSON}:{remote_log}", os.path.join(out, "jetson", "joints.csv")])
    if wrist_pid:
        RS.sh(["rsync", "-a", f"{RS.JETSON}:{wrist_dir}/", os.path.join(out, "jetson") + "/"])
        RS.remote(f"rm -rf {wrist_dir}", timeout=20)
    summ = [l[8:] for l in res.stdout.splitlines() if l.startswith("SUMMARY ")]
    summary = json.loads(summ[-1]) if summ else {"result": "no summary", "exit": res.returncode}
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=1)
    st = RS.remote(f"{RS.PY_ARM} -c \"from pymycobot import MyCobot280; mc=MyCobot280('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0','1000000'); print(mc.get_angles())\"", timeout=30)
    meta["arm_end_angles"] = st.stdout.strip()
    print(f"ARM END ANGLES {st.stdout.strip()}  (REST is ~[64.5, -104.8, -31.6, 39.9, 97.6, 35.2])")
    meta["clock_offset_end"] = RS.clock_offset()
    meta["cycle_exit"] = res.returncode
    meta["timestamps"] = "realsense t_host = laptop time.time(); jetson/joints.csv t_host = JETSON time; laptop = jetson - offset"
    json.dump(meta, open(os.path.join(out, "metadata.json"), "w"), indent=1)
    print("\n".join(l for l in res.stdout.splitlines() if not l.startswith("SUMMARY ")))
    print(f"RESULT {summary.get('result')}  exit {res.returncode}  seconds {summary.get('seconds')}  temps_end {summary.get('temps_end')}")
    print(f"session {out}")
    return res.returncode


if __name__ == "__main__":
    sys.exit(main())
