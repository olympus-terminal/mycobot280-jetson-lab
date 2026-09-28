#!/usr/bin/env python3
"""Index every recorded session in data/ (world-model dataset reference), one row per session.

Reads only what the sessions already contain: metadata.json (script, args, git, clock offset), summary.json (cycle result,
grasp attempts, put-back), realsense_info.json / jetson/wrist_info.json (frames, fps), and for demo-loop cycles the
camera verification from data/demo/demo_<run>.jsonl (the `verify_prev` of a later record whose target equals this
cycle's dst). Nothing is inferred beyond that; missing fields stay empty.

Usage: python3 scripts/index_sessions_20260927_123000.py
Output: data/index/sessions_<stamp>.csv and .json (+ the git commit of this repo at indexing time).
"""
import csv
import datetime
import glob
import json
import os
import re
import subprocess

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DATA = os.path.join(REPO, "data")


def load(path):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return None


def arg(args, flag, n=1):
    if flag not in args:
        return None
    i = args.index(flag)
    v = args[i + 1: i + 1 + n]
    if not v:
        return None   # flag without a value (older sessions)
    return v[0] if n == 1 else v


def verifications():
    """(run_stamp, cycle) -> verify dict, from the demo-loop logs."""
    out = {}
    for f in glob.glob(os.path.join(DATA, "demo", "demo_*.jsonl")):
        run = os.path.basename(f)[5:-6]
        recs = []
        for line in open(f):
            try:
                recs.append(json.loads(line))
            except ValueError:
                pass
        for i, r in enumerate(recs):
            if "dst" not in r or r.get("result") == "dry-run":
                continue
            for r2 in recs[i + 1:]:
                v = r2.get("verify_prev")
                if v and v.get("target") and all(abs(a - b) < 0.2 for a, b in zip(v["target"], r["dst"])):
                    out[(run, r["cycle"])] = v
                    break
    return out


def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ver = verifications()
    rows = []
    for d in sorted(glob.glob(os.path.join(DATA, "2026*_*"))):
        if not os.path.isdir(d):
            continue
        name = os.path.basename(d)
        meta, summ = load(os.path.join(d, "metadata.json")), load(os.path.join(d, "summary.json"))
        rsi, wri = load(os.path.join(d, "realsense_info.json")), load(os.path.join(d, "jetson", "wrist_info.json"))
        args = (meta or {}).get("args") or []
        row = {"session": name, "created": (meta or {}).get("created"), "script": (meta or {}).get("script"),
               "git_commit": ((meta or {}).get("git") or {}).get("commit"), "git_dirty": ((meta or {}).get("git") or {}).get("dirty"),
               "synthetic": (meta or {}).get("synthetic"),
               "realsense_frames": (rsi or {}).get("frames"), "realsense_fps": (rsi or {}).get("fps_measured"),
               "depth_dropped": (rsi or {}).get("depth_frames_dropped"),
               "wrist_frames": (wri or {}).get("frames"), "wrist_fps": (wri or {}).get("fps_measured"),
               "has_joints": os.path.exists(os.path.join(d, "jetson", "joints.csv")) or os.path.exists(os.path.join(d, "joints.csv"))}
        if (meta or {}).get("script") == "cycle_local.py" and len(args) >= 5:
            row.update({"src_x": args[0], "src_y": args[1], "dst_r": args[2], "dst_theta": args[3], "level": args[4],
                        "side": arg(args, "--side"), "picker": "--picker" in args, "grasp_z": arg(args, "--grasp-z"),
                        "yaw": arg(args, "--yaw"), "grasp_bias_xy": arg(args, "--bias", 2),
                        "place_bias_rt": arg(args, "--place-bias-rt", 2), "place_j6": arg(args, "--place-j6"),
                        "drop_z": arg(args, "--drop-z")})
        if summ:
            att = summ.get("attempts") or []
            held = [k for k, a in enumerate(att) if a.get("value") is not None and 12 < a["value"] < 70]
            row.update({"result": summ.get("result"), "seconds": summ.get("seconds"), "n_attempts": len(att),
                        "held_attempt": held[0] if held else None,
                        "grip_value": att[held[0]]["value"] if held else None,
                        "put_back": summ.get("put_back"), "placed_tip": summ.get("placed_tip"),
                        "temps_start": summ.get("temps_start"), "temps_end": summ.get("temps_end")})
        m = re.search(r"_cycle_demo_(\d{8}_\d{6})_(\d{3})$", name)
        if m:
            v = ver.get((m.group(1), int(m.group(2))))
            row.update({"demo_run": m.group(1), "demo_cycle": int(m.group(2)),
                        "verified": None if v is None else v.get("verified"),
                        "verify_error_mm": None if v is None else v.get("error_mm")})
        rows.append(row)
    os.makedirs(os.path.join(DATA, "index"), exist_ok=True)
    out = os.path.join(DATA, "index", f"sessions_{stamp}")
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(out + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: json.dumps(v) if isinstance(v, (list, dict)) else v for k, v in r.items()})
    git = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    json.dump({"created": stamp, "script": os.path.basename(__file__), "git": git, "n_sessions": len(rows), "sessions": rows},
              open(out + ".json", "w"), indent=1)
    n_cyc = sum(1 for r in rows if r.get("script") == "cycle_local.py")
    n_ver = sum(1 for r in rows if r.get("verified") is True)
    print(f"{len(rows)} sessions ({n_cyc} cycles, {n_ver} camera-verified placements) -> {os.path.relpath(out, REPO)}.csv/.json")


if __name__ == "__main__":
    main()
