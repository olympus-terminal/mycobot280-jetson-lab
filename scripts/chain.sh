#!/bin/bash
# Run recorded movements in order; STOP at the first one whose runner refused, stalled or failed.
# Usage: scripts/chain.sh [--delay N] movements/a.json movements/b.json ...
cd "$(dirname "$0")/.."
D=3; [ "$1" = "--delay" ] && { D=$2; shift 2; }
for m in "$@"; do
  out=$(timeout 600 ~/venvs/realsense/bin/python scripts/record_session.py "$m" --delay "$D" --note "chain $(basename $m .json)" 2>&1)
  rc=$?
  echo "$out" | grep -v sign_and_send | grep -E "gripper|STOP|refusing|done:|Traceback|end angles" | tail -4
  if [ $rc -ne 0 ] || echo "$out" | grep -qE "refusing|STOP|Traceback|runner exit [1-9]"; then echo "CHAIN STOPPED at $m"; exit 1; fi
  D=2
done
