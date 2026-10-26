#!/bin/bash
# One-time setup of the ARM HOST: the computer the myCobot 280's USB cable plugs into (here a Jetson Orin Nano Super,
# JetPack 7.2.1; a stock Ubuntu 24.04 PC should also work, but only the Jetson was tested). docs/SETUP_PROTOCOL.md step 3.
#
# Usage (on the arm host, from the repo copy at ~/orin-mycobot, which the laptop scripts expect):
#     bash scripts/setup_arm_host_20261026_204203.sh
# Asks for the sudo password. Idempotent: rerun it after a kernel/JetPack update (it rebuilds the CH340 driver if missing).
# Moves nothing and does not open the arm's serial port; the first contact is scripts/probe_arm.py (protocol step 4).
#
# Does: apt packages (python3-venv, python3-opencv, v4l-utils, rsync, curl, build tools), groups dialout + video,
#       CH340 serial driver (Jetson kernels lack it: scripts/build_ch341.sh), venv ~/venvs/mycobot from
#       requirements-jetson.txt. Log: data/setup/setup_arm_host_<stamp>.log
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$REPO/data/setup"
LOG="$REPO/data/setup/setup_arm_host_$STAMP.log"
exec > >(tee -a "$LOG") 2>&1

echo "== setup_arm_host_20261026_204203.sh  $(date -Is)  host $(hostname)  user $USER"
echo "repo:   $REPO"
echo "kernel: $(uname -r)  arch: $(uname -m)"
. /etc/os-release && echo "os:     $PRETTY_NAME"
JETSON=0
if [ -f /etc/nv_tegra_release ]; then JETSON=1; echo "jetson: $(head -1 /etc/nv_tegra_release)"; fi
[ "$REPO" = "$HOME/orin-mycobot" ] || echo "NOTE: the laptop scripts sync to and run from ~/orin-mycobot; this copy is at $REPO"

echo "== 1/4 apt packages"
PKGS="python3-venv python3-opencv v4l-utils rsync curl build-essential"
if [ "$JETSON" = 1 ] && [ ! -f "/lib/modules/$(uname -r)/build/Module.symvers" ]; then
  PKGS="$PKGS nvidia-l4t-kernel-headers"     # needed to build the CH340 driver
fi
sudo apt-get update -qq
# shellcheck disable=SC2086
sudo apt-get install -y -qq $PKGS

echo "== 2/4 groups (serial port and cameras without sudo)"
NEED_RELOGIN=0
for g in dialout video; do
  if id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
    echo "already in $g"
  else
    sudo usermod -aG "$g" "$USER"; echo "added to $g"; NEED_RELOGIN=1
  fi
done

echo "== 3/4 CH340 serial driver (ch341)"
if modinfo ch341 >/dev/null 2>&1; then
  echo "ch341 available: $(modinfo -n ch341)"
  lsmod | grep -q '^ch341' || sudo modprobe ch341
elif [ "$JETSON" = 1 ]; then
  sudo bash "$REPO/scripts/build_ch341.sh"
else
  echo "ERROR: no ch341 module for kernel $(uname -r), and this is not a Jetson; install your distribution's extra kernel modules" >&2
  exit 1
fi

echo "== 4/4 venv ~/venvs/mycobot"
mkdir -p "$HOME/venvs"
[ -x "$HOME/venvs/mycobot/bin/python" ] || /usr/bin/python3 -m venv "$HOME/venvs/mycobot"
"$HOME/venvs/mycobot/bin/pip" install -q --upgrade pip
"$HOME/venvs/mycobot/bin/pip" install -q -r "$REPO/requirements-jetson.txt"

echo "== summary"
"$HOME/venvs/mycobot/bin/python" -c "import sys, pymycobot, serial, numpy; print('venv python', sys.version.split()[0], '| pymycobot', pymycobot.__version__, '| pyserial', serial.__version__, '| numpy', numpy.__version__)"
/usr/bin/python3 -c "import cv2; print('system OpenCV (wrist camera)', cv2.__version__)"
echo "serial devices:"; ls -l /dev/serial/by-id/ 2>/dev/null || echo "  none (arm USB not plugged in, or driver not loaded)"
echo "video devices:";  v4l2-ctl --list-devices 2>/dev/null || echo "  none"
echo
if [ "$NEED_RELOGIN" = 1 ]; then
  echo "NEXT: log out and back in (or reboot) so the new groups apply, then:"
else
  echo "NEXT:"
fi
echo "  with the arm's 12 V on and nobody touching it (read-only, no motion):"
echo "  ~/venvs/mycobot/bin/python $REPO/scripts/probe_arm.py --check"
echo "log: $LOG"
