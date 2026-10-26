#!/bin/bash
# One-time setup of the LAPTOP: RealSense, perception, planning, recording; it drives the arm host over ssh.
# Tested on Ubuntu 24.04 with the system Python 3.12. docs/SETUP_PROTOCOL.md step 2.
#
# Usage (on the laptop, from the repo):
#     bash scripts/setup_laptop_20261026_204203.sh                       # laptop only
#     bash scripts/setup_laptop_20261026_204203.sh nvidia@<arm-host>      # also: ssh key login + copy the repo to the arm host
# Asks for the sudo password (and once for the arm host's password, for ssh-copy-id). Idempotent.
#
# Does: apt packages (python3-venv, rsync, ssh, curl, ffmpeg, espeak-ng), Intel's RealSense udev rules (camera without
#       root), venv ~/venvs/realsense from requirements-laptop.txt, a read-only RealSense stream check if a camera is
#       plugged in; with a host: an ssh key (if none), ssh-copy-id, and rsync of the repo (no data/, no .git) to
#       <host>:~/orin-mycobot. Log: data/setup/setup_laptop_<stamp>.log
set -euo pipefail

HOST=${1:-}
REPO=$(cd "$(dirname "$0")/.." && pwd)
STAMP=$(date +%Y%m%d_%H%M%S)
PY=${PYTHON:-/usr/bin/python3}
RS_RULES_URL=https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.58.4/config/99-realsense-libusb.rules
mkdir -p "$REPO/data/setup"
LOG="$REPO/data/setup/setup_laptop_$STAMP.log"
exec > >(tee -a "$LOG") 2>&1

echo "== setup_laptop_20261026_204203.sh  $(date -Is)  host $(hostname)  user $USER"
echo "repo:   $REPO  (git $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo n/a))"
. /etc/os-release && echo "os:     $PRETTY_NAME"
echo "python: $PY ($($PY --version 2>&1)); requirements-laptop.txt was tested with 3.12"

echo "== 1/4 apt packages"
sudo apt-get update -qq
sudo apt-get install -y -qq python3-venv rsync openssh-client curl ffmpeg espeak-ng

echo "== 2/4 RealSense udev rules"
if [ -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
  echo "99-realsense-libusb.rules already present"
else
  sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules "$RS_RULES_URL"
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "installed from $RS_RULES_URL (sha256 $(sha256sum /etc/udev/rules.d/99-realsense-libusb.rules | cut -d' ' -f1))"
  echo "replug the camera if it is already plugged in"
fi

echo "== 3/4 venv ~/venvs/realsense"
mkdir -p "$HOME/venvs"
[ -x "$HOME/venvs/realsense/bin/python" ] || "$PY" -m venv "$HOME/venvs/realsense"
"$HOME/venvs/realsense/bin/pip" install -q --upgrade pip
"$HOME/venvs/realsense/bin/pip" install -q -r "$REPO/requirements-laptop.txt"
"$HOME/venvs/realsense/bin/python" -c "import sys, cv2, pyrealsense2; from importlib.metadata import version as v; print('venv python', sys.version.split()[0], '| numpy', v('numpy'), '| opencv', cv2.__version__, '| pyrealsense2', v('pyrealsense2'), '| AprilTag (cv2.aruco):', hasattr(cv2, 'aruco'))"
if lsusb -d 8086: 2>/dev/null | grep -qi realsense; then
  echo "RealSense on USB; stream check (saves nothing):"
  "$HOME/venvs/realsense/bin/python" "$REPO/scripts/probe_realsense.py" --seconds 3 || echo "WARNING: probe_realsense.py failed (USB 3 cable/port? see SETUP_PROTOCOL.md)"
else
  echo "no RealSense on USB now; later: ~/venvs/realsense/bin/python scripts/probe_realsense.py"
fi

echo "== 4/4 arm host"
if [ -z "$HOST" ]; then
  echo "skipped (no host given). Rerun with: bash scripts/setup_laptop_20261026_204203.sh nvidia@<arm-host>"
else
  [ -f "$HOME/.ssh/id_ed25519.pub" ] || ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" true 2>/dev/null; then
    echo "key login to $HOST works"
  else
    ssh-copy-id -i "$HOME/.ssh/id_ed25519.pub" "$HOST"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" true && echo "key login to $HOST works"
  fi
  rsync -a --exclude data/ --exclude .git/ --exclude labels/ --exclude docs/media/ --exclude docs/photos/ \
    --exclude __pycache__/ "$REPO/" "$HOST:orin-mycobot/"
  echo "repo copied to $HOST:~/orin-mycobot"
  echo
  echo "NEXT (on the arm host; asks for its sudo password):"
  echo "  ssh -t $HOST 'bash ~/orin-mycobot/scripts/setup_arm_host_20261026_204203.sh'"
  echo "and add to ~/.bashrc here:  export JETSON_HOST=$HOST"
fi
echo "log: $LOG"
