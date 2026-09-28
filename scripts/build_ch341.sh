#!/bin/bash
# Build and install the ch341 USB-serial driver (CH340/CH341 chips, used by the myCobot 280 Arduino)
# for the running Jetson kernel. NVIDIA's tegra kernel ships with CONFIG_USB_SERIAL_CH341 unset.
#
# Usage (on the Jetson):  sudo bash scripts/build_ch341.sh
# Rerun after any kernel update (nvidia-l4t-kernel / JetPack upgrade).
#
# Source: upstream stable kernel ch341.c at the tag matching the running kernel's base version.
# Installs: /lib/modules/$(uname -r)/updates/ch341.ko and /etc/modules-load.d/ch341.conf
# Undo:     sudo rm /lib/modules/$(uname -r)/updates/ch341.ko /etc/modules-load.d/ch341.conf && sudo depmod -a
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

KREL=$(uname -r)                              # e.g. 6.8.12-1021-tegra
KBASE=${KREL%%-*}                             # e.g. 6.8.12
KDIR=/lib/modules/$KREL/build
URL="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v$KBASE"
WORK=$(mktemp -d "${TMPDIR:-/tmp}/ch341_build_XXXXXX")
trap 'rm -rf "$WORK"' EXIT

echo "kernel:  $KREL (base v$KBASE)"
echo "headers: $(readlink -f "$KDIR")"
echo "gcc:     $(gcc --version | head -1)"
echo "source:  $URL"

[ -f "$KDIR/Module.symvers" ] || { echo "kernel headers missing: $KDIR" >&2; exit 1; }
modinfo usbserial >/dev/null || { echo "usbserial module missing" >&2; exit 1; }

curl -fsSL "$URL" -o "$WORK/ch341.c"
grep -q 'module_usb_serial_driver' "$WORK/ch341.c" || { echo "downloaded file doesn't look like ch341.c" >&2; exit 1; }
echo "sha256:  $(sha256sum "$WORK/ch341.c" | cut -d' ' -f1)"

echo 'obj-m := ch341.o' > "$WORK/Makefile"
make -C "$KDIR" M="$WORK" modules

install -D -m 644 "$WORK/ch341.ko" "/lib/modules/$KREL/updates/ch341.ko"
echo ch341 > /etc/modules-load.d/ch341.conf
depmod -a "$KREL"
modprobe ch341

echo "installed: $(modinfo -n ch341)"
echo "vermagic:  $(modinfo -F vermagic ch341)"
lsmod | grep -E '^ch341'
