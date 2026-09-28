# Jetson Orin Nano: setup notes

These notes cover how the Jetson that drives the arm was set up, and the problems we hit.

## Configuration

| Item | Value |
|---|---|
| Board | Jetson Orin Nano Super Developer Kit |
| OS | Ubuntu 24.04, kernel 6.8.12-tegra, Jetson Linux R39.2.1 |
| JetPack | 7.2.1 (CUDA 13.2, cuDNN 9.20, TensorRT 10.16) |
| Power mode | MAXN_SUPER |
| Root disk | USB hard drive (see the NVMe note below) |
| Python | 3.12 (system); arm venv `~/venvs/mycobot` with pymycobot 4.0.7, pyserial 3.5; system OpenCV 4.8.0 for the wrist camera |
| Access | Headless; ssh with keys only; the laptop shares its network over Ethernet |
| User groups | `dialout` and `video` (serial port and cameras without sudo) |

## CH340 serial driver (required for the arm)

NVIDIA's kernel is built without `CONFIG_USB_SERIAL_CH341`, so the arm's USB bridge gives no `/dev/ttyUSB0`.
`scripts/build_ch341.sh` downloads the upstream `ch341.c` for the running kernel version (v6.8.12 here; it prints the file's
sha256, which was `b286d49d…235951` for ours), builds it
against `nvidia-l4t-kernel-headers`, installs it to `updates/` and autoloads it at boot.

- The module is unsigned and taints the kernel. Signatures are not enforced, and NVIDIA's own out-of-tree modules taint it too.
- **Rerun after any kernel/JetPack update.** If the arm has no `/dev/ttyUSB0`, check this first.
- Undo: `sudo rm /lib/modules/$(uname -r)/updates/ch341.ko /etc/modules-load.d/ch341.conf && sudo depmod -a`.

## Problems we hit

- **NVMe:** the one 8 cm single-notch NVMe stick we tried stalled the UEFI firmware in both M.2 slots, before any boot. The
  system runs from a USB hard drive instead.
- **Boot entry tied to a USB port:** the UEFI boot entry for a USB disk is bound to its physical port. In another port, the
  firmware falls back to network boot and then the UEFI shell.
- **Stuck boot:** an interrupted boot can mark *OS chain A* as failed. To fix it: Esc at the logo → Device Manager → NVIDIA Configuration →
  L4T Configuration → OS chain A status = Normal.
- **USB-C is device mode only** on this kit (for connecting a computer to the Jetson). Peripherals such as the RealSense can't use it.
- **RealSense on the Jetson** enumerated at USB 3 and dropped after ~2 s. It runs on the laptop instead.
- **Clock:** the Jetson and laptop clocks drift apart at ~80–100 ppm. Each recording measures the offset at start and end over
  ssh. Right after a power cut the Jetson's clock can be far off until NTP syncs.
- **Displays:** some monitors drop the signal when inputs switch, and the boot firmware never redraws. Stay on the Jetson's input during
  boot, or run headless.
- **Shut down cleanly** (`sudo shutdown -h now`). After a power cut, ext4 journal recovery ran without data loss.
- **The Ethernet plug can slide out of the socket.** If the Jetson disappears, check the cable first.
