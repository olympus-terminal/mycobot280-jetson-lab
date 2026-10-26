# Setup protocol: myCobot 280 (Arduino), from the box to the first recorded pick

This is the order of steps that worked for us, written for someone setting up a second copy of this lab. Each step ends
with a **done when** check. Don't start a step until the one before it passes. Our bring-up took two days (2026-09-23/24) and
included all the debugging recorded in [LAB_NOTEBOOK.md](LAB_NOTEBOOK.md). Most of that debugging is now handled by the scripts
and checks below.

Tested configuration: **myCobot 280 for Arduino** (base-board firmware 6.5), a **Jetson Orin Nano Super** (JetPack 7.2.1) as
the arm host, and an **Ubuntu 24.04 laptop** with the RealSense. Other myCobot 280 variants (Pi, M5Stack, Jetson Nano) have
different controllers and were not tested. A plain Ubuntu 24.04 PC as the arm host should work, because its kernel
already has the CH340 driver, but we haven't tried it.

| Step | What | Where | Arm |
|---|---|---|---|
| 0 | Parts, table, safety rules | | unpowered |
| 1 | Network between laptop and arm host | both | unpowered |
| 2 | Laptop software (`setup_laptop_*.sh`) | laptop | unpowered |
| 3 | Arm-host software (`setup_arm_host_*.sh`) | arm host | unpowered |
| 4 | First contact, read-only (`probe_arm.py --check`) | arm host | powered, no motion |
| 5 | First motion, one joint at a time | arm host | **moves** |
| 6 | Make the configuration yours (REST pose, joint windows, movements) | both | **moves** |
| 7 | Cameras | both | parked |
| 8 | First recorded session | laptop | **moves** |
| 9 | Camera↔arm calibration and cat guard | laptop | **moves** |
| 10 | Blocks, dry run, first pick | laptop | **moves** |

---

## 0. Parts, table and safety rules

| Part | What we used | Notes |
|---|---|---|
| Arm | Elephant Robotics myCobot 280 for Arduino | Rating label: DC 12 V, 60 W |
| Power | 12 V DC adapter, **≥ 5 A** | The adapter plug is the hardware stop. Keep it within reach |
| Gripper | Parallel "picker" gripper, pointing straight down | The finger geometry was measured on ours (step 6) |
| Arm host | Jetson Orin Nano Super Developer Kit, JetPack 7.2.1 | Runs the serial link and the wrist camera. See [JETSON_SETUP.md](JETSON_SETUP.md) |
| Laptop | Ubuntu 24.04, Python 3.12, a **USB-C** port | Runs the RealSense, planning and recording, and drives the arm host over ssh |
| Tabletop camera | Intel RealSense D435i, **USB-C to USB-C** cable, tripod | C-to-A cables gave USB 2 or nothing. On the Jetson it dropped after ~2 s |
| Wrist camera (optional) | Generic UVC camera, 640×480 | On the arm host |
| Network | One Ethernet cable, laptop ↔ arm host | The plug slid out of the Jetson's socket more than once |
| Blocks | 30 mm cubes with 20 mm AprilTag 36h11 tags | Replacement tags: `scripts/make_ptouch_tags_*.py` (Brother P-touch, 24 mm tape) |
| Mounting | Base plate screwed or clamped to the table | A 6-DOF arm tips itself over when extended |

**Table:** clear a circle of at least **350 mm** radius around the base (280 mm reach, plus the gripper and camera). Keep
tripods out of it: the controller does **not** detect collisions, and our first move ran into the camera tripod without an
error.

**Safety rules (they came from real incidents; see [METHODOLOGY.md](METHODOLOGY.md#safety)):**

1. No motion without a person at the arm, a hand near the 12 V plug, and the lowest speed first.
2. **Never release the servos or unplug the 12 V while the arm is raised.** It falls (J2 dropped 49° in ~6 s). Park it low (REST) first.
3. The software stops are `Ctrl-C` in the running script (sends `stop()`) and `release_all_servos()` from a low pose.
4. Chains of commands stop at the first refusal (`scripts/chain.sh`, or `set -e`). Never run a motion command after a check without testing the check's exit code.
5. Servos: don't start a move above 52 °C (J5 above 50 °C). The servos cut out at 70 °C. Cool while parked, never while holding a raised pose.
6. Unattended runs only with the room closed (people, pets), after many supervised runs.

**Privacy rules for recordings:**

- Cameras record **only while the arm moves**: `record_session.py` and `record_cycle.py` start the cameras right before a
  movement and stop them right after. Don't leave `capture_realsense.py` or `capture_wrist.py` running on their own as a recorder.
- The cat guard saves no images by default (an intrusion is usually a person or a pet). `--snapshot` is for tuning only.
- Before keeping or sharing any session, look through it and **delete everything that shows a person**. Never upload
  frames, video or depth that may show a person to any cloud service, tracker or repository.

**Done when:** the arm is screwed down, the 350 mm circle is clear, the adapter is ≥ 12 V / 5 A, and everyone who will
run the arm has read the rules above.

---

## 1. Network between the laptop and the arm host

The scripts reach the arm host over ssh. Any network works. A direct cable with NetworkManager's *shared* mode gives the
arm host an address and internet access through the laptop:

```bash
# On the laptop. Find the Ethernet interface name first: nmcli device
nmcli connection add type ethernet ifname <ethernet-interface> con-name arm-link ipv4.method shared
nmcli connection up arm-link
# The laptop becomes 10.42.0.1; the arm host gets an address in 10.42.0.0/24:
ip neigh show dev <ethernet-interface>
```

For the Jetson itself (flashing JetPack, boot-disk choice, recovering a
stuck boot), see [JETSON_SETUP.md](JETSON_SETUP.md). In short: avoid unknown NVMe sticks (ours stalled the firmware), and keep a
USB boot disk in the same port it was installed from.

**Done when:** `ssh <user>@<arm-host-address> true` works from the laptop (password login is fine at this point).

---

## 2. Laptop software

```bash
git clone https://github.com/olympus-terminal/mycobot280-jetson-lab.git
cd mycobot280-jetson-lab
bash scripts/setup_laptop_20261026_204203.sh <user>@<arm-host-address>
echo 'export JETSON_HOST=<user>@<arm-host-address>' >> ~/.bashrc     # the laptop scripts read JETSON_HOST
```

The script installs the apt packages and Intel's RealSense udev rules (sudo), creates `~/venvs/realsense` from
`requirements-laptop.txt` (numpy 2.5.3, OpenCV-contrib 5.0.0 with AprilTag support, pyrealsense2 2.58.4), runs a read-only
RealSense check if the camera is plugged in, sets up ssh key login to the arm host and copies the repository to
`~/orin-mycobot` there. It logs to `data/setup/`.

**Done when:** the script ends with "key login to … works" and "repo copied". With the camera plugged in,
`~/venvs/realsense/bin/python scripts/probe_realsense.py` reports depth and colour at ~30 fps.

---

## 3. Arm-host software

```bash
ssh -t $JETSON_HOST 'bash ~/orin-mycobot/scripts/setup_arm_host_20261026_204203.sh'
```

The script installs the apt packages, adds the user to `dialout` (serial port) and `video` (cameras), and makes sure the
**CH340 serial driver** exists. NVIDIA's Jetson kernel is built without it, so on a Jetson it builds the driver
(`scripts/build_ch341.sh`). On a stock Ubuntu kernel it is already there. It then creates `~/venvs/mycobot` from
`requirements-jetson.txt` (pymycobot 4.0.7, pyserial 3.5, numpy). The wrist-camera scripts use the system OpenCV
(`python3-opencv`).

**Rerun it after any kernel or JetPack update.** If the arm has no serial device, the CH340 driver is the first thing to check.

**Done when:** the summary shows pymycobot 4.0.7 and the system OpenCV. Log out and back in if it added you to a group.

---

## 4. First contact (arm powered, no motion)

1. Wrist camera **off** the arm for now. Arm lying low or folded, nothing in the 350 mm circle.
2. Plug the arm's USB cable into the arm host, then the 12 V.
3. Run the read-only probe (it sends only `get_*` queries; opening the port resets the Arduino board, as with any Arduino):

```bash
ssh $JETSON_HOST '~/venvs/mycobot/bin/python ~/orin-mycobot/scripts/probe_arm.py --check'
```

It prints firmware versions, servo voltages and temperatures, angles and firmware joint limits, then a go/no-go list:

| Check | If it fails |
|---|---|
| Port exists, readable | USB cable; `lsmod \| grep ch341`; rerun step 3; log out/in for `dialout` |
| Answers at **1,000,000 baud** | Ours answered only at 1 Mbaud (the documentation says 115200). If yours answers only at 115200, the motion scripts (which open the port at 1 Mbaud) won't talk to it. Stop and check the base-board firmware with Elephant Robotics' myStudio before changing any script |
| Servo power on | 12 V adapter |
| Temperatures ≤ 52 °C | Let it cool parked. Single reads can glitch high (the motion code takes the minimum of 3) |
| **All joints inside the firmware limits** | The controller then refuses **all** motion, even back toward the range, and clearing the error doesn't help. Unplug the 12 V, move the listed joints by hand toward the middle of their range, plug in, rerun. (Ours came out of the box with J2 at −138.3° and J4 at −152.0°, beyond −135 and −145.) |

The last line compares the pose with `REST` in `config/poses.json`. That is **our** REST pose. You record yours in step 6.

**Done when:** `CHECK: GO`. Record the firmware version and the serial path in your notes.

---

## 5. First motion (person at the arm)

Test the stops before any larger move: know where the 12 V plug is, and that `Ctrl-C` stops a running script. Then
move one joint at a time, a few degrees, at speed 10–15:

```bash
# on the arm host (ssh $JETSON_HOST), from ~/orin-mycobot
~/venvs/mycobot/bin/python scripts/jog_joint.py --joint 1 --delta 5 --speed 15
~/venvs/mycobot/bin/python scripts/jog_joint.py --joint 2 --delta 5 --speed 10 --delay 30   # 30 s to walk to the arm
```

`jog_joint.py` refuses steps over 10°, speeds over 30 and targets outside the firmware limits. With `--delay`, the ATOM
LED on the end shows the state: yellow = waiting, red = moving, green = reached, blue = refused. Do J1 to J6 in both
directions. Expect joints to stop 1–3° short when the arm is extended (gravity sag); that is normal. A joint that stops
well short has hit something.

**Done when:** every joint has moved a few degrees in both directions, in the expected direction, with nothing hit.

---

## 6. Make the configuration yours

The constants in this repository were **measured on our arm, gripper, wrist camera and table**. Replace them before
any longer motion.

**a. Move our calibrations out of the way** (in your copy). `find_blocks.py`, `cat_guard.py` and `demo_loop.py` load the
*newest* `config/cam_arm_calib_*.json`, so ours would otherwise be used with your camera:

```bash
mkdir -p config/original_lab
git mv config/cam_arm_calib_*.json config/wrist_cam_*.json config/original_lab/
```

The other files in `config/` (pyramid sites, chess piece tags, the FK pose pairs) are also ours, but nothing loads them
automatically.

**b. Record your REST pose.** REST is the parked pose: the arm folded down with its weight on the table, where it is
released and where it cools. With the 12 V **unplugged**, fold the arm by hand so that the hand rests on the table beside the
base, clear of the camera's view of the work area. Plug in the 12 V and run `probe_arm.py --check`. Copy the six angles
into `config/poses.json` as `REST` (with a `source` note: date, how it was set), and set `PRE_REST` = REST with
J2 + 20°. `arm_local.py`, `gen_movements.py` and `far_side.py` all read REST from this file.

**c. Map the wrist range with your hand on it.** Our gripper and wrist camera hit the forearm below J5 ≈ 44–48° on the
tilted branch, so `run_movement.py` refuses J5 < 48 (`J5_MIN`) and `gen_movements.py` has joint windows (`WINDOWS`).
With your hand and camera mounted, map the limit with stall detection and back-off:

```bash
~/venvs/mycobot/bin/python scripts/sweep_joint.py --joint 5 --goal 0 --step 10 --speed 10 --delay 15
```

Set `J5_MIN` and `WINDOWS` from the last good angle plus a margin. Do the same for any joint where the hand or cable
could touch the arm.

**d. Regenerate the movement files** from your REST pose and windows. Every waypoint and path is checked against the
firmware limits, the windows and the table:

```bash
python3 scripts/gen_movements.py            # movements/library_v1/
python3 scripts/far_side.py                 # movements/far/
```

**e. Test unpark and park** (person at the arm):

```bash
~/venvs/mycobot/bin/python scripts/run_movement.py movements/library_v1/unpark.json --delay 15
~/venvs/mycobot/bin/python scripts/run_movement.py movements/library_v1/park.json --delay 15
```

`run_movement.py` refuses to start more than 8° from a file's `start_pose`, and it stops if a joint ends short of its
target (contact). After a release, the wrist relaxes by 10–15°, and the fixed unpark is then refused. That is the
intended behaviour; `scripts/unpark_here.py` writes an unpark from the current pose.

**f. Mount the wrist camera** and run every `movements/library_v1/sweep_j*.json` once, watching the cable. It must not
snag, tug or pull the camera.

**Done when:** your REST is in `config/poses.json`, unpark → home → park works, and the joint sweeps run with the hand
and camera mounted.

---

## 7. Cameras

- **RealSense (laptop):** mount it on a stable tripod about 1 m from the base, looking down at the work area, with the
  whole 350 mm circle in view and the tripod outside it. `scripts/probe_realsense.py` (saves nothing) should report
  ~30 fps and mostly valid depth. The IMU needs the udev rules from step 2.
- **Wrist camera (arm host):** `v4l2-ctl --list-devices`, and `v4l2-ctl -d /dev/video0 --list-formats-ext` should show
  640×480. Its frames are first saved during the recorded session in step 8.
- **Clock:** recordings measure the laptop↔arm-host clock offset at the start and end of each session (the clocks drifted
  80–100 ppm). Running chrony with the laptop as the server is better.

**Done when:** both cameras are listed, and the RealSense reports its frame rate from the laptop.

---

## 8. First recorded session

```bash
~/venvs/realsense/bin/python scripts/record_session.py movements/library_v1/sweep_j1.json --delay 15
```

It syncs `scripts/ movements/ models/ config/` to the arm host, measures the clock offset, starts both cameras, runs the
movement through `run_movement.py` (with all its checks), stops the cameras and gathers everything in
`data/<stamp>_sweep_j1/`: `realsense_color.mp4`, `realsense_depth/`, `jetson/joints.csv`, `jetson/wrist.mp4`,
`movement.json` and `metadata.json` (git hash, arguments, versions, clock offsets).

**Done when:** the session folder exists, `joints.csv` has readings during the motion (not just at the ends), and
both videos play. Delete it if a person is in view.

---

## 9. Camera↔arm calibration and the cat guard

The calibration fits the camera's pose from a tagged block the arm holds at 14 poses. The fit uses the pixel
reprojection of the tag centre, because PnP range at ~1 m is only good to about ±2 cm.

```bash
# The arm unparks, opens at home and waits 15 s: put a tagged block (tag up) between the fingers.
~/venvs/realsense/bin/python scripts/record_cycle.py calib tour_local.py --handoff 15 --put 0 180
~/venvs/realsense/bin/python scripts/calibrate_cam_arm.py data/<stamp>_cycle_calib --reproj
```

The new `config/cam_arm_calib_<stamp>.json` is used from then on. Ours reached **0.27 px, leave-one-out 1.0 mm**. Fits that
used too few visible holds were much worse (13.8 mm, leave-one-out 34 mm) and went to `config/rejected/`. Keep rejected fits
there rather than deleting them. If the held tag faces away from your camera at many holds, move the camera rather than
accept a poor fit. The `--put` point (x, y in mm, controller frame) must be clear table within reach.

Then take the cat guard's reference, with the arm parked at REST and nothing in the work area:

```bash
~/venvs/realsense/bin/python cat_guard/cat_guard.py ref
~/venvs/realsense/bin/python cat_guard/cat_guard.py check; echo "exit $?"      # 0 clear; put a hand in: 1
```

Retake the reference after any change to the table, the room or the camera, and recalibrate after any camera move.
Before each session, check that the camera hasn't moved ([METHODOLOGY.md §5](METHODOLOGY.md#5-perception-and-cameraarm-calibration)).

**Done when:** a calibration with a leave-one-out error of a few mm is in `config/`, and the cat guard returns 0 when the
area is empty and 1 with a hand in it.

---

## 10. Blocks, dry run, first pick

1. Put a few tagged blocks tag-up in reach. `~/venvs/realsense/bin/python scripts/find_blocks.py` lists them in the arm frame
   (mm), with reachability.
2. **Grasp height by touch:** the camera is trusted in x and y only. Measure the table height with closed-gripper
   touch probes at a few points (`scripts/probe_local.py --pts X1 Y1 X2 Y2 ...` under `record_cycle.py`). Our hand met the
   table 5–21 mm below the model fingertip, depending on the posture.
3. **Corrections:** our grasp and place offsets, jaw centre and drop zones are in
   [PICK_PLACE_PROTOCOL.md](PICK_PLACE_PROTOCOL.md). Start from them, then replace each one with your own
   measurement. A short guided grasp with the arm limp (`scripts/teach_local.py`) gave us the jaw centre and the picker
   posture faster than parameter search did ([METHODOLOGY.md §3](METHODOLOGY.md#3-the-human-teaches-with-a-limp-arm)).
4. Dry run (camera and planning only, the arm never moves):
   `~/venvs/realsense/bin/python scripts/demo_loop.py --dry-run`
5. First supervised pick and place: `~/venvs/realsense/bin/python scripts/demo_loop.py --max-cycles 1`. A move counts as
   placed only if the camera finds the block tag-up near its target on the next look.

**Done when:** one camera-verified pick-and-place, recorded, with no hand intervention.

---

## Every session, before the first move

1. Room and table clear; the 12 V plug within reach.
2. `probe_arm.py --check` → `CHECK: GO`.
3. Camera not moved (SIFT check against a reference frame) and the cat guard reference still valid.
4. At the end: park at REST before cutting power. Delete or keep sessions only after checking them for people.

## Troubleshooting

| Symptom | Cause we found | Fix |
|---|---|---|
| No `/dev/ttyUSB0` on the Jetson | CH340 driver missing from NVIDIA's kernel, or a kernel update removed ours | `sudo bash scripts/build_ch341.sh` |
| `Permission denied` on the serial port | Not in `dialout` | `sudo usermod -aG dialout $USER`, log out and in |
| No answer at 115200 | This arm talks at 1,000,000 baud | Use 1 Mbaud (all scripts do) |
| Every move refused, even toward the valid range | A joint is beyond its firmware limit | 12 V off, move it by hand, power on |
| A write takes ~1.5 s, joint log has no mid-move samples | pymycobot resends unacknowledged writes 3× | Asynchronous writes (`_async=True`), as in `arm_local.py` |
| Joint stops a few degrees short | Gravity sag (extended arm) or contact | `arm_local.goto()` corrects with encoder feedback; a large shortfall is contact: stop and look |
| Unpark refused after a release | The wrist relaxed 10–15° out of its window | `scripts/unpark_here.py`, or `Arm.unpark()` in `arm_local.py` |
| J5 hot (> 55 °C) | Holding a bent wrist; ~10 °C per pick/place move | Park and cool; never wait in a raised pose |
| RealSense drops after ~2 s, or USB 2 only | Cable or port | USB-C to USB-C on the laptop |
| Cat guard false alarms | Old reference, relaxed parked arm, room changes | Retake `cat_guard.py ref` with the arm at REST |
| Arm left hovering after a laptop session died | Remote process died on a write to a closed pipe | Fixed in `arm_local.py` (SIGHUP/SIGTERM → safe exit); use it, not ad-hoc scripts |

## Not tested

- Arm hosts other than our Jetson, and other myCobot 280 variants.
- Calibration with a camera position other than ours. Our tour poses were chosen for our camera, and the far-side poses
  showed the tag to it poorly.
- The setup scripts on a fresh machine from start to finish. They were checked for syntax, and the pinned requirements were
  installed into fresh venvs (2026-10-26); the individual steps are the ones used on 2026-09-24.
