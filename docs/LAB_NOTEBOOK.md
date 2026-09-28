# Lab notebook (edited)

This is a distilled version of the lab's running notebook, kept as the work happened, 2026-09-23 → 2026-09-27.
Entries are grouped by day. Each gives the problem, what was measured, and what changed. Personal and site details
are removed; the numbers are as recorded. Times are local.

"The operator" is the person in the room. "The agent" is Claude Code, which wrote and ran the scripts (see
[CLAUDE_CODE_WORKFLOW.md](CLAUDE_CODE_WORKFLOW.md)).

---

## Day 0–1 (09-23/24): bring-up, first motion, first pick-and-place

**Jetson bring-up.** JetPack 7.2.1 on a USB hard drive (an 8 cm NVMe stick stalled the UEFI firmware in both M.2 slots).
Keys-only ssh, headless with autologin. The boot entry is tied to the drive's physical USB port.

**No serial device for the arm.** NVIDIA's kernel is built without `CONFIG_USB_SERIAL_CH341`. `scripts/build_ch341.sh`
builds the upstream `ch341.c` (v6.8.12) against the installed headers and autoloads it. It survived a power cut.

**Probe.** No answer at the documented 115200 baud, but the arm answers at **1,000,000 baud**. Firmware 6.5. The rest pose read
J2 −138.3 and J4 −152.0, *outside* the firmware limits (−135, −145).

**First move refused.** While a joint is out of range the controller blocks all motion, even motion back toward the
valid range. Clearing the error did not help. Releasing J2 in software moved nothing. **Recovery: move the joints by hand
with the 12 V unplugged**, then power on. All joints were then inside their limits.

**First commanded motion** (J1 +5° at speed 15): it stopped 1.14° short because it **hit the camera tripod**. The controller
reported no error. Rule: the controller does not detect collisions.

**Undershoot pattern.** J2 fell 1.3° short (the gripper was resting on the table), then 2.6° short in the air with the arm
extended, and under 1° with the arm upright → **gravity sag**, not servo error.

**Wrist self-collision.** Driving J5 toward 0 made the gripper's lower protrusion hit the forearm. J5 crept slowly and heated
to 54 °C. The wrist carries two protrusions (camera above, bracket below). Straight-down grasps with this hand were ruled out
at the time (J5 lower limit ≈ 40–44).

**Kinematics.** The controller's IK returns −1 for every query. The official URDF fitted the controller's `get_coords` to
**1.0 mm RMS** over 7 poses, with orientation exact. The agent wrote its own IK with path checks.

**First pick** at a tilted "working" posture (tool 60° from vertical), with the block placed by hand under the fingertips:
held on the first try (gripper value 28 vs 4 closing on air). **First place** 2 min later.

**Heat.** J5 reached 55–59 °C while holding the bent wrist and did not cool while holding. Temperature guards were added. Some
J3 readings of 56/65/73 °C between normal ones were read glitches, so guards now use the minimum of 3 reads.

**Release drops the arm.** `release_all_servos()` from a raised pose: J2 fell 49° and J4 75° in about 6 s. Rule: park low (REST) first, then
release. A REST pose was recorded, and `park`/`unpark` movements were written.

**Recording pipeline.** `record_session.py`: RealSense on the laptop (28 fps), wrist camera + joints on the Jetson,
clock offset by ssh ping-pong (0.8 ms at first; it drifts ~80–100 ppm). Pymycobot's blocking writes resend
unacknowledged commands 3× (~1.5 s), so the joint log missed most of each move. The fix was asynchronous writes, after which 63 samples
were logged mid-motion in one sweep.

**RealSense cabling.** The camera enumerated at USB 3 on the Jetson for ~2 s and dropped, twice. Other cables gave nothing
or USB 2 only. It is stable at USB 3 on the laptop's USB-C port, so it stays on the laptop.

## Day 2 (09-25): calibration, autonomy, the picker grasp

**Calibration v1.** A held-tag tour gave a fit of 1.9 mm RMS and leave-one-out 3.2 mm. PnP range at ~1 m scatters ±2 cm, so
block-top **ray-plane** intersection replaced PnP translation, and repeatability became 0.1 mm.

**Table height conflict.** The camera said z −20, and the pick said the fingers reach below the model tip. A touch test
settled it: the hand meets the table at model z +5 mm. Only touch-test heights are trusted; the camera is trusted in x,y only near the calibrated
region.

**First autonomous far-side pick missed.** The near-side calibration extrapolated badly to the far side (1–3 cm).
Encoder-feedback positioning (`precise.py`) reached 0.9 mm by encoders, but that doesn't help when the target itself is wrong.
Touch probing found block tops. The wrist heats ~4 °C/min in far poses, which cut searches short.

**Speed fix.** Per-step ssh cost ~3 s, so the control loop moved onto the Jetson (`arm_local.py`, one serial connection,
reads in 0.02 s). A `safe_exit` was added for any error or temperature above 60 °C.

**First fully autonomous pick** (08:59), then place-and-look to estimate a far-side bias. Two pyramid base blocks were placed
**3.6 mm and 3.0 mm** from their slots. Six minutes later all the blocks had been moved, and a cat was then seen standing
on the table in the work area.

**Cat guard** (`cat_guard/`): a depth-based check of the work volume. False positives came from the room behind the table, so
changed pixels are now kept only if their 3-D point is inside the work cylinder.

**Wrist camera calibration.** Assuming a webcam-like focal length of 520 px gave a degenerate fit. The lens is narrow
(f ≈ 1000 px). Fitted on blocks the RealSense saw: RMS 3.5 px, with an independent block checked within ~4 mm.

**Camera remounted higher.** The old calibration became invalid. The new method is **pixel-reprojection calibration**: **0.27 px, leave-one-out 1.0 mm**.
The model fingertip then projected exactly onto the real fingertips.

**Grasp misses were not calibration.** An open finger landed on the block's top edge and pushed the block 20–40 mm. Fixes:
grasp lower (z 13), a jaw offset, and J6 set to square the fingers.

**Operator teaching (limp arm), 14:34.** *"The hand is trying to grip nearly parallel to the table, it should be angled down
much more … like a picker."* The demonstrated grasps had the tool 3–14° from vertical and J5 ≈ 0–5, so the earlier J5 limit applied only to
the other IK branch. The **picker branch** (tool vertical) was built from the demonstration. It reaches r 150–270 mm.

**Guided grasp, 20:12.** The fingers close at **45° to the flange axes**, and the jaw centre is **(+7, +18) mm** from the model
tip. The next two picks were first-attempt and placed. The operator called one "flawless".

**Demo loop v1.** Two errors: it reported "placed" with the gripper still open (the success test is now `12 < value < 70`), and it
re-grasped from its own remembered aim instead of camera detections.

## Day 3 (09-26): rehearsal, a regression, dice rolls

**Camera rotated 0.5°** after a power-down (SIFT, pure-rotation fit) → a corrected calibration, with no new tour needed.

**Workspace map** (`map_local.py`, picker, hover z 80): encoder-kinematics error ≤ 3.3 mm at r 150–200, 5–9 mm at r 250, and in practice
no reach beyond r ≈ 260. J5 heats ~1 °C per point even with the wrist nearly straight.

**P-touch tags.** Pixel-exact 36h11 tags from a label printer (128 px = the whole 180 dpi head, measured 18.0 mm). Tag size is
now per ID so that old 20 mm and new 18 mm tags coexist. Gripper tags were decodable only when the hand leaned toward the camera
(4/21 holds). They confirmed the 45° closing axis to within ±10°.

**Recorder drops.** Writing depth to the USB HDD dropped 1069 frames with gaps up to 5 s. Capturing to NVMe and copying afterwards
gave 29.7 fps with no drops.

**Unattended operation authorised** by the operator for the demo loop. Fixes that day: the stop file must also end the cat-guard
wait, the cat-guard wait no longer times out, and reach is checked on the *finger-adjusted* aim.

**Regression: 8/8 grasps held before 15:12, 0/8 after.** J6, closing axis and code were all ruled out. The cause was not found
that day (see Day 4).

**Dice roll** (the operator's idea): when no block shows a tag on top, lift a side-tagged block and drop it so it tumbles.
Its first dry run found a deeper bug, the **PnP mirror ambiguity**: tag-up blocks read as side faces. With `solvePnPGeneric`
preferring the near-vertical solution, every top tag read n_z ≈ 1.0.

**Blocks disappear.** Every miss or edge grip tends to leave a block picture-up. With tags on only one face, the loop
runs out of usable blocks. The real fix is tags on all six faces.

**Grasp height.** The copies of tags 1/2 sit on slightly shorter blocks. z 13 closed above them, and **z 5** held on the first attempt.

## Day 4 (09-27): measured corrections, verification, towers

**Left-front error measured.** Place-and-look showed the hand landing ~10 mm tangentially off on the left front only. Corrected
re-grasps found −8 mm radial, then (per the operator: "the forward edge") −6/−16 mm. **The next run: 5/5 first-attempt grasps, 5/5
camera-verified placements (3.9–32.6 mm).**

**"Placed" ≠ success.** The operator pointed out that a clip counted as success showed a failed move. Camera
verification of every placement on the next pass became standard (tag-up block within 45 mm of the target). Placements the
camera can't see (forearm shadow, base-column shadow) were removed from the drop zones.

**Put-back on error** was used for real three times. After an error while holding a block, the block is set down where it was picked up.

**Unattended watchdog run** (room closed): 11 moves, **9 camera-verified tag-up (7.1–21.7 mm)**, 2 in a camera shadow.

**Thermal limits.** The servos' own limit is 70 °C on all six (Feetech STS register 13, read only). J5 heats ~10 °C per move. The
operator allowed higher limits: start at J5 ≤ 50 °C, cool to ≤ 46 °C, abort at 62 °C.

**Towers.** The first stack fell apart at release, so release became slower and partial, with a slow straight lift. **The first standing
two-block tower** came at 17:30, then a second. 2/4 stood, both with square grips (value 29). A cat bumped the camera tripod, and SIFT +
rotation-only fit gave a 0.111° correction in minutes.

**Photo pose.** Lifting the arm upright let the camera see 6 tags instead of 3. But holding it heated J5 to **70 °C** once
(see incidents in [CLAUDE_CODE_WORKFLOW.md](CLAUDE_CODE_WORKFLOW.md)). Now every photo look is followed by park and cool.

**3-high attempts: 0/2.** One block was released onto a tower that had already been taken down (the plan was 10 minutes old),
so level ≥ 2 now requires a fresh check. The other came from a skewed grip (value 43), so stacking now refuses grips above 33.

**Skewed grips unexplained.** J6 tracking was measured within a few degrees (commanded −30/−15/0/+15/+30 → read −29.2/−15.5/−0.6/+14.3/+29.4),
and the operator confirmed the tags are not crooked. Of 44 held grasps that day, 37 were square and 6 skewed, with no clear J6 pattern. This is open.

**In-hand measurement failed by design.** With the block held, its top tag is under the palm. An in-hand estimate needs a
side view.

**Chess groundwork** (for a larger arm): see [CHESS.md](CHESS.md).

---

## 09-28: equipment labels

**A2S2 GROUP labels on the PT-P710BT (24 mm tape).** The first print (job `PT-P710BT-4`, artwork centred on the 24 mm page)
had its top clipped. The `ptouch-pt` driver renders 170 px across at 180 dpi, but the head prints only 128 px (about 18 mm).
The v2 layout keeps everything inside 6–24 mm of the page (job `PT-P710BT-6`, approved by the operator). **20 labels
printed** (job `PT-P710BT-7`, about 2 min, about 2 m of tape). Files, the reprint command and the calibration strip are in
[../labels/README.md](../labels/README.md).
