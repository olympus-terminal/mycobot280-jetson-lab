# Pick-and-place rehearsal protocol (myCobot 280 picker + RealSense)

State as of 2026-09-27 12:30. The dated history is in [LAB_NOTEBOOK.md](LAB_NOTEBOOK.md). This file is the distilled
procedure. Update it when a number below changes.

## Hardware and frames

- Arm: myCobot 280 (Arduino), 1 Mbaud on the Jetson (`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`). Picker
  gripper: the tool points straight down. The fingers close at 45° to the flange x/y (`arm_local.PICKER_J6_OFFSET = 45`).
- Tabletop camera: RealSense D435i on the laptop, fixed tripod. Calibration: `config/cam_arm_calib_20260927_170528_rotfix2.json` (09-27 17:05, +0.111° after a cat bumped the tripod; before: `…20260926_071021_rotfix.json`)
  (p_cam = R·p_base + t, mm, controller frame). Before trusting it, check that the camera hasn't moved: SIFT on the
  base-plate/Jetson region against a recent frame. A median shift under 1 px is fine. A 0.5° rotation needed a new calibration.
- Hand (wrist) camera: UVC on the Jetson, 640×480 at about 7–8 fps, on the back of the hand. It shows the block during
  grasps but not the fingers' centre.
- Blocks: 30 mm (IDs 1–4 appear on two blocks each; some copies are a few mm shorter). Top AprilTags are 36h11, 20 mm.
  The P-touch tags are 18 mm (IDs 10–17 per block, 30 base plate, 31–34 gripper).

## One rehearsal cycle (`scripts/demo_loop.py`, laptop)

1. Cat guard (`cat_guard/cat_guard.py check`, silent): the loop waits until the view is clear, with no timeout.
   The reference must be taken with the arm at REST and nothing in view (`cat_guard.py ref`). Retake it after any
   change to the room or table.
2. Servo temperatures: start only when J5 ≤ 50 °C (all ≤ 52). Otherwise it takes a cooling break until J5 ≤ 46 (2026-09-27; J5 rises ~10 °C per move, max 15; in-move abort 62 °C; servo cutoff 70 °C).
3. `find_blocks.py`: 30 frames at 1920×1080. Each tag's position comes from where its pixel ray meets the plane of the
   block tops (z 14.9). The PnP pose ambiguity is resolved in favour of the vertical solution. A tag counts as "TOP" when
   n_z > 0.98 and it is seen in 15 or more frames.
4. Plan: a random TOP source within reach (θ 0–124 near side, −90…−30 far side, r + 25 ≤ 262). It needs 45 mm clearance
   to other blocks, and the finger-adjusted aim must be inside every joint window at z 5/25/80. The destination is random
   in ZONES, at least 55 mm from every block, reachable with the place J6.
5. `record_cycle.py` → `cycle_local.py` on the Jetson: RealSense RGB-D + joints + hand camera are recorded to
   `data/<stamp>_cycle_<tag>/`. This captures to the internal NVMe, then moves the files to the data HDD.
6. Verify on the next pass: a TOP tag within 45 mm of the slot target. Otherwise the move counts as a miss.
   "placed" in summary.json only means the gripper opened at the slot.

Stop the loop: `touch /tmp/demo_stop`. It finishes the current move, and the arm is parked after every move.

## Current corrections (all empirical; see LAB_NOTEBOOK.md, days 3–4)

| What | Value | Evidence |
|---|---|---|
| Grasp height (model fingertip) | z 5 | z 13 closed above the shorter blocks (0/5); z 5: 1st-attempt holds |
| Jaw centre in the flange frame | (+7, +18) mm | 09-25 guided (limp-arm) grasp |
| Grasp aim, front (θ ≥ 0) | +5 mm radial | operator: fingers on the edge closest to the base |
| Grasp aim, left front (θ ≥ 75, ramp from 40) | extra −6 radial, −16 tangential | place-and-look + corrected re-grasps; 0/8 → 5/5 |
| Grasp aim, far side (θ < 0) | −5 mm radial | +5 missed 5/5 at θ −68 |
| Place bias | 0 radial / −6 tangential; left front +10 radial and −16 tangential | run 13 landings |
| Place J6 | random ±30° (variation) | 2026-09-27 12:30 |
| Grasp search | (0,0), (+8,0), (−8,0), (0,+12), (0,−12) mm (radial, tangential) | `cycle_local` cands |
| Finger resting on the block | goto `stop_above=8` → BLOCKED, try the next offset | 09-27 09:31 |
| Path floors | gs_low gz−8, slot_near zp−5, slot_place zp−16 | sized for grasp z 5 and far-side sag |
| Error while holding | put-back: set the block down at the grasp spot, then safe exit | worked twice (09-27) |

## Drop zones (r mm, θ deg)

(175–200, 78–100), (175–225, 12–35), (140–175, 75–100), (175–225, −85…−60).
Excluded, and why:
- θ 35–75: the parked forearm lies low there.
- Inner right front (r 140–175, θ 12–35): the parked forearm hides it from the camera.
- θ 78–100 beyond r 200: the parked gripper rests there (hides the tag; collision risk when released).
- θ −60…−40: the base column's camera shadow.
- r > 225: placements land up to 30 mm out, which puts blocks beyond pick reach.

## Safety and operation

- Never cut power or release the arm from a raised pose, because it drops. It always parks at REST before release.
- The cat guard gates every move. Unattended runs need the operator's explicit go-ahead (given 2026-09-26/27, room closed).

## Data

- Every move: `data/<stamp>_cycle_demo_<run>_<n>/`: `realsense_color.mp4`, `realsense_depth/*.png` (aligned to colour),
  `realsense_frames.csv`, `jetson/joints.csv` (commands + readings), `jetson/wrist.mp4` + `wrist_frames.csv`,
  `jetson/look_a*_{hover,low}.jpg`, `cycle.log`, `summary.json`, `metadata.json` (git hash, clock offset, args).
  About 1 GB per move. `data/` is not in this repository.
- Per-run loop log: `data/demo/demo_<stamp>.jsonl` (source, destination, biases, verification).
- Promo cuts: `data/promo/promo_draft_<stamp>_{plain,dimbg}.mp4` + a .json manifest listing the source sessions.
  They are made by `scripts/make_promo_cut_20260926_171000.py [--pip] [--dim-bg]`. Check every clip for people.

## Known open problems

- The camera's block positions on the far side are extrapolated (the calibration holds were near side): ±5–10 mm scatter.
- The left-front error varies with position. The proper fix is hand-camera centring at the hover (to be built).
  The "block at the bottom centre" reference depends on J6.
- Blocks drift out of reach and turn picture-up over long runs. Tags on all six faces would remove the picture-up problem.

## Promo video format (operator-approved 2026-09-27)

`scripts/make_promo_cut_20260926_171000.py --speed 8 --crop 540:540:418:0 SESSION ...` → 720×720, 8×, no hand-camera inset
(the inset shows desk-side items), plain (no background dimming). The crop was chosen to contain 100 % of the action
envelope of the 9 clips (arm and blocks, x 459–916, y 11–503 of the 1280×720 colour). Re-check the envelope if the camera or
the zones change. Use only camera-verified moves and check every clip for people.
