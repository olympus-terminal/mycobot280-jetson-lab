# Hardware

Facts marked *measured* were checked on our unit. Vendor facts link to their source.

## Arm: Elephant Robotics myCobot 280 for Arduino

| Item | Value | Source |
|---|---|---|
| DOF / payload / reach / repeatability | 6 / 250 g / 280 mm / ±0.5 mm | [Elephant Robotics specs](https://www.elephantrobotics.com/en/mycobot280-for-arduino-specifications-en/) |
| Power | DC 12 V, 60 W | rating label |
| Serial | CH340 USB bridge (`1a86:7523`) → `/dev/ttyUSB0`; **answers only at 1,000,000 baud** (docs say 115200) | measured |
| Firmware | `get_system_version` 6.5; controller IK (`solve_inv_kinematics`) not supported (−1) | measured |
| Firmware joint limits (deg) | min [−168, −135, −150, −145, −165, −180], max [168, 135, 150, 145, 165, 180] | `get_joint_min/max_angle` |
| Writes | No ACK for writes, so pymycobot returns −1 and resends 3× (~1.5 s); use `_async=True` for single writes | measured |
| URDF | Official `mycobot_280_arduino.urdf` ([mycobot_ros](https://github.com/elephantrobotics/mycobot_ros), BSD-3-Clause) matches the controller's `get_coords` to 1.0 mm RMS; controller frame = URDF root rotated 270° about z | measured (`fk_check.py`) |
| Servos | Feetech STS register map; **max temperature register = 70 °C** on all six; max voltage 12.5 V (J1–J3), 9.0 V (J4–J6) | measured, read-only |
| Out-of-range recovery | The controller blocks all motion while any joint is beyond its limit; move it by hand with 12 V unplugged | measured |
| Collision detection | None observed (a move into a tripod reported no error) | measured |
| Release | `release_all_servos()` from a raised pose drops the arm fast; park low first | measured |
| Heat | J5 (wrist pitch) heats ~10 °C per pick/place move in the picker posture; others stay ≤ 41 °C | measured |
| Lifespan | 500 h working life (vendor, undefined further); ~4.1 h of recorded motion by 2026-09-27 | vendor / session index |

## End effector

- A parallel "picker" gripper (value scale 0 closed … 100 open; empty ≈ 4, holding a 30 mm block ≈ 20–46).
- **The fingers close at 45° to the flange x/y axes. The jaw centre is (+7, +18) mm from the URDF's 100 mm "fingertip" point**
  (guided-grasp measurement).
- The hand meets the table 5–21 mm *below* the model fingertip point, depending on posture. Grasp heights come from touch
  tests, not the model.
- The wrist camera and a bracket protrude from the hand. On the tilted IK branch they limit J5 to ≥ ~48°.

## Cameras

| | Tabletop | Wrist |
|---|---|---|
| Model | Intel RealSense D435i (RGB + stereo depth + IMU), firmware 5.11.1.100 | Generic UVC camera (`0c45:6340`) |
| Host | Laptop, USB 3 via USB-C (stable). On the Jetson it enumerated at USB 3 and dropped | Jetson |
| Modes used | Colour 1280×720 (recording) / 1920×1080 (tag detection), depth 848×480 aligned to colour, ~30 fps | 640×480 YUYV, ~7–8 fps |
| Calibration | Camera↔arm by held-tag reprojection (0.27 px, leave-one-out 1.0 mm) | Hand-eye + intrinsics from blocks at known positions; f ≈ 1000 px (narrow lens) |
| Notes | A 20 mm tag at ~1 m is ~27 px; PnP range ±2 cm, so positions use the ray-plane intersection | Looks sideways from the back of the hand; sees the block during grasps but not the finger centre |

## Blocks and tags

- 30 mm wooden cubes with picture faces and one AprilTag 36h11 face (20 mm). Some IDs repeat, and some copies are a few mm shorter.
- Replacement tags from a Brother PT-P710BT label printer on 24 mm tape: 128 px = the full 180 dpi head, black square
  18.06 mm nominal, **18.0 mm measured**. Generated pixel-exact by `scripts/make_ptouch_tags_*.py`, verified to decode.
- Tag IDs in use: 1–4 (original blocks), 10–17 (relabelled blocks), 30 (base plate, camera drift check), 31–34 (gripper),
  100–139 (reserved for chess).

## Compute

- **Jetson Orin Nano Super Developer Kit**, JetPack 7.2.1, runs the arm's serial link and the wrist camera. See
  [JETSON_SETUP.md](JETSON_SETUP.md).
- **Laptop** (Ubuntu): RealSense, planning, recording; gives the Jetson its network over Ethernet.
- Recording: capture to internal NVMe, then move to a large disk (~1 GB per pick/place move).
