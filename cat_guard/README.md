# Cat guard

A depth-based check that the arm's work area is empty before every move. We built it after a cat walked through the work area
and scattered a half-built pyramid. (It also catches hands.)

```bash
~/venvs/realsense/bin/python cat_guard/cat_guard.py ref       # reference: empty work area, arm parked at REST
~/venvs/realsense/bin/python cat_guard/cat_guard.py check     # exit 0 clear, 1 intrusion
~/venvs/realsense/bin/python cat_guard/cat_guard.py wait --speak   # spoken warnings until clear
```

## How it works

1. **Reference:** the median depth of 15 frames of the empty work area, with the arm parked at REST.
2. **Check:** the median of 5 current frames. Candidate pixels are > 45 mm *closer* to the camera than the reference. A 30 mm block
   barely changes depth at this viewing angle; a cat or a hand changes it a lot.
3. **3-D filter:** each candidate pixel is deprojected with the current depth into the arm frame, using the camera↔arm
   calibration, and kept only if it lies inside the work cylinder (r ≤ 300 mm around the base, table level to +350 mm). The 2-D
   outline of that cylinder also covers everything behind it, which caused false alarms from the room behind the table.
4. **Decision:** intrusion if the largest connected blob is ≥ 1500 px. The check saves a snapshot with a box, plus the depth, and logs a
   JSON line.

`scripts/demo_loop.py` and the tower/pyramid scripts call `check` before every move and **wait with no timeout** until it is
clear. An intrusion is logged, never overridden.

## Limits

- It works only while the arm is **parked at REST**, because the moving arm changes depth itself. Retake the reference after any change
  to the table or room, or if the parked pose drifts (a relaxed forearm counted as an intrusion).
- It depends on the calibration: after a camera move, recalibrate, then retake the reference.
- The RealSense can be opened by one process at a time, so the check runs between recordings, not during them.
- The voice uses `espeak-ng`. Its lines are in `LINES` in the script.
