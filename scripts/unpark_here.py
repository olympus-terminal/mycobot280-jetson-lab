#!/usr/bin/env python3
"""Write movements/unpark_here.json: from the arm's CURRENT (resting, released) pose, lift J2 +20 deg, then go to near
home. After a release the wrist relaxes (J4/J6 drift 10-15 deg), so the fixed REST-based unpark is (correctly) refused.
The lift->home segment is path-checked; the first segment starts on the table by design (J2-only lift)."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_movements as G  # noqa: E402
import precise as P  # noqa: E402

cur = P.angles()
lift = cur.copy()
lift[1] += 20
H = G.solve_rz(180, 80)[0]
ok, why = G.path_ok(lift, H, G.TIP_MIN)
if not ok:
    sys.exit(f"lift->home path check failed: {why}")
mv = {"name": "unpark_here", "description": "Lift J2 +20 from the current resting pose, then near home", "created": "2026-09-25",
      "generator": "scripts/unpark_here.py", "start_pose": [round(float(v), 2) for v in cur], "units": "degrees",
      "steps": [G.A(cur, 5, "lying"), G.A(lift, 8, "lift"), G.A(H, label="home")]}
json.dump(mv, open(os.path.join(os.path.dirname(__file__), "..", "movements", "unpark_here.json"), "w"), indent=1)
print("wrote movements/unpark_here.json from", np.round(cur, 2).tolist())
