#!/usr/bin/env python3
"""P-touch 24 mm printable-band calibration strip (40 mm long).
Lines run along the tape at every 1 mm across its width; numbers = mm from the label's
top edge (same orientation as the A2S2 label). Rotated onto a 24 x 40 mm portrait page,
exactly like a2s2_tape24_ptouch_portrait."""
import datetime, os, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
g = []
for mm in range(0, 25):
    x1 = 7 if mm % 2 == 0 else 11
    g.append(f'<line x1="{x1}" y1="{mm}" x2="39" y2="{mm}" stroke="#000" stroke-width="0.25"/>')
    if mm % 2 == 0:
        g.append(f'<text x="6" y="{mm + 0.6}" text-anchor="end" font-family="DejaVu Sans" '
                 f'font-weight="bold" font-size="1.7">{mm}</text>')
body = f'<g transform="translate(24 0) rotate(90)">{"".join(g)}</g>'
svg = f'<svg xmlns="http://www.w3.org/2000/svg" width="24mm" height="40mm" viewBox="0 0 24 40">{body}</svg>'
os.makedirs(os.path.join(HERE, "output"), exist_ok=True)
svg_p = os.path.join(HERE, "output", f"ptouch_calib_{TS}.svg")
open(svg_p, "w").write(svg)
pdf_p = svg_p[:-4] + ".pdf"
subprocess.run(["inkscape", svg_p, "--export-text-to-path", "--export-type=pdf", f"--export-filename={pdf_p}"],
               check=True, stderr=subprocess.DEVNULL)
print(pdf_p)
