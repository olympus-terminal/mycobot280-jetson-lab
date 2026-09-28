#!/usr/bin/env python3
"""A2S2 GROUP equipment label v2 for Brother PT-P710BT, 24 mm TZe tape (black only).

v2 changes vs make_a2s2_label_20260928_153918.py (draft 02 "Squared"):
  - corner (crop) marks removed
  - adds draft-05 text: LAB EQUIPMENT, EQ No. field, PROPERTY OF A2S2 GROUP · DO NOT REMOVE
  - GROUP set larger, beside the A²S² wordmark
Printable band: the ptouch-pt driver's 24 mm page is 170 px wide at 180 dpi, the head prints 128 px (18 mm);
job PT-P710BT-4 clipped the top, so the band is assumed to be BAND_TOP..24 mm of the page (not yet
measured; calibration strip ptouch_calib_20260928_161826.pdf printed as job PT-P710BT-5).
Outputs in ./out/, timestamped. Text is converted to paths by Inkscape.
"""
import datetime
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUT = os.path.join(HERE, "output")

BAND_TOP = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0  # mm, first printable mm of the 24 mm page
LENGTH = 72.0  # mm of tape
INK = "#000000"
DISP = "Syncopate"
MONO = "DejaVu Sans Mono"


def label(t):
    """Artwork in mm, landscape (x along tape, y across it). t = top of the 18 mm printable band."""
    m = 4.0  # left/right margin
    r = LENGTH - m
    return f'''
  <text x="{m}" y="{t + 3.0}" font-family="{MONO}" font-weight="bold" font-size="1.8" letter-spacing="0.45" fill="{INK}">LAB EQUIPMENT</text>
  <text x="{r - 17}" y="{t + 3.0}" font-family="{MONO}" font-weight="bold" font-size="1.8" fill="{INK}">EQ №</text>
  <line x1="{r - 11.5}" y1="{t + 3.1}" x2="{r}" y2="{t + 3.1}" stroke="{INK}" stroke-width="0.2"/>
  <text x="{m - 0.4}" y="{t + 11.6}" font-family="{DISP}" font-weight="bold" font-size="9" fill="{INK}">A<tspan font-size="4.1" dx="0.35" dy="-4.7">2</tspan><tspan dx="0.9" dy="4.7">S</tspan><tspan font-size="4.1" dx="0.35" dy="-4.7">2</tspan></text>
  <line x1="{m + 26.6}" y1="{t + 5.2}" x2="{m + 26.6}" y2="{t + 11.6}" stroke="{INK}" stroke-width="0.35"/>
  <text x="{m + 29.2}" y="{t + 11.6}" font-family="{DISP}" font-size="6.2" fill="{INK}" textLength="{r - m - 29.2}" lengthAdjust="spacing">GROUP</text>
  <line x1="{m}" y1="{t + 13.2}" x2="{r}" y2="{t + 13.2}" stroke="{INK}" stroke-width="0.3"/>
  <text x="{m}" y="{t + 16.1}" font-family="{MONO}" font-size="1.55" fill="{INK}" textLength="{r - m}" lengthAdjust="spacing">PROPERTY OF A2S2 GROUP · DO NOT REMOVE</text>
'''


def render(svg, name):
    svg_p = os.path.join(OUT, f"{name}_{TS}.svg")
    pdf_p = svg_p[:-4] + ".pdf"
    with open(svg_p, "w") as f:
        f.write(svg)
    env = dict(os.environ, FONTCONFIG_FILE=os.path.join(HERE, "fonts.conf"))
    subprocess.run(["inkscape", svg_p, "--export-text-to-path", "--export-type=pdf",
                    f"--export-filename={pdf_p}"], check=True, env=env, stderr=subprocess.DEVNULL)
    print(pdf_p)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    # Syncopate must be in ./fonts (see README); a private fontconfig makes Inkscape find it without installing it
    if not os.path.exists(os.path.join(HERE, "fonts", "Syncopate-Bold.ttf")):
        sys.exit("missing fonts/Syncopate-Bold.ttf and Syncopate-Regular.ttf, see labels/README.md")
    with open(os.path.join(HERE, "fonts.conf"), "w") as f:
        f.write('<?xml version="1.0"?><!DOCTYPE fontconfig SYSTEM "fonts.dtd"><fontconfig>'
                f'<include ignore_missing="yes">/etc/fonts/fonts.conf</include><dir>{HERE}/fonts</dir>'
                '</fontconfig>')
    print(f"BAND_TOP={BAND_TOP} mm, LENGTH={LENGTH} mm", file=sys.stderr)
    art = label(BAND_TOP)
    # screen preview: full 24 mm tape, grey = area outside the assumed printable band
    render(f'<svg xmlns="http://www.w3.org/2000/svg" width="{LENGTH}mm" height="24mm" viewBox="0 0 {LENGTH} 24">'
           f'<rect width="{LENGTH}" height="24" fill="#FFF"/>'
           f'<rect width="{LENGTH}" height="{BAND_TOP}" fill="#DDD"/>'
           f'<rect y="{BAND_TOP + 18}" width="{LENGTH}" height="{max(0, 6 - BAND_TOP)}" fill="#DDD"/>{art}</svg>',
           "a2s2_tape24_v2_preview")
    # print file: page width = tape width, artwork rotated onto a 24 x LENGTH mm portrait page
    render(f'<svg xmlns="http://www.w3.org/2000/svg" width="24mm" height="{LENGTH}mm" viewBox="0 0 24 {LENGTH}">'
           f'<g transform="translate(24 0) rotate(90)">{art}</g></svg>',
           "a2s2_tape24_v2_ptouch")
