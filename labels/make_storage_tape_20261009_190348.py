#!/usr/bin/env python3
"""Storage-bag labels for Brother PT-P710BT, 24 mm TZe tape (black only): one word per label, nothing else.

Same page geometry, font and printable band as make_a2s2_tape_v2_20260928_183921.py (BAND_TOP 6 mm,
18 mm band, 72 mm label, artwork rotated onto a 24 x 72 mm portrait page). All words share one font size
(the largest at which the longest word fits LENGTH - 2*MARGIN), centred in the band.
Usage: python3 make_storage_tape_<TS>.py [WORD ...]   (default TIES CABLES CHARGERS)
Outputs in ./output/, timestamped. Text is converted to paths by Inkscape.
"""
import datetime
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUT = os.path.join(HERE, "output")

BAND_TOP = 6.0  # mm, first printable mm of the 24 mm page (see README)
BAND = 18.0     # mm printable across the tape
LENGTH = 72.0   # mm of tape
MARGIN = 4.0    # mm left/right
INK = "#000000"
DISP = "Syncopate"
WORDS = [w.upper() for w in sys.argv[1:]] or ["TIES", "CABLES", "CHARGERS"]
ENV = dict(os.environ, FONTCONFIG_FILE=os.path.join(HERE, "fonts.conf"))


def query(svg_p, what):
    r = subprocess.run(["inkscape", svg_p, "--export-text-to-path", f"--query-{what}"],
                       capture_output=True, text=True, check=True, env=ENV)
    return float(r.stdout.strip().splitlines()[-1]) * 25.4 / 96  # Inkscape reports px (96/in), not viewBox units


def word_svg(word, size, x=0.0, y=20.0):
    return (f'<text x="{x}" y="{y}" font-family="{DISP}" font-weight="bold" font-size="{size}" '
            f'fill="{INK}">{word}</text>')


def measure(word, size):
    """(x, y, width, height) of the ink of `word` at `size`, in mm, drawn at baseline (0, 20)."""
    p = os.path.join(OUT, f"_measure_{TS}.svg")
    with open(p, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="30mm" viewBox="0 0 100 30">'
                f'{word_svg(word, size)}</svg>')
    vals = [query(p, k) for k in ("x", "y", "width", "height")]
    os.remove(p)
    return vals


def render(svg, name):
    svg_p = os.path.join(OUT, f"{name}_{TS}.svg")
    pdf_p = svg_p[:-4] + ".pdf"
    with open(svg_p, "w") as f:
        f.write(svg)
    subprocess.run(["inkscape", svg_p, "--export-text-to-path", "--export-type=pdf",
                    f"--export-filename={pdf_p}"], check=True, env=ENV, stderr=subprocess.DEVNULL)
    print(pdf_p)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(os.path.join(HERE, "fonts", "Syncopate-Bold.ttf")):
        sys.exit("missing fonts/Syncopate-Bold.ttf and Syncopate-Regular.ttf, see labels/README.md")
    with open(os.path.join(HERE, "fonts.conf"), "w") as f:
        f.write('<?xml version="1.0"?><!DOCTYPE fontconfig SYSTEM "fonts.dtd"><fontconfig>'
                f'<include ignore_missing="yes">/etc/fonts/fonts.conf</include><dir>{HERE}/fonts</dir>'
                '</fontconfig>')
    # one size for all words: longest word fills the width, capped so the caps use at most 75% of the band
    ref = 10.0
    w_max = max(measure(w, ref)[2] for w in WORDS)
    h_ref = max(measure(w, ref)[3] for w in WORDS)
    size = ref * min((LENGTH - 2 * MARGIN) / w_max, 0.75 * BAND / h_ref)
    print(f"BAND_TOP={BAND_TOP} mm, LENGTH={LENGTH} mm, font-size={size:.2f} mm, words={WORDS}", file=sys.stderr)
    for word in WORDS:
        x0, y0, w, h = measure(word, size)
        x = (LENGTH - w) / 2 - x0                    # centre the ink along the tape
        y = BAND_TOP + (BAND - h) / 2 + (20 - y0)    # centre the ink across the printable band
        art = word_svg(word, size, round(x, 3), round(y, 3))
        print(f"  {word}: ink {w:.1f} x {h:.1f} mm", file=sys.stderr)
        render(f'<svg xmlns="http://www.w3.org/2000/svg" width="{LENGTH}mm" height="24mm" viewBox="0 0 {LENGTH} 24">'
               f'<rect width="{LENGTH}" height="24" fill="#FFF"/>'
               f'<rect width="{LENGTH}" height="{BAND_TOP}" fill="#DDD"/>{art}</svg>',
               f"storage_{word.lower()}_tape24_preview")
        render(f'<svg xmlns="http://www.w3.org/2000/svg" width="24mm" height="{LENGTH}mm" viewBox="0 0 24 {LENGTH}">'
               f'<g transform="translate(24 0) rotate(90)">{art}</g></svg>',
               f"storage_{word.lower()}_tape24_ptouch")
