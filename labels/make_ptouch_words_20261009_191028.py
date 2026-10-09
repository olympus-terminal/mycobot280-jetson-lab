#!/usr/bin/env python3
"""Trimmed word labels for Brother PT-P710BT (24 mm tape), printed directly with ptouch-print (no CUPS driver).

Each label is a 1-bit PNG exactly 128 px tall (= the 128-dot head, 180 dpi, ~18 mm), only as long as its text plus
FLANK_MM each side. Font: Syncopate Bold (fonts/, see README), same cap height for every label.
Usage: python3 make_ptouch_words_<TS>.py "myCobot" "Jetson|Orin Nano"     ('|' = line break)
Writes output/word_<n>_<TS>.png per label and prints the one-job ptouch-print command (cut mark between labels).
"""
import datetime
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
OUT = os.path.join(HERE, "output")
DPI, HEAD = 180, 128
FLANK_MM = 2.0
CAP_PX = 44          # cap height in dots (~6.2 mm, as the 10-09 storage-bag labels)
LINE_GAP_PX = 14     # between lines of a two-line label
FONT = os.path.join(HERE, "fonts", "Syncopate-Bold.ttf")


def mm(px):
    return px * 25.4 / DPI


def label_png(text, font):
    lines = text.split("|")
    boxes = [font.getbbox(l, anchor="ls") for l in lines]          # relative to the baseline
    width = max(b[2] - b[0] for b in boxes)
    flank = round(FLANK_MM * DPI / 25.4)
    img = Image.new("L", (width + 2 * flank, HEAD), 255)
    d = ImageDraw.Draw(img)
    block = len(lines) * CAP_PX + (len(lines) - 1) * LINE_GAP_PX
    top = (HEAD - block) // 2
    for i, (l, b) in enumerate(zip(lines, boxes)):
        base = top + (i + 1) * CAP_PX + i * LINE_GAP_PX
        x = flank + (width - (b[2] - b[0])) // 2 - b[0]
        d.text((x, base), l, font=font, fill=0, anchor="ls")
    return img.point(lambda v: 0 if v < 128 else 255).convert("1")


if __name__ == "__main__":
    texts = sys.argv[1:] or sys.exit(__doc__)
    os.makedirs(OUT, exist_ok=True)
    # font size whose capital H is CAP_PX dots tall
    size = CAP_PX
    for _ in range(5):
        h = ImageFont.truetype(FONT, size).getbbox("H", anchor="ls")
        size = round(size * CAP_PX / (h[3] - h[1]))
    font = ImageFont.truetype(FONT, size)
    paths, total = [], 0
    for n, t in enumerate(texts):
        img = label_png(t, font)
        bbox = Image.eval(img.convert("L"), lambda v: 255 - v).getbbox()
        assert bbox and bbox[1] >= 0 and bbox[3] <= HEAD, (t, bbox)
        p = os.path.join(OUT, f"word_{n}_{TS}.png")
        img.save(p)
        paths.append(p)
        total += img.width
        print(f"{t!r}: {img.width} x {img.height} px = {mm(img.width):.1f} mm long, ink rows {bbox[1]}-{bbox[3]}",
              file=sys.stderr)
    print(f"font size {size}px; strip ink+flanks {mm(total):.1f} mm (printer adds its own feed margins)", file=sys.stderr)
    args = " --cutmark ".join(f"--image {p}" for p in paths)
    print(f"~/.local/bin/ptouch-print {args}")
