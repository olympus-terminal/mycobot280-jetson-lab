# A2S2 GROUP equipment labels

Black-on-white labels for lab equipment, printed on the Brother **PT-P710BT** (USB, CUPS queue `PT-P710BT`) with
**24 mm TZe tape**. Each label is 72 mm long: A²S² | GROUP, LAB EQUIPMENT, an EQ № line to write on, and
PROPERTY OF A2S2 GROUP · DO NOT REMOVE.

First batch: 20 labels, 2026-09-28 (CUPS job `PT-P710BT-7`).

## Print more of the same

The PDF is final. Its text is already converted to outlines, so no fonts or scripts are needed.

```bash
lp -d PT-P710BT -n 20 -o PageSize=Custom.68x204 -o print-scaling=none \
  labels/a2s2_tape24_v2_ptouch_20260928_183941.pdf
```

- `-n 20` is the number of labels. Each one is cut separately.
- Keep `-o PageSize=Custom.68x204`. That is 24 × 72 mm in points (68 × 204). With the driver default (`tz-24`,
  68 × 284 pt) you get about 28 mm of blank tape on every label.
- The printer must be switched on and plugged in. A job sent while it is off waits in the queue and prints when it
  comes back (CUPS disables the queue with "Unplugged or turned off", then re-enables it by itself).
- Check progress with `lpstat -W not-completed -o PT-P710BT`. Twenty labels took about 2 minutes.

**Tape use:** about 97 mm per label (72 mm label plus roughly 25 mm cut margin), so 20 labels use about 2 m. An 8 m cassette
makes about 80. The printer does not report how much tape is left. Look through the cassette window.

## Files

| File | What it is |
|---|---|
| `a2s2_tape24_v2_ptouch_20260928_183941.pdf` | **The file to print.** 24 × 72 mm portrait page with the artwork rotated 90°, because the driver's page width is the tape width |
| `a2s2_tape24_v2_ptouch_20260928_183941.svg` | Source SVG for that PDF (text not outlined) |
| `a2s2_tape24_v2_preview_20260928_183941.pdf` | Flat 72 × 24 mm preview. Grey marks the tape strip the printer cannot reach |
| `make_a2s2_tape_v2_20260928_183921.py` | Generator for the two PDFs above |
| `ptouch_calib_20260928_155200.py` / `ptouch_calib_20260928_161826.pdf` | 1 mm ruler strip for measuring the printable band (see below) |

## Why the artwork sits low on the page

The driver (`printer-driver-ptouch`, "Foomatic/ptouch-pt") renders a 24 mm (68 pt) page at 180 dpi, which is 170 px
across. The print head covers only 128 px (about 18 mm), and the lost 6 mm comes off the **top** of the label: the first print
(job `PT-P710BT-4`, artwork centred on the page) had its top clipped. The generator therefore keeps everything
inside the band from `BAND_TOP` = 6 mm to 24 mm of the page, which lands centred on the tape. The v2 label printed at this setting (job `PT-P710BT-6`)
was approved by the operator. The exact band has not been measured. If a new driver version or printer
shifts it, print the calibration strip:

```bash
lp -d PT-P710BT -o PageSize=Custom.68x113 -o print-scaling=none labels/ptouch_calib_20260928_161826.pdf
```

Hold it the same way as the label, with the numbers on the left. The numbers give mm from the top of the page. The
first visible line is the new `BAND_TOP`.

## Change the design or regenerate

Requirements: Python 3, Inkscape 1.x (tested with 1.2.2), and the Syncopate font (Apache 2.0) in `labels/fonts/`:

```bash
mkdir -p labels/fonts && \
for f in Syncopate-Regular Syncopate-Bold; do \
  curl -sSL -o labels/fonts/$f.ttf https://github.com/google/fonts/raw/main/apache/syncopate/$f.ttf; \
done
```

Then regenerate with the default `BAND_TOP` of 6 mm:

```bash
python3 labels/make_a2s2_tape_v2_20260928_183921.py
```

Or give a measured band top in mm:

```bash
python3 labels/make_a2s2_tape_v2_20260928_183921.py 5.5
```

Output goes to `labels/output/` (gitignored) with a new timestamp, so nothing is overwritten. The layout is in
`label()`: all coordinates are in mm, with x along the tape and y across it, and `t` is the top of the printable band.
`LENGTH` sets the label length. If you change it, change the `PageSize=Custom.68x<LENGTH × 2.835>` in the `lp` command too.

Tested on 2026-09-28: running the generator from the repo reproduced the committed print PDF exactly (the 300 dpi rasters
are byte-identical).

The design is draft 02 ("Squared") from the logo drafts, with its corner marks removed and the text from draft 05 added.
The 62 × 29 mm die-cut version and the other drafts were never printed on labels and are not kept here.
