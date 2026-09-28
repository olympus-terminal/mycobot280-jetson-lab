#!/usr/bin/env python3
"""Black-and-white cat head icons for the Brother PT-P710BT (24 mm tape, 128-dot head, 180 dpi).

Cats: Bengal, fat Bengal, Arabian Mau, grey Maine Coon. Hand-drawn vector shapes, no data.
Each icon is drawn on a 1024 px canvas (8x supersampling), box-downsampled to 128 px and
thresholded to pure 1-bit. "Grey" areas (Maine Coon coat) use a 1-on/2-off diagonal hatch
applied at print resolution so it stays crisp on the thermal head.

Output: assets/ptouch_cat_icons_<timestamp>/  (one PNG per cat + strip + 4x preview)
"""
import datetime
import os
import sys

import numpy as np
import PIL
from PIL import Image, ImageDraw
from scipy import ndimage

S = 1024          # drawing canvas
OUT = 128         # printer head height in dots
SCALE = S // OUT
GAP = 40          # white dots between icons in the strip (~5.6 mm, for cutting)


# ---------------------------------------------------------------- mask helpers
def _canvas():
    im = Image.new("L", (S, S), 0)
    return im, ImageDraw.Draw(im)


def _arr(im):
    return np.asarray(im) > 127


def ellipse(cx, cy, rx, ry):
    im, d = _canvas()
    d.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=255)
    return _arr(im)


def poly(pts):
    im, d = _canvas()
    d.polygon([tuple(p) for p in pts], fill=255)
    return _arr(im)


def line(pts, w):
    im, d = _canvas()
    d.line([tuple(p) for p in pts], fill=255, width=w, joint="curve")
    r = w / 2
    for x, y in (pts[0], pts[-1]):          # round caps
        d.ellipse([x - r, y - r, x + r, y + r], fill=255)
    return _arr(im)


def arc(cx, cy, r, a0, a1, w):
    im, d = _canvas()
    d.arc([cx - r, cy - r, cx + r, cy + r], a0, a1, fill=255, width=w)
    return _arr(im)


def lens(cx, cy, half_w, half_h, tilt=0.0):
    """Almond eye: intersection of two circles, optionally tilted (radians, + = outer corner up)."""
    R = (half_w ** 2 + half_h ** 2) / (2 * half_h)
    yy, xx = np.mgrid[0:S, 0:S].astype(np.float32)
    c, s = np.cos(tilt), np.sin(tilt)
    u = (xx - cx) * c + (yy - cy) * s
    v = -(xx - cx) * s + (yy - cy) * c
    return (u ** 2 + (v - (R - half_h)) ** 2 <= R ** 2) & (u ** 2 + (v + (R - half_h)) ** 2 <= R ** 2)


def rounded(m, r):
    """Round the corners of a mask (open then close with a disc)."""
    k = disc(r)
    m = ndimage.binary_opening(m, structure=k)
    return m


def disc(r):
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return xx ** 2 + yy ** 2 <= r ** 2


def outline(m, w):
    """Ring of width w on the inside of mask m."""
    return m & ~(ndimage.distance_transform_edt(m) > w)


def inset(m, w):
    return ndimage.distance_transform_edt(m) > w


def mirror(m):
    return m[:, ::-1]


def both(m):
    return m | mirror(m)


def rosette(cx, cy, r=62, w=22, dot=24, gap_deg=50):
    """Bengal rosette: a broken ring (two arcs) around a solid dot."""
    m = arc(cx, cy, r, 20, 180 - gap_deg / 2 + 20, w) | arc(cx, cy, r, 200, 360 - gap_deg / 2 + 20, w)
    return m | ellipse(cx, cy, dot, dot)


class Icon:
    def __init__(self):
        self.K = np.zeros((S, S), bool)   # black
        self.G = np.zeros((S, S), bool)   # grey (hatched)

    def paint(self, m, c):
        if c == "k":
            self.K |= m; self.G &= ~m
        elif c == "g":
            self.G |= m; self.K &= ~m
        else:                              # white
            self.K &= ~m; self.G &= ~m

    def render(self):
        def down(m):
            a = m.reshape(OUT, SCALE, OUT, SCALE).mean(axis=(1, 3))
            return a >= 0.5
        k, g = down(self.K), down(self.G)
        yy, xx = np.mgrid[0:OUT, 0:OUT]
        hatch = ((xx + yy) % 3) == 0
        black = k | (g & hatch)
        return Image.fromarray(np.where(black, 0, 255).astype(np.uint8)).convert("1")


# ---------------------------------------------------------------- common face parts
def face_parts(ic, cy_eye, eye_dx, eye_hw, eye_hh, nose_y, tilt=0.18, eye_col="k", pupil_col="w"):
    L = lens(512 - eye_dx, cy_eye, eye_hw, eye_hh, tilt=-tilt)
    ic.paint(both(L), eye_col)
    # highlight / pupil
    if pupil_col == "w":
        ic.paint(both(ellipse(512 - eye_dx + eye_hw * 0.28, cy_eye - eye_hh * 0.25, 20, 20)), "w")
    # nose (rounded inverted triangle)
    nose = rounded(poly([(512 - 50, nose_y - 30), (512 + 50, nose_y - 30), (512, nose_y + 32)]), 10)
    ic.paint(nose, "k")
    # mouth: short philtrum + "w"
    m = line([(512, nose_y + 20), (512, nose_y + 62)], 20)
    m |= arc(470, nose_y + 40, 42, 20, 160, 20) | arc(554, nose_y + 40, 42, 20, 160, 20)
    ic.paint(m, "k")


def ear_pair(base_out, tip, base_in):
    return both(poly([base_out, tip, base_in]))


# ---------------------------------------------------------------- the cats
def bengal():
    ic = Icon()
    head = ellipse(512, 610, 330, 290)
    ears = rounded(ear_pair((215, 520), (255, 150), (455, 340)), 18)
    shape = head | ears
    ic.paint(outline(shape, 42), "k")
    ic.paint(both(rounded(poly([(270, 420), (283, 250), (390, 350)]), 8)), "k")    # inner ears
    # forehead stripes (the "M")
    for x, h in ((462, 90), (512, 110), (562, 90)):
        ic.paint(line([(x, 360), (x, 360 + h)], 26), "k")
    # rosettes on the cheeks + crown
    for cx, cy in ((275, 665), (350, 790), (372, 440)):
        ic.paint(both(rosette(cx, cy, r=54, w=24, dot=22)), "k")
    face_parts(ic, cy_eye=585, eye_dx=135, eye_hw=78, eye_hh=46, nose_y=710)
    return ic


def fat_bengal():
    ic = Icon()
    head = ellipse(512, 590, 420, 300) | ellipse(512, 700, 470, 240)   # wide, jowly
    ears = rounded(ear_pair((190, 470), (215, 190), (400, 330)), 18)
    shape = head | ears
    ic.paint(outline(shape, 42), "k")
    ic.paint(both(rounded(poly([(240, 420), (245, 290), (330, 360)]), 8)), "k")
    for x, h in ((467, 80), (512, 100), (557, 80)):
        ic.paint(line([(x, 360), (x, 360 + h)], 26), "k")
    for cx, cy in ((185, 690), (300, 815), (340, 440), (200, 560)):
        ic.paint(both(rosette(cx, cy, r=50, w=22, dot=20)), "k")
    # contented closed eyes (downward arcs) + chubby grin
    ic.paint(both(arc(385, 560, 70, 20, 160, 30)), "k")
    nose_y = 700
    nose = rounded(poly([(512 - 50, nose_y - 30), (512 + 50, nose_y - 30), (512, nose_y + 32)]), 10)
    ic.paint(nose, "k")
    m = line([(512, nose_y + 20), (512, nose_y + 62)], 20)
    m |= arc(465, nose_y + 40, 47, 20, 160, 20) | arc(559, nose_y + 40, 47, 20, 160, 20)
    ic.paint(m, "k")
    ic.paint(rounded(poly([(482, nose_y + 90), (542, nose_y + 90), (542, nose_y + 125), (482, nose_y + 125)]), 1)
             | ellipse(512, nose_y + 125, 30, 28), "k")                                          # tongue blep
    return ic


def arabian_mau():
    ic = Icon()
    head = ellipse(512, 640, 285, 285) | poly([(300, 700), (724, 700), (512, 930)])
    ears = rounded(ear_pair((235, 560), (185, 60), (470, 380)), 22)
    shape = rounded(head | ears, 20)
    ic.paint(shape, "k")                                                        # solid black cat
    ic.paint(both(outline(poly([(265, 470), (225, 160), (410, 380)]), 16) & ~poly([(250, 540), (470, 380), (380, 600)])), "w")
    # big green-gold eyes -> white almonds with black slit pupils
    L = lens(512 - 125, 620, 88, 58, tilt=-0.22)
    ic.paint(both(L), "w")
    ic.paint(both(ellipse(512 - 125, 620, 17, 50)), "k")
    ny = 760
    ic.paint(rounded(poly([(512 - 42, ny - 26), (512 + 42, ny - 26), (512, ny + 26)]), 8), "w")
    m = line([(512, ny + 14), (512, ny + 52)], 16)
    m |= arc(475, ny + 34, 37, 20, 160, 16) | arc(549, ny + 34, 37, 20, 160, 16)
    ic.paint(m, "w")
    # whiskers: white inside the head only
    wh = both(line([(330, 790), (140, 760)], 12) | line([(330, 820), (150, 850)], 12))
    ic.paint(wh & shape, "w")
    ic.paint(wh & ~shape, "k")
    return ic


def maine_coon():
    ic = Icon()
    head = ellipse(512, 590, 320, 270) | rounded(poly([(215, 600), (809, 600), (740, 840), (512, 890), (284, 840)]), 60)
    ears = rounded(ear_pair((215, 500), (235, 110), (470, 380)), 14)
    tufts = both(poly([(218, 160), (228, 36), (262, 150)]))                     # lynx tips
    # ruff: spiky mane around the lower face
    ang = np.linspace(np.deg2rad(-5), np.deg2rad(185), 27)
    pts = []
    for i, a in enumerate(ang):
        r = 445 if i % 2 == 0 else 385
        pts.append((512 + r * np.cos(a), 600 + 0.82 * r * np.sin(a)))
    ruff = poly([(512 + 370, 520)] + pts + [(512 - 370, 520)])
    ruff &= ~ellipse(512, 1600, 10, 10)   # no-op, keeps type
    shape = head | ears | tufts | ruff
    ic.paint(ruff | tufts, "k")
    ic.paint(inset(ruff, 34) & ~head, "w")
    ic.paint(head | ears, "k")
    ic.paint(inset(head | ears, 40), "g")                                       # grey coat
    ic.paint(both(poly([(265, 430), (262, 215), (390, 345)])), "w")            # pale inner ear
    ic.paint(both(line([(258, 200), (262, 300)], 12) | line([(275, 230), (300, 320)], 12)), "k")  # ear furnishings
    # white muzzle
    muzzle = ellipse(455, 735, 85, 70) | ellipse(569, 735, 85, 70) | ellipse(512, 800, 70, 55)
    ic.paint(muzzle, "w")
    ic.paint(outline(muzzle, 14), "k")
    # eyes: white almond, black outline, round pupil
    L = both(lens(512 - 140, 580, 82, 52, tilt=-0.2))
    ic.paint(L, "k")
    ic.paint(inset(L, 12), "w")
    ic.paint(both(ellipse(512 - 140, 580, 30, 38)), "k")
    ic.paint(both(ellipse(512 - 130, 566, 10, 10)), "w")
    ny = 705
    ic.paint(rounded(poly([(512 - 48, ny - 28), (512 + 48, ny - 28), (512, ny + 30)]), 10), "k")
    m = line([(512, ny + 18), (512, ny + 60)], 18)
    m |= arc(472, ny + 38, 40, 20, 160, 18) | arc(552, ny + 38, 40, 20, 160, 18)
    ic.paint(m, "k")
    # tabby forehead lines
    for x, h in ((470, 70), (512, 90), (554, 70)):
        ic.paint(line([(x, 410), (x, 410 + h)], 20), "k")
    return ic


CATS = [("bengal", bengal), ("fat_bengal", fat_bengal), ("arabian_mau", arabian_mau), ("grey_maine_coon", maine_coon)]


def main():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "assets", f"ptouch_cat_icons_{ts}")
    os.makedirs(out)
    icons = []
    for name, fn in CATS:
        im = fn().render()
        p = os.path.join(out, f"cat_{name}_128px.png")
        im.save(p)
        icons.append(im)
        print("wrote", p)
    W = GAP + sum(i.width + GAP for i in icons)
    strip = Image.new("1", (W, OUT), 1)
    x = GAP
    for im in icons:
        strip.paste(im, (x, 0)); x += im.width + GAP
    sp = os.path.join(out, "strip_cats_128px.png")
    strip.save(sp)
    strip.convert("L").resize((W * 4, OUT * 4), Image.NEAREST).save(os.path.join(out, "preview_strip_x4.png"))
    with open(os.path.join(out, "README.txt"), "w") as f:
        f.write(f"Generated {ts} by scripts/{os.path.basename(__file__)}\n"
                f"python {sys.version.split()[0]}, numpy {np.__version__}, Pillow {PIL.__version__}\n"
                f"128 px tall = full PT-P710BT head (180 dpi, ~18 mm printed on 24 mm tape); "
                f"strip {W} px = {W / 180 * 25.4:.1f} mm long.\n")
    print("strip", sp, f"{W} px = {W / 180 * 25.4:.1f} mm")


if __name__ == "__main__":
    main()
