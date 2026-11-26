"""Render the t=3.60 s crop with every Eq. 6 component boxed and labelled."""
import sys
from PIL import Image, ImageDraw
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import foveal_detector as fd
from check_green_box import decode

src, out = sys.argv[1], sys.argv[2]
ev = decode(src)
best, comps = fd.detect(ev)
S = 20                                   # px per sensor pixel in the render
u0, v0 = fd.U0 - fd.R, fd.V0 - fd.R
side = 2 * fd.R + 1
im = Image.new('RGB', (side * S, side * S + 120), (11, 11, 16))
d = ImageDraw.Draw(im)
for u, v, p in ev:
    if abs(u - fd.U0) <= fd.R and abs(v - fd.V0) <= fd.R:
        x, y = (u - u0) * S, (v - v0) * S
        d.rectangle([x, y, x + S - 1, y + S - 1], fill=(230, 57, 53) if p > 0 else (59, 130, 246))
d.ellipse([(fd.U0 - u0) * S + S/2 - 5, (fd.V0 - v0) * S + S/2 - 5, (fd.U0 - u0) * S + S/2 + 5, (fd.V0 - v0) * S + S/2 + 5], outline=(255, 255, 255))
lines = []
for i, c in enumerate(comps):
    a, b, e, f = c['bbox']
    col = (80, 200, 120) if c['passes'] else (240, 200, 60)
    d.rectangle([(a - u0) * S - 3, (b - v0) * S - 3, (e - u0 + 1) * S + 2, (f - v0 + 1) * S + 2], outline=col, width=3)
    d.text(((a - u0) * S, (f - v0 + 1) * S + 6), f"C{i+1}", fill=col)
    lines.append(f"C{i+1}: centroid {c['centroid']}  d={c['d']}px  aspect={c['aspect']}  dist={c['dist']}px  Eq.11 {'PASS' if c['passes'] else 'fail'}")
y = side * S + 8
for s in ["White circle = principal point (80,60). Green = passes Eq.11, amber = fails."] + lines:
    d.text((8, y), s, fill=(220, 220, 220)); y += 22
im.save(out)
print('\n'.join(lines))
