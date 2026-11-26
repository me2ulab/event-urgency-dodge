"""Remove the green box from manuscript Fig. 6 (top-right panel) and restore the events it covered.

usage: remove_green_box.py <fig6_in.png> <raw t=3.60 events png> <fig6_out.png>
"""
import sys
from PIL import Image
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from check_green_box import decode

fig_in, raw, out = sys.argv[1:4]
im = Image.open(fig_in).convert('RGBA')
X0, Y0, S = 1011.0, 221.0, 2.72          # top-right panel: sensor (0,0) origin and px per sensor px
BG, ON, OFF = (10, 10, 15, 255), (234, 56, 56, 255), (51, 122, 242, 255)
BOX = (1219, 343, 1237, 357)              # green-box footprint (incl. anti-aliasing), x0,y0,x1,y1 inclusive

def is_green(p): return p[1] > 140 and p[0] < 190 and p[2] < 175 and p[1] > p[0] + 20 and p[1] > p[2] + 15
# 1) wipe the footprint to background, keeping the yellow crop-box edge (rows <= 343 belong to it)
yellow_aa = im.getpixel((1215, 343))
for y in range(BOX[1], BOX[3] + 1):
    for x in range(BOX[0], BOX[2] + 1):
        im.putpixel((x, y), yellow_aa if y == 343 else BG)
# 2) redraw the true events under the box from the raw frame
cells = [(u, v, p) for u, v, p in decode(raw) if 76 <= u <= 83 and 44 <= v <= 50]
for u, v, p in cells:
    for y in range(int(Y0 + v * S + 0.5), int(Y0 + (v + 1) * S + 0.5)):
        for x in range(int(X0 + u * S + 0.5), int(X0 + (u + 1) * S + 0.5)):
            if y >= 344: im.putpixel((x, y), ON if p > 0 else OFF)
left = sum(is_green(im.getpixel((x, y))) for x in range(im.width) for y in range(im.height))
im.save(out)
print('restored events:', cells, '| green pixels left in figure:', left)
