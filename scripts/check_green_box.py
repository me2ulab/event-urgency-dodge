"""Decode the surviving t=3.60 s event frame (960x720 = 6x of 160x120) and run the manuscript detector."""
import sys, json
from PIL import Image
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import foveal_detector as fd

def decode(path, scale=6):
    im = Image.open(path).convert('RGB')
    ev = []
    for v in range(fd.H):
        for u in range(fd.W):
            r, g, b = im.getpixel((u * scale + scale // 2, v * scale + scale // 2))
            if r > 150 and g < 120: ev.append((u, v, +1))
            elif b > 150 and r < 120: ev.append((u, v, -1))
    return ev

if __name__ == '__main__':
  for t in sys.argv[1:]:
    ev = decode(t)
    best, comps = fd.detect(ev)
    print(f"\n{t}: {len(ev)} events, {sum(1 for u,v,_ in ev if abs(u-80)<=16 and abs(v-60)<=16)} in crop")
    for c in comps:
        print(('  SELECTED ' if c is best else '           '), json.dumps(c))
