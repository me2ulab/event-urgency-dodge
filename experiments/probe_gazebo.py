"""Probe the lockstep renderer: each returned frame must equal a settled render of the pose just set."""
import sys, time
from pathlib import Path
import numpy as np, cv2
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from urgency_dodge import Params
from sims.gz_lockstep import GazeboLockstepRenderer
P = Params(); out = ROOT / "results" / "gz_probe"; out.mkdir(parents=True, exist_ok=True)
r = GazeboLockstepRenderer(P)
poses = [((5.0, 0, 1.5), (12.0, 0, 1.5)), ((5.0, 0, 1.5), (6.5, 0, 1.5)), ((1.0, 0, 1.5), None),
         ((5.39, 0.3, 1.5), (10.5, 0, 1.5)), ((8.0, -0.5, 1.5), (9.2, 0.1, 1.5))]
def go(p, t): return r.render(np.array(p[0], float), None if p[1] is None else np.array(p[1], float), t)
ref = []
for i, p in enumerate(poses):                       # settled reference: same pose rendered 4 times
    for k in range(4): img = go(p, 0.0)
    ref.append(img)
seq = [0, 1, 2, 3, 4, 2, 0, 3, 1, 4, 4, 0]
bad = 0
for j, i in enumerate(seq):
    img = go(poses[i], j * 0.025)
    match = [int(np.abs(img.astype(int) - rr).max()) for rr in ref]
    ok = match[i] == 0
    bad += not ok
    print(f"tick {j:2d} pose {i}: max|diff| vs refs {match}  {'OK' if ok else 'MISMATCH'}")
print("mismatches:", bad, "of", len(seq))
cv2.imwrite(str(out / "probe.png"), cv2.resize(np.hstack(ref), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
t1 = time.time(); [go(((1 + 0.0375 * k, 0, 1.5), (12.0, 0, 1.5)), k * 0.025) for k in range(80)]
print(f"80 lockstep ticks in {time.time()-t1:.2f}s")
r.close()
