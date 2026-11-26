"""Pilot: one nominal ray-traced run (1.5 m/s cruise, 10 m/s ball) with a frame dump for inspection."""
import json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from ecsa_core import Params, run_episode
from sims.raytrace import RayTraceRenderer

order = sys.argv[1] if len(sys.argv) > 1 else "nearest_then_screen"
P = Params().with_(perception={"select_order": order})
r = run_episode(RayTraceRenderer(P), P, name=f"pilot_rt_{order}", snapshot_times=(1.35, 3.60, 3.75, 3.90), verbose=False)
print(json.dumps(r.summary, indent=1, default=float))
out = ROOT / "results" / "pilot"; out.mkdir(parents=True, exist_ok=True)
np.savez_compressed(out / f"{r.name}.npz", **r.log)
import cv2
for t, s in r.snapshots.items():
    img = (np.clip(s["image"], 0, 1) * 255).astype(np.uint8)
    pol = s["polarity"]; ev = np.zeros(pol.shape + (3,), np.uint8) + 12
    ev[pol > 0] = (56, 56, 234); ev[pol < 0] = (242, 122, 51)
    both = np.hstack([cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), ev])
    cv2.imwrite(str(out / f"{r.name}_t{t:.2f}.png"), cv2.resize(both, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
L = r.log
m = (L["t"] > 3.4) & (L["t"] < 4.3)
for i in np.nonzero(m)[0]:
    print(f"t={L['t'][i]:.3f} y={L['y'][i]:+.3f} vy={L['vy'][i]:+.2f} ph={L['phase'][i]:8s} ncomp={L['n_comp'][i]} d={L['det_d'][i]:5.1f} cu={L['det_cu'][i]:5.1f} trk={L['track'][i]} loom={L['looming'][i]} ddot={L['ddot'][i]:6.1f} tau={L['tau'][i]:.3f} Zd={L['closing'][i]:5.1f} G={L['gate'][i]} H={L['H'][i]} sep={L['sep'][i]:.2f}")
