"""Run the paper's named experiments (names as used for the reported runs).

    python3 experiments/run.py closed_loop_ttc_check            # one experiment
    python3 experiments/run.py --suite raytrace                 # Table 1 striped rows + Table 2 sweep
    python3 experiments/run.py --suite gazebo                   # Table 1 Gazebo rows (needs gz running)
    python3 experiments/run.py --list

Each run writes results/<name>/summary.json, log.npz and snapshot frames.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from urgency_dodge import Params, run_episode  # noqa: E402

SNAP_T = (1.35, 3.60, 3.75, 3.90)


def experiments() -> dict:
    P = Params()
    ex = {
        # Table 1, striped (ray-traced) rows
        "closed_loop_ttc_check": ("raytrace", P),
        "closed_loop_noball_check": ("raytrace", P.with_(encounter={"ball_present": False})),
        "closed_loop_nododge": ("raytrace", P.with_(control={"enabled": False})),
        # Table 1, Gazebo rows
        "gz_closed_015": ("gazebo", P),
        "gz_closed_015_noball": ("gazebo", P.with_(encounter={"ball_present": False})),
        "gz_closed_nododge": ("gazebo", P.with_(control={"enabled": False})),
    }
    # Table 2 speed sweep (ray-traced only)
    for vc, vb in [(1.0, 10), (1.5, 10), (2.0, 10), (2.5, 10), (3.0, 10), (1.5, 12)]:
        ex[f"speed_sweep/uav{vc}_ball{vb}"] = ("raytrace", P.with_(control={"v_cruise": vc},
                                                                   encounter={"ball_speed": float(vb)}))
    return ex


SUITES = {
    "raytrace": lambda ex: [k for k, v in ex.items() if v[0] == "raytrace"],
    "gazebo": lambda ex: [k for k, v in ex.items() if v[0] == "gazebo"],
}


def make_renderer(kind: str, P: Params):
    if kind == "raytrace":
        from sims.raytrace import RayTraceRenderer
        return RayTraceRenderer(P)
    if kind == "gazebo":
        from sims.gz_lockstep import GazeboLockstepRenderer
        return GazeboLockstepRenderer(P)
    raise ValueError(kind)


def save(r, out: Path, P: Params) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(r.summary, indent=1, default=float))
    (out / "params.json").write_text(json.dumps(P.to_dict(), indent=1))
    np.savez_compressed(out / "log.npz", **r.log)
    snaps = {}
    for t, s in r.snapshots.items():
        det = s["detection"]
        snaps[f"{t:.2f}"] = dict(t=s["t"], pos=s["pos"].tolist(), ball=s["ball"].tolist(),
                                 selected_bbox=list(det.selected.bbox) if det.selected else None,
                                 components=[dict(bbox=list(c.bbox), d=c.d, passes=c.passes) for c in det.components])
        np.savez_compressed(out / f"snap_{t:.2f}.npz", image=s["image"], counts=s["counts"], polarity=s["polarity"])
    (out / "snapshots.json").write_text(json.dumps(snaps, indent=1))


def run(name: str, verbose=False) -> dict:
    kind, P = experiments()[name]
    rend = make_renderer(kind, P)
    try:
        r = run_episode(rend, P, name=name, snapshot_times=SNAP_T, verbose=verbose)
    finally:
        if hasattr(rend, "close"):
            rend.close()
    save(r, ROOT / "results" / name, P)
    return r.summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--suite", choices=list(SUITES))
    ap.add_argument("--list", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    ex = experiments()
    if a.list:
        print("\n".join(f"{k:32s} {v[0]}" for k, v in ex.items())); return
    names = list(a.names) + (SUITES[a.suite](ex) if a.suite else [])
    for n in names:
        s = run(n, a.verbose)
        keys = ("commit_t", "commit_d", "tau_hat", "tau_true", "sep_min", "collision", "goal_t", "dodged")
        print(f"{n:32s} " + "  ".join(f"{k}={s[k]:.3f}" if isinstance(s[k], float) else f"{k}={s[k]}" for k in keys))


if __name__ == "__main__":
    main()
