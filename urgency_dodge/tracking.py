"""Looming tracker (Eqs. 22-25), optical tau (Eqs. 26-29) and the commit gate (Eqs. 30-31)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .params import Camera, Gate, Tracking
from .perception import Component


@dataclass
class Track:
    tid: int
    centroid: tuple
    hist: list = field(default_factory=list)     # [(t, d)]
    misses: int = 0                               # μ


@dataclass
class GateResult:
    track: Track | None = None
    looming: bool = False
    d: float = float("nan")
    ddot: float = float("nan")
    tau: float = float("inf")
    range_m: float = float("nan")                 # Eq. 29a (assumed R)
    closing: float = float("nan")                 # -Zdot, Eq. 29b
    passed: bool = False                          # G_k, Eq. 30


def looming_ok(ds: list, cfg: Tracking) -> bool:
    """Eqs. 22-23 on the last m diameters."""
    if len(ds) < cfg.m:
        return False
    w = ds[-cfg.m:]
    if w[-1] - w[0] < cfg.delta:
        return False
    return all(w[j + 1] >= w[j] - cfg.eps_r and w[j + 1] < w[j] + cfg.eps_s for j in range(cfg.m - 1))


def fit_rate(ts: list, ds: list) -> float:
    """Eq. 26: least-squares slope of d(t) on the window."""
    t = np.asarray(ts, float); d = np.asarray(ds, float)
    tm = t.mean()
    den = float(((t - tm) ** 2).sum())
    return float(((t - tm) * (d - d.mean())).sum() / den) if den > 0 else 0.0


class LoomingTracker:
    def __init__(self, cam: Camera, tcfg: Tracking, gcfg: Gate):
        self.cam, self.cfg, self.g = cam, tcfg, gcfg
        self.tracks: list[Track] = []
        self._next = 0
        self.H = 0                                  # Eq. 31 counter
        self.committed = False
        self.commit_info: dict | None = None

    def reset(self) -> None:
        self.__init__(self.cam, self.cfg, self.g)

    # -- Eqs. 24-25 ------------------------------------------------------------------------
    def associate(self, det: Component | None, t: float) -> Track | None:
        matched = None
        if det is not None:
            best, bd = None, self.cfg.gate_px
            for tr in self.tracks:
                dd = math.hypot(det.centroid[0] - tr.centroid[0], det.centroid[1] - tr.centroid[1])
                if dd < bd:
                    best, bd = tr, dd
            if best is None:
                best = Track(self._next, det.centroid); self._next += 1
                self.tracks.append(best)
            best.centroid, best.misses = det.centroid, 0
            best.hist.append((t, det.d))
            matched = best
        for tr in list(self.tracks):
            if tr is matched:
                continue
            tr.misses += 1
            if tr.misses >= self.cfg.m_clear:
                tr.hist.clear()
            if tr.misses > self.cfg.m_delete:
                self.tracks.remove(tr)
        return matched

    # -- Eqs. 26-30 ------------------------------------------------------------------------
    def evaluate(self, tr: Track | None) -> GateResult:
        r = GateResult(track=tr)
        if tr is None or not tr.hist:
            return r
        ts = [h[0] for h in tr.hist]; ds = [h[1] for h in tr.hist]
        r.d = ds[-1]
        r.looming = looming_ok(ds, self.cfg)
        if len(ds) < self.cfg.m:
            return r
        r.ddot = fit_rate(ts[-self.cfg.m:], ds[-self.cfg.m:])
        r.tau = math.inf if r.ddot < self.g.ddot_floor else r.d / r.ddot           # Eqs. 27-28
        R = self.g.radius_assumed
        r.range_m = self.cam.fx * 2 * R / r.d                                          # Eq. 29a
        r.closing = self.cam.fx * 2 * R * r.ddot / r.d ** 2                            # Eq. 29b (-Zdot)
        r.passed = (r.looming and r.d >= self.g.d_gate and self.g.tau_min < r.tau < self.g.tau_max
                    and self.g.zdot_min <= r.closing <= self.g.zdot_max)              # Eq. 30
        return r

    def step(self, det: Component | None, t: float) -> GateResult:
        tr = self.associate(det, t)
        r = self.evaluate(tr)
        self.H = self.H + 1 if r.passed else 0                                         # Eq. 31
        if not self.committed and self.H >= self.g.h_star:
            self.committed = True
            self.commit_info = dict(t=t, tau=r.tau, d=r.d, cu=tr.centroid[0], track=tr.tid)
        return r
