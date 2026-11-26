"""Foveal isolation and apparent size (Eqs. 13-21)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .params import Camera, Perception

_EIGHT = np.ones((3, 3), bool)


@dataclass
class Component:
    pixels: np.ndarray        # (n, 2) fired pixels, columns (u, v), full-image coordinates
    events: int               # total event count over the fired pixels (tie-break, Eq. 16)
    centroid: tuple           # (cu, cv)
    bbox: tuple               # (umin, vmin, umax, vmax)
    d_bbox: float             # Eq. 18
    d_eq: float               # Eq. 19
    d: float                  # Eq. 20
    aspect: float             # (umax-umin+1)/(vmax-vmin+1)
    dist: float               # distance of centroid to the principal point
    passes: bool              # Eq. 21

    @property
    def n(self) -> int:
        return len(self.pixels)


@dataclass
class Detection:
    selected: Component | None     # b* after the screen, or None
    components: list               # all Eq. 15 components (for display / diagnostics)
    nearest: Component | None      # Eq. 16 choice before screening


def roi_bounds(cam: Camera, r: int):
    """Integer pixel range of Ω (Eq. 13): |u - u0| <= r, |v - v0| <= r."""
    u_lo, u_hi = int(np.ceil(cam.u0 - r)), int(np.floor(cam.u0 + r))
    v_lo, v_hi = int(np.ceil(cam.v0 - r)), int(np.floor(cam.v0 + r))
    return u_lo, u_hi, v_lo, v_hi


def components(count_img: np.ndarray, cam: Camera, cfg: Perception) -> list[Component]:
    u_lo, u_hi, v_lo, v_hi = roi_bounds(cam, cfg.roi_radius)
    n = count_img[v_lo:v_hi + 1, u_lo:u_hi + 1]
    fired = n >= 1
    dil = ndimage.binary_dilation(fired, structure=_EIGHT)            # Eq. 14 (inside Ω only)
    lab, k = ndimage.label(dil, structure=_EIGHT)                     # 8-connected
    out = []
    for i in range(1, k + 1):
        vv, uu = np.nonzero((lab == i) & fired)
        if len(uu) < cfg.n_min:                                       # Eq. 15
            continue
        uu_f, vv_f = uu + u_lo, vv + v_lo
        cu, cv = float(uu_f.mean()), float(vv_f.mean())
        umin, umax, vmin, vmax = int(uu_f.min()), int(uu_f.max()), int(vv_f.min()), int(vv_f.max())
        d_bbox = float(max(umax - umin, vmax - vmin))
        d_eq = float(2.0 * np.sqrt(len(uu) / np.pi))
        d = max(d_bbox, d_eq)
        aspect = (umax - umin + 1) / (vmax - vmin + 1)
        dist = float(np.hypot(cu - cam.u0, cv - cam.v0))
        passes = (cfg.d_min <= d <= cfg.d_max and cfg.aspect_lo <= aspect <= cfg.aspect_hi
                  and dist <= cfg.roi_radius)
        out.append(Component(np.stack([uu_f, vv_f], 1), int(n[vv, uu].sum()), (cu, cv),
                             (umin, vmin, umax, vmax), d_bbox, d_eq, d, aspect, dist, passes))
    return out


def detect(count_img: np.ndarray, cam: Camera, cfg: Perception) -> Detection:
    comps = components(count_img, cam, cfg)
    key = lambda c: (c.dist, -c.events)                               # nearest; ties by count
    nearest = min(comps, key=key) if comps else None
    if cfg.select_order == "nearest_then_screen":                     # Eqs. 16 then 21
        sel = nearest if (nearest is not None and nearest.passes) else None
    elif cfg.select_order == "screen_then_nearest":
        ok = [c for c in comps if c.passes]
        sel = min(ok, key=key) if ok else None
    else:
        raise ValueError(cfg.select_order)
    return Detection(sel, comps, nearest)
