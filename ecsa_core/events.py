"""ESIM-style event synthesis between rendered frames (Eqs. 6-11) and the sliding window (Eq. 12)."""
from __future__ import annotations

import numpy as np

from .params import EventModel

# structured event array: x, y (pixel), t (s), p (+1/-1)
EVENT_DTYPE = np.dtype([("x", np.int16), ("y", np.int16), ("t", np.float64), ("p", np.int8)])


def log_intensity(img: np.ndarray, eps: float) -> np.ndarray:
    """Eq. 6. `img` is float in [0, 1] or uint8 (scaled to [0, 1])."""
    if img.dtype == np.uint8:
        img = img.astype(np.float64) / 255.0
    return np.log(img.astype(np.float64) + eps)


class EventCamera:
    """Contrast-threshold event generator fed with frames at the camera rate.

    For every pixel the change from its reference ΔL = L(t_k) - L_ref is converted into
    N = min(floor(|ΔL| / C), N_max) events (Eq. 9) at linearly interpolated times (Eq. 10), and the
    reference is advanced by exactly the N crossings consumed (Eq. 11).
    """

    def __init__(self, cfg: EventModel, width: int, height: int):
        self.cfg = cfg
        self.w, self.h = width, height
        self.L_ref: np.ndarray | None = None
        self.t_prev: float | None = None

    def reset(self) -> None:
        self.L_ref, self.t_prev = None, None

    def step(self, img: np.ndarray, t: float) -> np.ndarray:
        L = log_intensity(img, self.cfg.log_eps)
        if self.L_ref is None:                         # first frame only initialises the reference
            self.L_ref, self.t_prev = L.copy(), t
            return np.zeros(0, EVENT_DTYPE)
        C = self.cfg.contrast
        dL = L - self.L_ref
        n = np.minimum(np.floor(np.abs(dL) / C), self.cfg.n_max).astype(np.int32)
        ys, xs = np.nonzero(n)
        if len(xs) == 0:
            self.t_prev = t
            return np.zeros(0, EVENT_DTYPE)
        counts = n[ys, xs]
        d = dL[ys, xs]
        pol = np.sign(d).astype(np.int8)                # Eq. 8
        total = int(counts.sum())
        ev = np.empty(total, EVENT_DTYPE)
        rep = np.repeat(np.arange(len(xs)), counts)
        m = np.arange(total) - np.repeat(np.cumsum(counts) - counts, counts) + 1   # 1..N per pixel
        dt = t - self.t_prev
        ev["x"], ev["y"], ev["p"] = xs[rep], ys[rep], pol[rep]
        ev["t"] = self.t_prev + m * C / np.abs(d[rep]) * dt                        # Eq. 10
        self.L_ref[ys, xs] += pol * counts * C                                     # Eq. 11
        self.t_prev = t
        ev.sort(order="t", kind="stable")
        return ev


class EventWindow:
    """E(t) = {e : t - T_w < t_e <= t} (Eq. 12)."""

    def __init__(self, window_s: float):
        self.window_s = window_s
        self.buf = np.zeros(0, EVENT_DTYPE)

    def reset(self) -> None:
        self.buf = np.zeros(0, EVENT_DTYPE)

    def push(self, ev: np.ndarray, t: float) -> np.ndarray:
        buf = np.concatenate([self.buf, ev]) if len(ev) else self.buf
        # small tolerance so a window of exactly 2 frame periods keeps both intervals
        self.buf = buf[buf["t"] > t - self.window_s + 1e-9]
        return self.buf

    def count_image(self, w: int, h: int) -> np.ndarray:
        n = np.zeros((h, w), np.int32)
        np.add.at(n, (self.buf["y"], self.buf["x"]), 1)
        return n

    def polarity_image(self, w: int, h: int) -> np.ndarray:
        """Net polarity per pixel (for display): >0 ON, <0 OFF."""
        s = np.zeros((h, w), np.int32)
        np.add.at(s, (self.buf["y"], self.buf["x"]), self.buf["p"].astype(np.int32))
        return s
