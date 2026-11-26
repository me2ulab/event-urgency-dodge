"""Ray-traced corridor (the 'custom simulator', manuscript Fig. 5 / ecsa_paper Table 2).

Flat-albedo, point-sampled (one ray per pixel centre), deterministic:
  * side walls, floor and ceiling carry axial black/white stripes (bands across x),
  * the end wall carries a checkerboard,
  * dark 'shelf' spheres stand along both walls,
  * the ball is a checker-patterned sphere.
Camera at the UAV centre, heading +x (Eq. 1: heading held fixed), pinhole of Eqs. 3-5.
RECONSTRUCTED: stripe width, checker size, sphere layout and albedos were read off the surviving
render (PHD/fig6_kin_image_events.png); the original values are not in the papers.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ecsa_core.params import Params

WHITE, BLACK = 0.90, 0.05


@dataclass(frozen=True)
class Look:
    stripe_w: float = 1.0                 # m, axial stripe band width
    checker_end: float = 1.0              # m, end-wall checker square
    shelf_r: float = 0.40                 # m, shelf sphere radius
    shelf_y: float = 1.55                 # |y| of shelf sphere centres
    shelf_z: float = 1.50
    shelf_xs: tuple = tuple(float(x) for x in range(3, 25, 2))   # every 2 m, as in the published render
    shelf_albedo: float = 0.02
    ball_lon: int = 8                     # checker cells around the ball
    ball_lat: int = 4


class RayTraceRenderer:
    name = "raytrace"

    def __init__(self, P: Params, look: Look = Look()):
        self.P, self.L = P, look
        cam = P.camera
        u = np.arange(cam.width) - cam.u0
        v = np.arange(cam.height) - cam.v0
        uu, vv = np.meshgrid(u, v)
        d = np.stack([np.ones_like(uu), -uu / cam.fx, -vv / cam.fy], -1)   # camera -> world (x fwd, y left, z up)
        self.dirs = d / np.linalg.norm(d, axis=-1, keepdims=True)
        shelves = [(x, s * look.shelf_y, look.shelf_z) for x in look.shelf_xs for s in (+1, -1)]
        self.shelves = np.array(shelves, float)

    @staticmethod
    def _sphere_t(o, d, c, r):
        oc = o - c
        b = d @ oc
        cc = oc @ oc - r * r
        disc = b * b - cc
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        return np.where((disc >= 0) & (t > 1e-6), t, np.inf)

    def render(self, uav_pos, ball_pos, t):
        P, L = self.P, self.L
        co = P.corridor
        o = np.asarray(uav_pos, float)
        d = self.dirs
        best = np.full(d.shape[:2], np.inf)
        shade = np.zeros(d.shape[:2])
        # --- planes of the corridor box ---
        planes = [(1, +co.half_width), (1, -co.half_width), (2, 0.0), (2, co.ceiling), (0, co.x_max), (0, co.x_min)]
        for axis, val in planes:
            with np.errstate(divide="ignore", invalid="ignore"):
                tp = (val - o[axis]) / d[..., axis]
            tp = np.where(tp > 1e-6, tp, np.inf)
            with np.errstate(invalid="ignore"):
                hit = o + d * tp[..., None]
            if axis == 0:
                ok = (np.abs(hit[..., 1]) <= co.half_width + 1e-9) & (hit[..., 2] >= -1e-9) & (hit[..., 2] <= co.ceiling + 1e-9)
                with np.errstate(invalid="ignore"):
                    cell = np.floor(hit[..., 1] / L.checker_end) + np.floor(hit[..., 2] / L.checker_end)
            else:
                ok = (hit[..., 0] >= co.x_min - 1e-9) & (hit[..., 0] <= co.x_max + 1e-9)
                cell = np.floor(hit[..., 0] / L.stripe_w)
            cell = np.nan_to_num(cell, nan=0.0, posinf=0.0, neginf=0.0)
            alb = np.where(np.mod(cell, 2) == 0, WHITE, BLACK)
            m = ok & (tp < best)
            best = np.where(m, tp, best); shade = np.where(m, alb, shade)
        # --- shelf spheres ---
        for c in self.shelves:
            ts = self._sphere_t(o, d, c, L.shelf_r)
            m = ts < best
            best = np.where(m, ts, best); shade = np.where(m, L.shelf_albedo, shade)
        # --- the ball: checker in body-fixed longitude/latitude ---
        if ball_pos is not None:
            R = P.encounter.ball_radius
            c = np.asarray(ball_pos, float)
            ts = self._sphere_t(o, d, c, R)
            m = ts < best
            if m.any():
                hp = o + d[m] * ts[m][:, None] - c
                lon = np.arctan2(hp[:, 1], hp[:, 0])
                lat = np.arcsin(np.clip(hp[:, 2] / R, -1, 1))
                cell = np.floor((lon + np.pi) / (2 * np.pi) * L.ball_lon) + np.floor((lat + np.pi / 2) / np.pi * L.ball_lat)
                best[m] = ts[m]
                shade[m] = np.where(np.mod(cell, 2) == 0, WHITE, BLACK)
        # 8-bit frame, like a real camera and like the Gazebo path (and identical through ROS images)
        return np.round(shade * 255).astype(np.uint8)
