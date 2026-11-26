"""One camera tick of the closed loop, and a full episode with scoring (Eqs. 39-41).

The same `EventFront` and `Avoider` objects are used by the offline runner and by the ROS 2 nodes,
so the two paths execute identical code.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .control import Ball, Controller, Phase, segment_min_distance
from .events import EventCamera, EventWindow
from .params import Params
from .perception import detect
from .tracking import LoomingTracker


class EventFront:
    """Frames -> events (Eqs. 6-11) -> 50 ms window (Eq. 12) -> per-pixel counts."""

    def __init__(self, P: Params):
        cam = P.camera
        self.P = P
        self.cam = EventCamera(P.events, cam.width, cam.height)
        self.win = EventWindow(P.events.window_s)

    def step(self, img: np.ndarray, t: float):
        ev = self.cam.step(img, t)
        self.win.push(ev, t)
        return ev, self.win.count_image(self.P.camera.width, self.P.camera.height)


@dataclass
class TickOut:
    det: object
    gate: object
    cmd: object
    t_perc_ms: float
    t_cmd_ms: float


class Avoider:
    """Counts image -> foveal detection -> looming track -> gate -> velocity command."""

    def __init__(self, P: Params):
        self.P = P
        self.tracker = LoomingTracker(P.camera, P.tracking, P.gate)
        self.ctrl = Controller(P)
        self.t_hunt = P.encounter.t_launch(P.control) if P.tracking.hunt_after_throw else -math.inf

    def step(self, counts: np.ndarray, t: float, pos: np.ndarray, ball_x: float | None) -> TickOut:
        t0 = time.perf_counter()
        det = detect(counts, self.P.camera, self.P.perception)
        t1 = time.perf_counter()
        if t >= self.t_hunt - 1e-9 and self.P.control.enabled:
            g = self.tracker.step(det.selected, t)
        else:
            g = self.tracker.evaluate(None)
        ci = self.tracker.commit_info
        # Eq. 33 reads tau of the confirmed track only; estimates from other tracks are ignored
        tau_ctrl = g.tau if (ci and g.track is not None and g.track.tid == ci["track"]) else math.inf
        cmd = self.ctrl.step(t, pos, self.tracker.committed, ci["cu"] if ci else None, tau_ctrl, ball_x)
        t2 = time.perf_counter()
        return TickOut(det, g, cmd, (t1 - t0) * 1e3, (t2 - t0) * 1e3)


@dataclass
class EpisodeResult:
    name: str
    log: dict                         # column arrays, one row per tick
    summary: dict
    snapshots: dict = field(default_factory=dict)   # t -> (image, counts, polarity, detection)


def run_episode(renderer, P: Params, name: str = "run", snapshot_times=(), t_max: float | None = None,
                verbose: bool = False) -> EpisodeResult:
    """Closed loop: render at the current pose -> events -> perception -> command -> integrate (Eq. 2).

    `renderer.render(uav_pos, ball_pos, t)` returns a grayscale frame (uint8 or float in [0, 1]).
    Ball pose enters the loop only for rendering, scoring, and the Eq. 35 'ball behind' test.
    """
    dt = P.camera.dt
    if t_max is None:   # long enough for the slowest cruise to reach the goal
        c, e = P.control, P.encounter
        t_max = max(P.t_max, e.t_takeoff(c) + (c.goal_x - e.x0) / c.v_cruise + 2.0)
    front, av, ball = EventFront(P), Avoider(P), Ball(P)
    pos = np.array([P.encounter.x0, 0.0, P.encounter.z_pad])
    cols = {k: [] for k in ("t x y z vx vy vz bx by bz n_ev n_roi n_comp det_d det_cu det_cv "
                            "track looming ddot tau closing gate H committed phase tau_cmd sep "
                            "t_perc_ms t_cmd_ms").split()}
    snaps, want = {}, sorted(snapshot_times)
    sep_min, sep_min_t = math.inf, None
    first_gate_t = first_vy_t = None
    hit = False
    k = 0
    while True:
        t = round(k * dt, 9)
        bpos = ball.pos(t) if ball.present else np.array([1e3, 0.0, 0.0])
        img = renderer.render(pos.copy(), bpos.copy() if ball.present else None, t)
        ev, counts = front.step(img, t)
        out = av.step(counts, t, pos, float(bpos[0]) if ball.present else None)
        v = out.cmd.v
        g, det = out.gate, out.det
        if g.passed and first_gate_t is None:
            first_gate_t = t
        if abs(v[1]) > 0 and first_vy_t is None:
            first_vy_t = t
        sep = float(np.linalg.norm(pos - bpos)) if ball.present else math.inf
        if ball.present:
            smin = segment_min_distance(pos, v, bpos, ball.vel(t), dt)
            if smin < sep_min:
                sep_min = smin
                sep_min_t = t
        sel = det.selected
        row = dict(t=t, x=pos[0], y=pos[1], z=pos[2], vx=v[0], vy=v[1], vz=v[2],
                   bx=bpos[0], by=bpos[1], bz=bpos[2], n_ev=len(ev), n_roi=int(sum(c.n for c in det.components)),
                   n_comp=len(det.components), det_d=sel.d if sel else np.nan,
                   det_cu=sel.centroid[0] if sel else np.nan, det_cv=sel.centroid[1] if sel else np.nan,
                   track=g.track.tid if g.track else -1, looming=g.looming, ddot=g.ddot, tau=g.tau,
                   closing=g.closing, gate=g.passed, H=av.tracker.H, committed=av.tracker.committed,
                   phase=out.cmd.phase, tau_cmd=out.cmd.tau_used, sep=sep,
                   t_perc_ms=out.t_perc_ms, t_cmd_ms=out.t_cmd_ms)
        for kk, vv in row.items():
            cols[kk].append(vv)
        while want and t >= want[0] - 1e-9:
            snaps[want.pop(0)] = dict(t=t, image=np.array(img), counts=counts.copy(),
                                      polarity=front.win.polarity_image(P.camera.width, P.camera.height),
                                      detection=det, pos=pos.copy(), ball=bpos.copy())
        if verbose and (k % 40 == 0 or out.cmd.phase in (Phase.SIDESTEP,)):
            print(f"t={t:6.3f} x={pos[0]:6.2f} y={pos[1]:+.3f} z={pos[2]:.2f} ph={out.cmd.phase:8s} "
                  f"d={row['det_d']:.1f} tau={g.tau:.3f} H={av.tracker.H} sep={sep:.2f}")
        if ball.present and sep_min < P.encounter.uav_radius + P.encounter.ball_radius:
            hit = True
        if out.cmd.phase == Phase.GOAL or t >= t_max or (hit and ball.present and t > sep_min_t + 0.5):
            break
        pos = pos + v * dt                                                 # Eq. 2
        k += 1
    log = {k_: np.array(v_) for k_, v_ in cols.items()}
    ci = av.tracker.commit_info
    tau_true = None
    if ci is not None and ball.present:
        i = int(np.argmin(np.abs(log["t"] - ci["t"])))
        tau_true = (log["bx"][i] - log["x"][i]) / (P.control.v_cruise + abs(ball.v[0]))    # Eq. 40
    summary = dict(
        name=name, v_cruise=P.control.v_cruise, ball_speed=P.encounter.ball_speed,
        ball_present=ball.present, avoidance=P.control.enabled,
        t_launch=round(ball.t_launch, 4),
        commit_t=ci["t"] if ci else None, commit_d=ci["d"] if ci else None,
        tau_hat=ci["tau"] if ci else None, tau_true=tau_true,
        first_gate_t=first_gate_t, first_vy_t=first_vy_t,
        sep_min=None if not ball.present else round(sep_min, 4),
        sep_min_tick=None if not ball.present else round(float(np.min(log["sep"])), 4),   # per-tick sample
        sep_min_t=sep_min_t, collision=hit,
        goal_t=av.ctrl.goal_t, y_max=float(np.max(np.abs(log["y"]))),
        dodged=bool(np.any(np.abs(log["vy"]) > 0)),
        t_cmd_ms_mean=float(np.mean(log["t_cmd_ms"])), t_perc_ms_mean=float(np.mean(log["t_perc_ms"])),
    )
    return EpisodeResult(name, log, summary, snaps)
