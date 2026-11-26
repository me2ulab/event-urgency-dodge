"""Sidestep controller (Eqs. 32-35), kinematic plant (Eqs. 1-2) and ball (Eqs. 36-38)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .params import Params


class Phase:
    TAKEOFF, CRUISE, SIDESTEP, HOLD, RETURN, GOAL = "takeoff", "cruise", "sidestep", "hold", "return", "goal"


@dataclass
class Command:
    v: np.ndarray                 # (vx, vy, vz) world frame
    phase: str
    tau_used: float = float("nan")


class Controller:
    """The five-branch law of Eq. 35 plus goal arrival."""

    def __init__(self, P: Params):
        self.P = P
        self.c = P.control
        self.dt = P.camera.dt
        self.reset()

    def reset(self) -> None:
        self.sign = 0
        self.cleared = False
        self.done_return = False
        self.goal_t: float | None = None
        self.tau_last: float | None = None
        self.tau_last_t: float | None = None

    def sign_from_image(self, cu: float) -> int:
        """Eq. 34 as printed: s = -1 if c_u >= u0 + 1, else +1."""
        return -1 if cu >= self.P.camera.u0 + 1 else +1

    def tau_for_command(self, t: float, tau_now: float) -> float:
        """tau used by Eq. 33 (see Control.tau_mode). The first call is on the commit tick."""
        if self.c.tau_mode == "commit_countdown" and self.tau_last is not None:
            return max(self.tau_last - (t - self.tau_last_t), 1e-3)
        if math.isfinite(tau_now) and tau_now > 0:
            self.tau_last, self.tau_last_t = tau_now, t
            return tau_now
        if self.tau_last is None:
            return math.inf
        return max(self.tau_last - (t - self.tau_last_t), 1e-3)

    def step(self, t: float, pos: np.ndarray, committed: bool, commit_cu: float | None,
             tau_now: float, ball_x: float | None) -> Command:
        c, dt = self.c, self.dt
        x, y, z = pos
        if self.goal_t is not None or (c.goal_x - x) <= c.goal_tol:
            if self.goal_t is None:
                self.goal_t = t
            return Command(np.zeros(3), Phase.GOAL)
        if z < c.z_cruise - c.eps_z:                                      # takeoff
            return Command(np.array([0.0, 0.0, min(c.v_climb, (c.z_cruise - z) / dt)]), Phase.TAKEOFF)
        vz = (c.z_cruise - z) / dt if abs(c.z_cruise - z) > 1e-12 else 0.0   # hold altitude exactly
        if committed and c.enabled:
            if self.sign == 0:
                self.sign = self.sign_from_image(commit_cu)
            s = self.sign
            if not self.cleared and ball_x is not None and ball_x < x - c.eps_x:
                self.cleared = True                                       # ball behind (Eq. 35)
            if not self.cleared:
                if s * y < c.y_clear:                                     # sidestep, Eqs. 32-33
                    tau = self.tau_for_command(t, tau_now)
                    dy = max(0.0, c.y_clear - s * y)
                    vy = s * min(max(dy / tau, c.v_min), c.v_max)
                    return Command(np.array([c.v_cruise, vy, vz]), Phase.SIDESTEP, tau)
                return Command(np.array([c.v_cruise, 0.0, vz]), Phase.HOLD)
            if not self.done_return and abs(y) > c.eps_y:                 # return to the centreline
                vy = -math.copysign(min(c.v_ret, abs(y) / dt), y)
                return Command(np.array([c.v_cruise, vy, vz]), Phase.RETURN)
            self.done_return = True
        return Command(np.array([c.v_cruise, 0.0, vz]), Phase.CRUISE)


class Ball:
    """Spawned at (x_launch, 0, z_cruise); constant velocity from t_launch (Eq. 38); stops at the back wall."""

    def __init__(self, P: Params):
        e, c = P.encounter, P.control
        self.present = e.ball_present
        self.p0 = np.array([e.x_launch, 0.0, c.z_cruise])
        aim = np.array([e.x_aim, 0.0, c.z_cruise])
        self.v = (aim - self.p0) / np.linalg.norm(aim - self.p0) * e.ball_speed
        self.t_launch = e.t_launch(c)
        self.x_stop = P.corridor.x_min + e.ball_radius

    def pos(self, t: float) -> np.ndarray:
        p = self.p0 + self.v * max(0.0, t - self.t_launch)
        if p[0] < self.x_stop:
            p = self.p0 + self.v * ((self.x_stop - self.p0[0]) / self.v[0])
        return p

    def vel(self, t: float) -> np.ndarray:
        return self.v.copy() if t >= self.t_launch and self.pos(t)[0] > self.x_stop + 1e-9 else np.zeros(3)


def segment_min_distance(p0, v_uav, b0, v_ball, dt) -> float:
    """Minimum of |(p0 + v t) - (b0 + w t)| for t in [0, dt]: separation between ticks (Eq. 39)."""
    r0 = np.asarray(p0) - np.asarray(b0)
    dv = np.asarray(v_uav) - np.asarray(v_ball)
    den = float(dv @ dv)
    s = 0.0 if den == 0 else min(max(-float(r0 @ dv) / den, 0.0), dt)
    return float(np.linalg.norm(r0 + dv * s))
