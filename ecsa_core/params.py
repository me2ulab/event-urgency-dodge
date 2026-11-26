"""All parameters of the method, with the equation each comes from.

Source of truth: PHD/ecsa_paper.docx (the full-length ECSA-13 draft; equation numbers below refer
to it) and manuscript.v4.docx (the proceedings version). Values marked RECONSTRUCTED are not
stated in either document and were chosen as described in RECOVERY.md.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict, replace


@dataclass(frozen=True)
class Camera:                      # Eqs. 3-5
    width: int = 160
    height: int = 120
    fx: float = 120.0              # = W / (2 tan(theta_h/2)), theta_h = 1.176 rad
    rate_hz: float = 40.0          # camera tick; dt = 25 ms (Eq. 2)

    @property
    def fy(self) -> float:
        return self.fx

    @property
    def u0(self) -> float:         # Eq. 4: principal point at the array centre
        return 0.5 * (self.width - 1)

    @property
    def v0(self) -> float:
        return 0.5 * (self.height - 1)

    @property
    def hfov(self) -> float:       # Eq. 5 inverted
        return 2.0 * math.atan(self.width / (2.0 * self.fx))

    @property
    def dt(self) -> float:
        return 1.0 / self.rate_hz


@dataclass(frozen=True)
class EventModel:                  # Eqs. 6-12
    contrast: float = 0.20         # C
    n_max: int = 8                 # cap on crossings per frame interval (Eq. 9)
    window_s: float = 0.050        # T_w sliding window (Eq. 12)
    log_eps: float = 1e-3          # RECONSTRUCTED: L = log(I + eps), I in [0, 1]


@dataclass(frozen=True)
class Perception:                  # Eqs. 13-21
    roi_radius: int = 16           # r (Eq. 13)
    n_min: int = 4                 # fired pixels per component (Eq. 15)
    d_min: float = 3.0             # detection screen (Eq. 21)
    d_max: float = 18.0
    aspect_lo: float = 0.5
    aspect_hi: float = 2.0
    # Eq. 16 picks the component nearest the principal point and Eq. 21 then screens it.
    # "screen_then_nearest" applies Eq. 21 to every component first (see GREEN_BOX_CHECK.md).
    select_order: str = "nearest_then_screen"


@dataclass(frozen=True)
class Tracking:                    # Eqs. 22-25
    m: int = 4                     # growth window (samples)
    delta: float = 1.5             # px, minimum growth over the window
    eps_r: float = 0.75            # px, allowed reversal per step
    eps_s: float = 10.0            # px, maximum jump per step
    gate_px: float = 18.0          # g, nearest-neighbour association gate
    m_clear: int = 4               # clear diameter history after this many misses
    m_delete: int = 5              # delete track after more than this many misses
    hunt_after_throw: bool = True  # "Hunt starts after the scheduled throw" (Sec. 3.6)


@dataclass(frozen=True)
class Gate:                        # Eqs. 26-31
    ddot_floor: float = 2.0        # px/s; below this tau = inf (Eq. 27)
    d_gate: float = 6.0            # px
    tau_min: float = 0.12          # s
    tau_max: float = 2.0           # s (tau_0)
    zdot_min: float = 3.0          # m/s, closing speed band (Eq. 30)
    zdot_max: float = 30.0
    radius_assumed: float = 0.15   # R used only in Eq. 29 for the closing-speed clause
    h_star: int = 3                # consecutive passing ticks to commit (Eq. 31)


@dataclass(frozen=True)
class Control:                     # Eqs. 32-35
    v_cruise: float = 1.5
    v_climb: float = 1.5
    y_clear: float = 0.85          # y_c (Eq. 32)
    v_min: float = 0.8             # Eq. 33 clip
    v_max: float = 5.0
    v_ret: float = 2.0             # return speed (Eq. 35)
    z_cruise: float = 1.5
    eps_z: float = 0.02            # RECONSTRUCTED takeoff tolerance
    eps_x: float = 0.60            # RECONSTRUCTED: ball "behind" margin; fits Fig. 7 return onset
    eps_y: float = 1e-6            # return ends exactly on the centreline (last step is -y/dt)
    goal_x: float = 24.0
    goal_tol: float = 0.60         # RECONSTRUCTED: goal reached at 15.75 s (Table 1) => ~0.6 m
    enabled: bool = True           # False = avoidance-off control
    # RECONSTRUCTED: which tau Eq. 33 divides by. "commit_countdown": tau_hat at commit minus the time
    # since commit ("covers Δy in one time-to-contact" => constant v_y). "track_latest": newest finite
    # tau of the confirmed track, counted down between updates (vulnerable to mis-association).
    tau_mode: str = "commit_countdown"


@dataclass(frozen=True)
class Encounter:                   # Eqs. 36-38, Sec. 4
    x0: float = 1.0                # pad position
    z_pad: float = 0.28
    x_launch: float = 16.0         # ball spawn (16, 0, 1.5)
    x_aim: float = 6.0
    ball_speed: float = 10.0
    ball_radius: float = 0.15
    uav_radius: float = 0.25
    ball_present: bool = True      # False = empty-corridor control

    def t_takeoff(self, c: Control) -> float:            # Eq. 36
        return (c.z_cruise - self.z_pad) / c.v_climb

    def t_launch(self, c: Control) -> float:             # Eq. 37
        return max(0.0, self.t_takeoff(c) + (self.x_aim - self.x0) / c.v_cruise
                   - (self.x_launch - self.x_aim) / self.ball_speed)


@dataclass(frozen=True)
class Corridor:                    # Sec. 4: 28 m x 4 m
    x_min: float = -2.0            # RECONSTRUCTED placement of the 28 m along x
    x_max: float = 26.0
    half_width: float = 2.0
    ceiling: float = 3.0           # RECONSTRUCTED


@dataclass(frozen=True)
class Params:
    camera: Camera = field(default_factory=Camera)
    events: EventModel = field(default_factory=EventModel)
    perception: Perception = field(default_factory=Perception)
    tracking: Tracking = field(default_factory=Tracking)
    gate: Gate = field(default_factory=Gate)
    control: Control = field(default_factory=Control)
    encounter: Encounter = field(default_factory=Encounter)
    corridor: Corridor = field(default_factory=Corridor)
    t_max: float = 17.0

    def to_dict(self) -> dict:
        return asdict(self)

    def with_(self, **groups) -> "Params":
        """Params().with_(control={'v_cruise': 2.0}, encounter={'ball_speed': 12})"""
        new = self
        for name, changes in groups.items():
            if isinstance(changes, dict):
                new = replace(new, **{name: replace(getattr(new, name), **changes)})
            else:
                new = replace(new, **{name: changes})
        return new
