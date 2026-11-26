"""Unit and closed-loop tests of urgency_dodge against the equations of the paper (Eqs. 1-41).

Run inside the sim container:  python3 -m pytest tests -q
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from urgency_dodge import Params, run_episode  # noqa: E402
from urgency_dodge.control import Ball, Controller, segment_min_distance  # noqa: E402
from urgency_dodge.events import EventCamera, EventWindow, EVENT_DTYPE  # noqa: E402
from urgency_dodge.perception import detect, roi_bounds  # noqa: E402
from urgency_dodge.tracking import LoomingTracker, fit_rate, looming_ok  # noqa: E402

P = Params()


# ---------------------------------------------------------------- camera and encounter (Eqs. 3-5, 36-37)
def test_camera_matches_paper():
    cam = P.camera
    assert abs(cam.hfov - 1.176) < 1e-3          # "theta_h = 1.176 rad"
    assert (cam.u0, cam.v0) == (79.5, 59.5)      # Eq. 4
    assert cam.dt == 0.025


def test_encounter_timing_matches_paper():
    c, e = P.control, P.encounter
    assert round(e.t_takeoff(c), 2) == 0.81      # "T_takeoff = 0.81 s"
    assert round(e.t_launch(c), 2) == 3.15       # "t_l = 3.15 s", the throw time of Fig. 7
    b = Ball(P)
    assert np.allclose(b.pos(0.0), [16, 0, 1.5])
    assert np.allclose(b.pos(e.t_launch(c) + 0.5), [11, 0, 1.5])


# ---------------------------------------------------------------- event model (Eqs. 6-12)
def test_esim_counts_times_and_reference():
    cfg = P.events
    cam = EventCamera(cfg, 2, 1)
    eps = cfg.log_eps
    i0 = np.array([[0.2, 0.2]])
    assert len(cam.step(i0, 0.0)) == 0                         # first frame initialises
    # pixel 0 brightens by 2.5 C in log space, pixel 1 unchanged
    i1 = np.array([[(0.2 + eps) * math.exp(2.5 * cfg.contrast) - eps, 0.2]])
    ev = cam.step(i1, 0.025)
    assert len(ev) == 2 and set(ev["p"]) == {1} and set(ev["x"]) == {0}   # N = floor(2.5) = 2, ON
    assert np.allclose(ev["t"], [0.025 * 1 / 2.5, 0.025 * 2 / 2.5])       # Eq. 10
    assert np.isclose(cam.L_ref[0, 0], math.log(0.2 + eps) + 2 * cfg.contrast)   # Eq. 11: residual kept
    big = np.array([[0.99, 0.2]])                                          # cap N_max (Eq. 9)
    cam2 = EventCamera(cfg, 2, 1); cam2.step(np.array([[0.001, 0.2]]), 0); ev2 = cam2.step(big, 0.025)
    assert len(ev2) == cfg.n_max


def test_window_keeps_exactly_two_frame_intervals():
    w = EventWindow(0.050)
    def ev(t):
        a = np.zeros(1, EVENT_DTYPE); a["t"] = t; return a
    w.push(ev(0.010), 0.025); w.push(ev(0.040), 0.050); w.push(ev(0.070), 0.075)
    assert sorted(w.buf["t"]) == [0.040, 0.070]                            # 0.010 is older than t - 50 ms


# ---------------------------------------------------------------- perception (Eqs. 13-21)
def _disc(r, cu=79.5, cv=59.5, ring=False):
    n = np.zeros((120, 160), np.int32)
    vv, uu = np.mgrid[0:120, 0:160]
    rr = np.hypot(uu - cu, vv - cv)
    n[(rr <= r) & ((rr >= r - 1.2) if ring else True)] = 1
    return n


def test_detect_disc_and_hollow_ring():
    d = detect(_disc(4), P.camera, P.perception).selected
    assert d is not None and 7 <= d.d <= 9 and d.passes
    ring = detect(_disc(5, ring=True), P.camera, P.perception).selected
    assert ring is not None and ring.d == ring.d_bbox > ring.d_eq          # Eq. 20 recovers a hollow ring


def test_detect_rejects_elongated_and_small():
    n = np.zeros((120, 160), np.int32); n[59, 70:90] = 1                   # 20x1 streak: aspect >> 2
    assert detect(n, P.camera, P.perception).selected is None
    n = np.zeros((120, 160), np.int32); n[59, 79:81] = 1; n[60, 79:81] = 1  # 4 px, d < d_min
    assert detect(n, P.camera, P.perception).selected is None


def test_roi_is_32_px_square():
    u0, u1, v0, v1 = roi_bounds(P.camera, 16)
    assert (u1 - u0 + 1, v1 - v0 + 1) == (32, 32)


# ---------------------------------------------------------------- looming, tau, gate (Eqs. 22-31)
def test_looming_rules():
    tc = P.tracking
    assert looming_ok([5, 6, 6.5, 7], tc)
    assert not looming_ok([5, 5.5, 6, 6.4], tc)            # growth 1.4 < delta 1.5
    assert not looming_ok([5, 7, 6, 8], tc)                # reversal 1 px > eps_r
    assert not looming_ok([5, 5, 5, 16], tc)               # jump 11 px >= eps_s


def test_tau_and_closing_speed():
    ts = [0.0, 0.025, 0.05, 0.075]
    ds = [6.0, 6.5, 7.0, 7.5]
    assert np.isclose(fit_rate(ts, ds), 20.0)
    tr = LoomingTracker(P.camera, P.tracking, P.gate)
    from urgency_dodge.perception import Component
    for t, d in zip(ts, ds):
        c = Component(np.zeros((5, 2)), 5, (79.5, 59.5), (0, 0, 0, 0), d, d, d, 1.0, 0.0, True)
        g = tr.step(c, t)
    assert np.isclose(g.tau, 7.5 / 20.0)                                   # Eq. 28
    assert np.isclose(g.closing, 120 * 2 * 0.15 * 20 / 7.5 ** 2)          # Eq. 29b
    assert g.passed and tr.H == 1


def test_commit_needs_three_consecutive_passes():
    tr = LoomingTracker(P.camera, P.tracking, P.gate)
    from urgency_dodge.perception import Component
    seq = [6, 6.5, 7, 7.5, 8, 8.5, 9]
    for k, d in enumerate(seq):
        c = Component(np.zeros((5, 2)), 5, (79.5, 59.5), (0, 0, 0, 0), d, d, d, 1.0, 0.0, True)
        tr.step(c, k * 0.025)
        if k == 5:
            assert tr.committed and tr.commit_info["t"] == pytest.approx(0.125)   # passes at k = 3, 4, 5


# ---------------------------------------------------------------- control (Eqs. 32-35)
def test_sign_rule_as_printed():
    c = Controller(P)
    assert c.sign_from_image(78.8) == +1 and c.sign_from_image(80.5) == -1


def test_sidestep_is_constant_speed_and_return_is_exact():
    c = Controller(P)
    pos = np.array([5.0, 0.0, 1.5]); dt = P.camera.dt
    vys = []
    for k in range(40):
        cmd = c.step(3.8 + k * dt, pos, True, 79.0, 0.3 if k == 0 else math.inf, ball_x=20.0 if k < 20 else 0.0)
        vys.append(cmd.v[1]); pos = pos + cmd.v * dt
    first = [v for v in vys[:10] if v > 0]
    assert np.allclose(first, 0.85 / 0.3)                        # Δy / tau, counted down => constant
    assert abs(pos[1]) < 1e-12                                   # return lands on the centreline


def test_segment_min_distance():
    assert np.isclose(segment_min_distance([0, 1, 0], [1, 0, 0], [0.5, 0, 0], [-1, 0, 0], 1.0), 1.0)
    assert np.isclose(segment_min_distance([0, 0, 0], [0, 0, 0], [3, 4, 0], [0, 0, 0], 1.0), 5.0)


# ---------------------------------------------------------------- closed loop, ray-traced corridor
@pytest.fixture(scope="module")
def rt():
    from sims.raytrace import RayTraceRenderer
    return lambda p: run_episode(RayTraceRenderer(p), p).summary


def test_nominal_dodges_and_reaches_goal(rt):
    s = rt(P)
    assert s["dodged"] and not s["collision"]
    assert 3.70 <= s["commit_t"] <= 3.85                          # paper: 3.75 (ray-traced), 3.72 (Gazebo)
    assert s["sep_min"] > 0.5 and s["goal_t"] == pytest.approx(15.75)


def test_empty_corridor_never_dodges(rt):
    s = rt(P.with_(encounter={"ball_present": False}))
    assert not s["dodged"] and s["goal_t"] == pytest.approx(15.75)


def test_avoidance_off_is_hit(rt):
    s = rt(P.with_(control={"enabled": False}))
    assert s["collision"] and not s["dodged"]
