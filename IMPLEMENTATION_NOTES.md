# Implementation notes

Which values come from the paper, which were chosen because the paper leaves them open, and where the results
of this code differ from the values printed in the paper. Equation numbers follow the full-length method
description (Eqs. 1-41).

## Stated in the paper

* **Camera:** 160 × 120, f = 120 px (θ_h = 1.176 rad), principal point ((W−1)/2, (H−1)/2), 40 Hz.
* **Event model:** C = 0.20, N_max = 8, linear time interpolation, the reference advances by the crossings used,
  and a 50 ms window.
* **Detection:**
  * r = 16 crop, 3 × 3 dilation, 8-connected components with n_min = 4.
  * The nearest component is chosen, with ties broken by event count.
  * d = max(d_bbox, d_eq).
  * Screen: d ∈ [3, 18] px, aspect ∈ [1/2, 2], centroid within r.
* **Looming and tracking:**
  * Looming: m = 4, δ = 1.5 px, ε_r = 0.75, ε_s = 10.
  * Tracking: 18 px association gate; history cleared after 4 misses, track deleted after more than 5.
  * Hunting starts after the scheduled throw.
* **Time-to-contact and gate:**
  * Least-squares ḋ on 4 samples, τ = ∞ when ḋ < 2 px/s, τ = d/ḋ.
  * Range and closing speed use R = 0.15 m.
  * Gate: d ≥ 6, 0.12 < τ < 2.0, closing speed 3-30 m/s. Commit after H* = 3 consecutive passes.
* **Controller (Eq. 35):**
  * Takeoff, sidestep, hold, return, cruise.
  * y_c = 0.85 m, v_y = s·clip(Δy/τ, 0.8, 5.0), v_ret = 2.0.
  * Forward speed is unchanged during the dodge.
  * Sign rule of Eq. 34 (s = −1 if c_u ≥ u0 + 1, otherwise +1).
* **Plant and encounter:**
  * Both plants integrate the commanded velocity at Δt = 25 ms, with heading fixed.
  * Pad at (1, 0, 0.28), climb at 1.5 m/s. Ball spawned at (16, 0, 1.5) and aimed at x = 6.
  * Launch time from Eq. 37 (3.15 s nominal). Gravity on the ball is neglected.
* **Scoring:** centre-to-centre separation, collision below 0.40 m, τ_true = Z/(v_cruise − v_b,x).

## Implementation choices (not stated in the paper)

| Item | Choice | Reason |
|---|---|---|
| τ used by Eq. 33 after commit | τ̂ at commit, counted down in time (`Control.tau_mode`) | "Covers Δy in one time-to-contact": v_y is constant and arrives at the predicted contact time. Re-reading τ from the track after commit lets a wall blob that joins the track slow the sidestep. The alternative is kept as `track_latest` |
| "Ball behind" test in Eq. 35 | x_b < x − 0.6 m | Eq. 35 is written with the ball position x_b. This is the only place the controller reads it, and only to start the return. Perception never reads it |
| Goal tolerance | 0.6 m | Goal reached at 15.75 s, as reported |
| Corridor placement | x ∈ [−2, 26], ceiling 3.0 m | Only "28 m × 4 m" is stated |
| log ε in L = log(I + ε) | 1e-3 on [0, 1] intensities | Not stated |
| Frames | 8-bit grayscale from both renderers | As a real camera. Also makes ROS images bit-identical to the offline path |
| Ray-traced look | 1 m stripes, 1 m end-wall checker, shelf spheres r = 0.4 m every 2 m at y = ±1.55, 8 × 4 checker ball, albedo 0.9/0.05 | Matched to the published render. The end-wall checker has the published apparent size (≈ 21-23 px at commit) |
| Gazebo look | Brick walls (0.5 × 0.2 m bricks), shelving blocks 2.5 × 0.4 × 1.6 m, plank floor, dark battened ceiling with lamps and point lights, ball with 12 black spots | Matched to the published render |
| Gazebo stepping | Two 25 ms steps per tick, keeping the second frame | Gazebo Jetty publishes a step's camera frame from the scene before that step's pose update. `experiments/probe_gazebo.py` verifies that every frame equals a settled render of its pose |

## Differences from the values printed in the paper

* **Gazebo commit at 3.85 s against 3.72 s.** The spotted ball's diameter reads 6 → 9 → 7 px around 3.70 s. That
  2 px reversal breaks Eq. 23, so H restarts and commits three ticks later. The miss (0.87 against 0.90 m) and the
  goal time (15.75 s) agree.
* **Speed-sweep miss distances** are 0.57-0.93 m against 0.55-0.77 m. With the counted-down τ the sidestep
  completes whenever τ̂ ≤ τ, so the miss sits near y_c = 0.85 m.
* **Apparent ball diameter at commit** is 7 px against 8-9 px in the ray-traced corridor, and 10 px against 7 px
  in Gazebo.
