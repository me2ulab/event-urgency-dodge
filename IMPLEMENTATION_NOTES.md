# Recovery notes: what is faithful and what is reconstructed

The original ECSA code and run logs were lost with the laptop. The sources used here are:

1. `../PHD/ecsa_paper.docx`, the full-length draft: Eqs. 1-41 with parameter values, the encounter design,
   Tables 1-3, and the experiment names.
2. `../manuscript.v4.docx`, the proceedings version: the same method, condensed.
3. The surviving renders `../PHD/fig5_gz_image_events.png`, `../PHD/fig6_kin_image_events.png` and
   `../PHD/scene_*_t*.png`. These were used for the look of both worlds and for the Fig. 6 check.

## Faithful (stated in the paper)

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
  * The sign rule follows Eq. 34 as printed (s = −1 if c_u ≥ u0 + 1, otherwise +1).
* **Plant and encounter:**
  * Both plants integrate the commanded velocity at Δt = 25 ms, with heading fixed.
  * Pad at (1, 0, 0.28), climb at 1.5 m/s. Ball spawned at (16, 0, 1.5) and aimed at x = 6.
  * t_launch from Eq. 37 (3.15 s nominal). Gravity on the ball is neglected.
* **Scoring:** centre-to-centre separation, collision below 0.40 m, τ_true = Z/(v_cruise − v_b,x).

## Reconstructed (not stated; chosen and justified here)

| Item | Choice | Why |
|---|---|---|
| τ used by Eq. 33 after commit | τ̂ at commit, counted down in time (`Control.tau_mode`) | Matches "covers Δy in one time-to-contact": v_y is constant and arrives at the predicted contact time. Re-reading τ from the track after commit let a wall blob join the confirmed track and slow the sidestep (seen in the pilot). The other mode is kept as `track_latest` |
| "Ball behind" test in Eq. 35 | x_b < x − 0.6 m, using the ball pose | Eq. 35 is printed with x_b. This conflicts with "pose is never read by control", so it is used ONLY for the return trigger and is listed here. ε_x = 0.6 m fits the return onset in the published Fig. 7 |
| Goal tolerance | 0.6 m | Reproduces the published goal time of 15.75 s exactly |
| Corridor placement | x ∈ [−2, 26], ceiling 3.0 m | Only "28 m × 4 m" is stated |
| log ε in L = log(I + ε) | 1e-3 on [0, 1] intensities | Not stated |
| Frames | 8-bit grayscale from both renderers | As a real camera; also makes ROS images bit-identical to the offline path |
| Ray-traced look | 1 m stripes, 1 m end-wall checker, shelf spheres r = 0.4 m every 2 m at y = ±1.55, 8 × 4 checker ball, albedo 0.9/0.05 | Read off `fig6_kin_image_events.png`. The end-wall checker reproduces the published apparent size (≈ 21-23 px at commit) |
| Gazebo look | Brick walls (0.5 × 0.2 m bricks), shelving blocks 2.5 × 0.4 × 1.6 m, plank floor, dark battened ceiling with lamps and point lights, ball with 12 black spots | Read off `fig5_gz_image_events.png`. The original SDF and textures are lost |
| Gazebo stepping | Two 25 ms steps per tick, keeping the second frame | Measured: Jetty publishes a step's frame from the scene before that step's pose update (`experiments/probe_gazebo.py`: 12/12 frames equal their settled reference) |

## Differences from the paper that remain (not tuned away)

* **Gazebo commit at 3.85 s vs 3.72 s.** The spotted ball's diameter reads 6 → 9 → 7 px around 3.70 s. That 2 px
  reversal breaks Eq. 23, so H restarts and commits three ticks later. It depends on the reconstructed texture and
  lighting, which cannot be recovered. The miss (0.87 vs 0.90 m) and goal time (15.75 s) still match.
* **Miss distances in the ray-traced sweep** are 0.57-0.93 m against the paper's 0.55-0.77 m. With the countdown
  τ the sidestep completes whenever τ̂ ≤ τ, so the miss sits near y_c. The paper's values below y_c suggest the
  original sidestep was slower. Its exact form is not recoverable.
* **Apparent ball diameter at commit** is 7 px against the paper's 8-9 px in the ray-traced world, and 10 px
  against 7 px in Gazebo.

## Manuscript issues found while rebuilding (also in the reports to the author)

* **Fig. 6 green box (manuscript v4):** in the surviving 3.60 s frame it marks a ceiling-edge cluster, not the
  ball (`GREEN_BOX_CHECK.md`, `tests/test_core.py::test_figure6_green_box_case`). The same cluster appears in the
  rebuilt Gazebo world at 3.43-3.50 s.
* **Inconsistent run numbers:** Fig. 7 / Table 1 / the abstract / the Conclusions disagree on cruise speed
  (1.5 vs 3 m/s) and on the miss distance (0.77 vs 0.90 m). In ecsa_paper.docx, 0.90 m is the Gazebo run and
  0.77 m the ray-traced one.
