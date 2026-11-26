# Figure 7 (trajectory): revised version

`fig7_revised.png` / `.pdf` replaces manuscript.v4 Fig. 7 (`fig7_original.png`, same file as `PHD/fig1_trajectory.png`).
Regenerate it with `make_fig7.py`, run inside the sim container (it needs numpy and matplotlib).

## Where the data comes from: read before submitting

The original run log and plotting code were lost with the laptop. This figure was rebuilt from the published image:

| Series | Source |
|---|---|
| UAV y(t) | Digitized from original panel (b), row by row along the bump flanks (`fit2.py`) |
| UAV x(t) | 1.54 m/s, from the spacing of the bump flanks in panel (a) (0.66 m) against panel (b) (0.43 s) |
| Ball x(t) | Model: straight line at 10 m/s from the 3.15 s throw (manuscript values). Stationary before the throw |
| Separation | Computed from the two tracks above, centre to centre |
| Event times | 3.15 s and 3.72 s from the caption; 4.12 s from the fit |

**Validation.** The model was fitted to the part of the original separation curve that was visible (below 3.2 m, n = 594 digitized points).
- It gives a closing speed of **11.53 m/s**, against 11.54 m/s expected (10 m/s ball + 1.54 m/s UAV).
- The rms residual is 0.11 m, about half a pixel on the steep flanks.
- The model puts the UAV at x = 5.36 m at 3.72 s, which is where the original panel (a) marker sits.

**What is not measured:** the separation above 3.2 m and the ball's launch point (x ≈ 15.7 m) come from the model, not from the run.
Before final submission, replace this with a figure drawn from a fresh simulator run.

`fig7_reconstructed_series.csv` holds the plotted series at 10 ms steps.

## Suggested caption

> **Figure 7.** Closed-loop avoidance of a 10 m/s ball at 1.5 m/s cruise. (a) Plan view of the 4 m-wide corridor. The UAV flies along y = 0 from the start pad to the goal at x = 24 m. The ball is launched along the centreline towards it. (b) The encounter region to scale, with the UAV and ball at closest approach. (c) Lateral position of the UAV and the ball. (d) Centre-to-centre UAV–ball separation. Vertical lines mark the throw (3.15 s, dotted), threat confirmation and the dodge command after three consecutive gated frames (3.72 s, dashed), and closest approach (4.12 s, dash-dot). The frame in Figures 4 and 6 (t = 3.60 s) is 0.12 s before confirmation. Minimum separation is 0.89 m between centres (≈ 0.49 m between surfaces).

## Inconsistencies found in the manuscript (not fixed here)

1. **This run is the 1.5 m/s cruise.** In Table 1 that row says confirmation at 3.75 s and miss 0.77 m. The Fig. 7 caption and the abstract say 3.72 s and 0.90 m.
2. **The Conclusions** say the ball was dodged "at a 3 m/s cruise". The figure shows 1.5 m/s.
3. **Minimum separation:** the original caption says 0.90 m; digitizing gives 0.885 ± 0.02 m. Use the logged value if you have one, and say it is centre-to-centre.
