# Reproduction report

Paper values: `PHD/ecsa_paper.docx` (Tables 1-3). Reproduced: this repository, one run per row (both simulators are deterministic). Miss = closest centre-to-centre distance; for hits the per-tick sampled minimum is shown, as in the paper.

## Table 1: nominal encounter (UAV 1.5 m/s, ball 10 m/s) and controls

Each cell is **paper → reproduced**.

| Experiment | Dodge (s) | d (px) | τ̂ / τ (s) | Miss (m) | Goal (s) |
|---|---|---|---|---|---|
| `closed_loop_ttc_check` | 3.75 → 3.775 | 8 → 7 | 0.33/0.38 → 0.29/0.37 | 0.77 → 0.87 | 15.75 → 15.75 |
| `closed_loop_noball_check` | none → none | — → — | — → — | no dodge → no dodge | — → 15.75 |
| `closed_loop_nododge` | — → — | — → — | — → — | 0.10 (hit) → 0.06 (hit) | — → — |
| `gz_closed_015` | 3.72 → 3.850 | 7 → 10 | 0.29/0.40 → 0.28/0.29 | 0.90 → 0.87 | 15.75 → 15.75 |
| `gz_closed_015_noball` | none → none | — → — | — → — | no dodge → no dodge | — → 15.75 |
| `gz_closed_nododge` | — → — | — → — | — → — | 0.10 (hit) → 0.06 (hit) | — → — |

## Table 2: speed sweep (ray-traced corridor)

| Run | Confirm (s) | d (px) | τ̂ / τ (s) | Miss (m) |
|---|---|---|---|---|
| `uav1.0_ball10` | 5.50 → 5.525 | 9 → 11 | 0.19/0.30 → 0.23/0.29 | 0.57 → 0.93 |
| `uav1.5_ball10` | 3.75 → 3.775 | 8 → 7 | 0.33/0.38 → 0.29/0.37 | 0.77 → 0.87 |
| `uav2.0_ball10` | 3.00 → 2.950 | 9 → 7 | 0.32/0.30 → 0.48/0.36 | 0.55 → 0.63 |
| `uav2.5_ball10` | 2.45 → 2.475 | 8 → 7 | 0.33/0.35 → 0.29/0.34 | 0.74 → 0.87 |
| `uav3.0_ball10` | 2.15 → 2.150 | 8 → 7 | 0.22/0.32 → 0.41/0.33 | 0.75 → 0.67 |
| `uav1.5_ball12` | 3.82 → 3.825 | 8 → 7 | 0.33/0.31 → 0.41/0.32 | 0.70 → 0.66 |

## Table 3: command-path time on gz_closed_015

Mean command-path time (detect + track + command) per tick: **0.46 ms** (of which detection 0.43 ms), 25 ms budget. Paper: 1.27 ms on the original laptop.

## Figures

`results/figures/`: fig4_gazebo_frames, fig5_raytrace_frames, fig6_isolation, fig7_trajectory.
