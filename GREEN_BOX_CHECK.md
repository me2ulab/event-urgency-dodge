# Figure 6 green box: verdict

**The green box should not be in Figure 6 as it stands.** It does not mark the ball.
It marks a small cluster of ceiling/door-lintel events at the top edge of the crop.

## Evidence

Source: `../PHD/scene_gz_events_t360.png`, the surviving raw 50 ms event frame at t = 3.60 s
(960x720, i.e. 6x the 160x120 sensor). Event pixels were decoded and passed through the detector
written exactly as manuscript.v4 Eqs. 4-11 (`scripts/foveal_detector.py`).
Overlay: `results/green_box_check_t360.png`.

| Component | Centroid (u,v) | bbox (px) | d | aspect | dist to (80,60) | Eq. 11 |
|---|---|---|---|---|---|---|
| C1: ball | (78.8, 58.2) | 7 x 3 | 6 | 2.33 | 2.2 px | **fails** (aspect > 2) |
| C2: ceiling edge | (78.4, 45.6) | 5 x 4 | 4 | 1.25 | 14.5 px | **passes** |

When the green box in Figure 6 is measured off the published image, it covers u ≈ 77-83, v ≈ 45-50, which is C2.

## What this means

- If the detector filters with Eq. 11 before choosing the closest component (Eq. 7), it picks C2.
  The green box is that output, so the overlay shows the detector locked onto a ceiling structure at t = 3.60 s.
- If it follows the manuscript's order (closest first, Eq. 7, then the Eq. 11 screen), it picks C1 (the ball),
  which then fails the screen. Nothing is detected at t = 3.60 s, so no box should be drawn.
- In both cases the green box contradicts the text ("the ball sits centred in the 16 px crop box"). The caption
  and legend also never explain it.

## Recommended fix

Remove the green box from the top-right panel. If a detection marker is wanted, draw it around C1 (the ball)
and add it to the caption. Also check which order the original code used for Eq. 7 and Eq. 11: the
manuscript gives one order, but the figure suggests the code used the other.

Caveat: the original detector code is lost. This check uses the manuscript's equations and the surviving
rendered frame, not the original arrays. The aspect-ratio band [1/2, 2] and d_max = 18 px are read from
Eq. 11 and the text after it.
