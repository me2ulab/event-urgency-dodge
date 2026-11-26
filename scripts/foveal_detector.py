"""Foveal ROI detector exactly as written in manuscript.v4 Sec. 3.3-3.4 (Eqs. 4-11).

Pure Python (no numpy) so it runs on the host. Events are (u, v, polarity) on a 160x120 sensor.
"""
from collections import Counter

W, H = 160, 120
U0, V0 = 80, 60          # principal point (fx = fy = 120, centred)
R = 16                   # Eq. 4 crop half-width
N_MIN = 4                # Eq. 6
D_MIN, D_MAX = 3, 18     # Eq. 11 detection limits (px)
AR_LO, AR_HI = 0.5, 2.0  # Eq. 11 aspect-ratio band "[1/2, 2]"


def crop(events):
    """Eq. 4: keep events with |u-u0|<=r and |v-v0|<=r; returns per-pixel counts n(u,v)."""
    return Counter((u, v) for u, v, _ in events if abs(u - U0) <= R and abs(v - V0) <= R)


def components(n):
    """Eqs. 5-6: binarise, dilate 3x3, 8-connected components; keep those with >= N_MIN fired pixels."""
    fired = set(n)
    dil = {(u + du, v + dv) for u, v in fired for du in (-1, 0, 1) for dv in (-1, 0, 1)
           if abs(u + du - U0) <= R and abs(v + dv - V0) <= R}
    seen, comps = set(), []
    for p in dil:
        if p in seen:
            continue
        stack, comp = [p], []
        seen.add(p)
        while stack:
            u, v = stack.pop()
            comp.append((u, v))
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    q = (u + du, v + dv)
                    if q in dil and q not in seen:
                        seen.add(q)
                        stack.append(q)
        px = [q for q in comp if q in fired]      # original fired pixels in this component
        if len(px) >= N_MIN:
            comps.append(px)
    return comps


def describe(px):
    us, vs = [p[0] for p in px], [p[1] for p in px]
    cu, cv = sum(us) / len(us), sum(vs) / len(vs)
    w, h = max(us) - min(us) + 1, max(vs) - min(vs) + 1
    d_bbox = max(max(us) - min(us), max(vs) - min(vs))          # Eq. 8
    d_eq = 2 * (len(px) / 3.141592653589793) ** 0.5             # Eq. 9
    d = max(d_bbox, d_eq)                                       # Eq. 10
    dist = ((cu - U0) ** 2 + (cv - V0) ** 2) ** 0.5
    passes = D_MIN <= d <= D_MAX and AR_LO <= w / h <= AR_HI and dist <= R   # Eq. 11
    return dict(n=len(px), centroid=(round(cu, 2), round(cv, 2)), bbox=(min(us), min(vs), max(us), max(vs)),
                d=round(d, 2), aspect=round(w / h, 2), dist=round(dist, 2), passes=passes)


def detect(events):
    """Eq. 7: of the retained components, the one closest to the principal point."""
    comps = [describe(c) for c in components(crop(events))]
    comps.sort(key=lambda c: c["dist"])
    return (comps[0] if comps else None), comps
