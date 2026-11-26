"""Manuscript Fig. 7 (fig1_trajectory.png): original two-panel layout with minimal cosmetic fixes.

The original plotting script and run log were lost; the series are the digitized/fitted ones from
fit2.py (UAV) and the 10 m/s constant-velocity ball model validated there (see README.md).
"""
import json, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.transforms as mtrans
from matplotlib.lines import Line2D
D = '/home/ibrahim/Documents/PHD/ecsa_dodge_repro/fig7/'
P = json.load(open(D + 'fit_params.json'))

# ---- data (unchanged from the reconstruction) ----
t_throw, t_dodge, v_ball = 3.15, 3.72, 10.0
vx, x_anchor, t_anchor = 1.54, 5.559, 3.849
x_start, x_end, goal = 0.92, 23.47, (24.0, 0.0)
t_end = 15.8
t = np.arange(0.0, t_end, 0.01)
def uav_y(tt): return np.interp(tt, [0, P['t_rise0'], P['t_rise1'], P['t_fall0'], P['t_fall1'], 99], [0, 0, P['plateau'], P['plateau'], 0, 0])
def uav_x(tt): return np.clip(x_anchor + vx * (tt - t_anchor), x_start, x_end)
x_ca = uav_x(P['tstar'])
ball_x0 = x_ca + v_ball * (P['tstar'] - t_throw)
t_ball_end = P['tstar'] + (x_ca - 0.1) / v_ball
tb = np.arange(t_throw, t_ball_end, 0.01)                 # ball in flight
ball_x = x_ca - v_ball * (tb - P['tstar']); ball_y = np.zeros_like(tb)
sep = np.hypot(ball_x - uav_x(tb), uav_y(tb) - ball_y)

# ---- original style ----
plt.rcParams.update({'font.family': 'DejaVu Serif', 'font.size': 8})
UAV, BALL, SEP, GOAL, GREY = '#1b7f3a', '#c0392b', '#7f8c8d', '#2e86de', '#808080'
fig, (a, b) = plt.subplots(1, 2, figsize=(7.17, 2.57), dpi=300)

# (a) plan view
wall = None
for yw in (2.0, -2.0):
    wall, = a.plot([0, 26], [yw, yw], color=GREY, lw=1.2)
wall.set_label('corridor wall')                           # one legend entry for both walls
a.plot(uav_x(t), uav_y(t), color=UAV, lw=1.6, label='UAV', zorder=2)
a.plot(np.r_[ball_x0, ball_x], np.r_[0, ball_y], color=BALL, ls='--', lw=1.0, marker='o', ms=3,
       markevery=10, label='ball', zorder=3)
xd = uav_x(t_dodge)
a.plot(xd, 0, 'o', color=UAV, mec='k', mew=0.6, ms=5, zorder=4)
a.annotate('dodge', (xd, 0), xytext=(1.6, 1.2), color=UAV, fontsize=7.5,
           arrowprops=dict(arrowstyle='-', color=UAV, lw=0.7))
a.plot(*goal, marker='*', ms=10, color=GOAL, ls='none', label='goal', zorder=4)
a.set_xlim(0, 26); a.set_ylim(-2.4, 2.4); a.set_aspect('equal')
a.set_xlabel('x (m)'); a.set_ylabel('y (m)'); a.grid(alpha=0.3)
a.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4, frameon=False, fontsize=7)
a.text(0, 1.0, '(a)', transform=a.transAxes, ha='left', va='bottom', fontsize=8.5)

# (b) time histories, separation on a secondary axis
b.plot(tb, ball_y, color=BALL, ls='--', lw=1.0, marker='o', ms=3, markevery=30, label='ball y', zorder=3)
b.plot(t, uav_y(t), color=UAV, lw=2.0, label='UAV y', zorder=2)
b.set_ylim(-0.2, 3.2); b.set_xlim(0, 16)
b.set_xlabel('t (s)'); b.set_ylabel('y (m)'); b.grid(alpha=0.3)
b2 = b.twinx()
k = np.ceil(sep.max() / 3.2 * 1.05)                        # integer scale; keeps zero aligned with the left axis
b2.plot(tb, sep, color=SEP, lw=1.2, label='separation', zorder=2)
b2.set_ylim(-0.2 * k, 3.2 * k); b2.set_ylabel('separation (m)')
vl1 = b.axvline(t_throw, color=GREY, ls=':', lw=1.0, label=f'throw (t = {t_throw:.2f} s)')
vl2 = b.axvline(t_dodge, color=UAV, ls=':', lw=1.0, label=f'dodge commanded (t = {t_dodge:.2f} s)')
top = mtrans.blended_transform_factory(b.transData, b.transAxes)
b.text(t_throw - 0.15, 0.97, 'throw', transform=top, ha='right', va='top', color='#595959', fontsize=7.5)
b.text(t_dodge + 0.15, 0.97, 'dodge', transform=top, ha='left', va='top', color=UAV, fontsize=7.5)
h1, l1 = b.get_legend_handles_labels(); h2, l2 = b2.get_legend_handles_labels()
order = [0, 1]                                             # ball y, UAV y, separation, then event lines
H = [h1[i] for i in order] + h2 + h1[2:]; L = [l1[i] for i in order] + l2 + l1[2:]
b.legend(H, L, loc='center right', fontsize=6.5, frameon=True, framealpha=0.95)
b.text(0, 1.0, '(b)', transform=b.transAxes, ha='left', va='bottom', fontsize=8.5)

fig.tight_layout()
fig.savefig(D + 'fig1_trajectory.png', dpi=300)
print('sep max %.2f m, min %.3f m; right-axis scale k=%d; t_throw=%.2f t_dodge=%.2f' % (sep.max(), sep.min(), k, t_throw, t_dodge))
