"""Redraw manuscript Fig. 7 from the digitized original + the fitted encounter model (see README.md)."""
import json, csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
D='/home/ibrahim/Documents/PHD/ecsa_dodge_repro/fig7/'
P=json.load(open(D+'fit_params.json'))
T_THROW, T_CMD, V_BALL = 3.15, 3.72, 10.0           # manuscript caption / text
VX = 1.54                                           # UAV forward speed, from panel (a)/(b) flank spacing
X_ANCHOR, T_ANCHOR = 5.559, 3.849                   # left flank at mid-height: x in (a), t in (b)
X_START, X_END, X_GOAL = 0.92, 23.47, 24.0
tstar=P['tstar']; plat=P['plateau']
def yu(t): return np.interp(t,[0,P['t_rise0'],P['t_rise1'],P['t_fall0'],P['t_fall1'],99],[0,0,plat,plat,0,0])
def xu(t): return np.clip(X_ANCHOR+VX*(t-T_ANCHOR),X_START,X_END)
XB_STAR=xu(tstar); XB0=XB_STAR+V_BALL*(tstar-T_THROW); T_BALL_END=tstar+(XB_STAR-0.1)/V_BALL
def xb(t): return np.where(t<T_THROW,XB0,XB_STAR-V_BALL*(t-tstar))
def sep(t): return np.hypot(xb(t)-xu(t),yu(t))
t=np.arange(3.0,T_BALL_END,0.002)
with open(D+'fig7_reconstructed_series.csv','w',newline='') as f:
    w=csv.writer(f); w.writerow(['t_s','uav_x_m','uav_y_m','ball_x_m','ball_y_m','separation_m'])
    for ti in np.arange(3.0,T_BALL_END,0.01): w.writerow([round(ti,3),round(float(xu(ti)),4),round(float(yu(ti)),4),round(float(xb(ti)),4),0.0,round(float(sep(ti)),4)])
smin=sep(t).min(); tmin=t[sep(t).argmin()]
print('min separation %.3f m at t=%.3f s; ball launched from x=%.2f m; UAV x at command %.2f m'%(smin,tmin,XB0,xu(T_CMD)))

plt.rcParams.update({'font.family':'DejaVu Serif','font.size':8.5,'axes.titlesize':9,'legend.fontsize':7.5,'axes.linewidth':0.8})
UAV,BALL,WALL='#1b7f3a','#c0392b','#555555'
EV=dict(throw=('#7f8c8d',':'),cmd=('#1b7f3a','--'),ca=('#2c3e50','-.'))
fig=plt.figure(figsize=(7.2,6.2),dpi=300)
gs=fig.add_gridspec(3,2,height_ratios=[0.85,1,1],width_ratios=[1,1.1],hspace=0.62,wspace=0.30)
# (a) full corridor
a=fig.add_subplot(gs[0,:])
for yw in (2,-2): a.axhline(yw,color=WALL,lw=1.6,label='corridor wall' if yw>0 else None)
tt=np.arange(0.84,16,0.005)
a.plot(xu(tt),yu(tt),color=UAV,lw=1.6,label='UAV path (1.5 m/s)',zorder=3)
a.plot([XB0,0.1],[0,0],ls=(0,(4,2)),color=BALL,lw=1.2,label='ball path (10 m/s)',zorder=4)
a.plot(XB0,0,'o',ms=4,color=BALL,zorder=5); a.annotate('ball launch',(XB0,0),(XB0,-1.15),ha='center',fontsize=7,color=BALL,arrowprops=dict(arrowstyle='-',color=BALL,lw=0.6))
a.plot(X_START,0,marker='s',ms=4,color='k',ls='none',label='start',zorder=5)
a.plot(X_GOAL,0,marker='*',ms=11,color='#2f80ed',ls='none',label='goal',zorder=5)
a.add_patch(plt.Rectangle((3.5,-1.9),5.0,3.8,fill=False,ec='k',lw=0.6,ls='--')); a.text(3.6,1.35,'(b)',fontsize=7)
a.set_xlim(0,26.5); a.set_ylim(-2.4,2.4); a.set_xlabel('x (m)'); a.set_ylabel('y (m)')
a.set_title('(a) Plan view of the corridor',loc='left')
a.legend(loc='lower left',bbox_to_anchor=(0,1.13),ncol=5,frameon=False,handlelength=2.0,columnspacing=1.2,borderaxespad=0)
# (b) zoomed plan view, to scale
b=fig.add_subplot(gs[1:,0])
for yw in (2,-2): b.axhline(yw,color=WALL,lw=1.6)
tz=np.arange(3.0,4.7,0.002)
b.plot(xu(tz),yu(tz),color=UAV,lw=1.4,zorder=3)
b.plot([8.5,3.5],[0,0],ls=(0,(4,2)),color=BALL,lw=1.0,zorder=4)
b.add_patch(Circle((xu(tstar),yu(tstar)),0.25,fc=UAV,ec='k',lw=0.6,alpha=0.9,zorder=5))
b.add_patch(Circle((float(xb(tstar)),0),0.15,fc=BALL,ec='k',lw=0.6,zorder=6))
b.plot([xu(tstar)]*2,[0,yu(tstar)],color='k',lw=0.7,zorder=7)
b.text(xu(tstar)+0.33,plat*0.45,'%.2f m'%smin,fontsize=7.5,va='center')
b.plot(xu(T_CMD),0,marker='v',ms=6,color=UAV,mec='k',mew=0.5,zorder=6); b.annotate('dodge\ncommanded\n%.2f s'%T_CMD,(xu(T_CMD),0),(4.25,-1.05),ha='center',fontsize=6.8,color=UAV,arrowprops=dict(arrowstyle='-',color=UAV,lw=0.6))
b.annotate('closest approach %.2f s'%tstar,(xu(tstar),yu(tstar)+0.27),(6.6,1.55),ha='center',fontsize=6.8,arrowprops=dict(arrowstyle='-',lw=0.6))
b.annotate('',(7.3,-0.3),(8.3,-0.3),arrowprops=dict(arrowstyle='->',color=BALL,lw=0.9)); b.text(7.8,-0.55,'ball',ha='center',fontsize=6.8,color=BALL)
b.annotate('',(5.2,0.3),(4.4,0.3),arrowprops=dict(arrowstyle='->',color=UAV,lw=0.9)); b.text(4.8,0.45,'UAV',ha='center',fontsize=6.8,color=UAV)
b.set_xlim(3.5,8.5); b.set_ylim(-2.2,2.2); b.set_aspect('equal')
b.set_xlabel('x (m)'); b.set_ylabel('y (m)'); b.set_title('(b) Encounter, to scale',loc='left')
b.legend(handles=[Line2D([],[],marker='o',ls='none',mfc=UAV,mec='k',ms=8,label='UAV, r = 0.25 m'),Line2D([],[],marker='o',ls='none',mfc=BALL,mec='k',ms=5,label='ball, R = 0.15 m')],loc='upper center',bbox_to_anchor=(0.45,-0.2),ncol=2,columnspacing=0.8,frameon=False)
# (c) lateral position, (d) separation
c=fig.add_subplot(gs[1,1]); d=fig.add_subplot(gs[2,1],sharex=c)
c.plot(tz,yu(tz),color=UAV,lw=1.6,label='UAV y',zorder=3)
c.plot(tz[tz>=T_THROW],0*tz[tz>=T_THROW],ls=(0,(4,2)),color=BALL,lw=1.2,label='ball y',zorder=4)
c.set_ylabel('y (m)'); c.set_ylim(-0.15,1.15); c.set_title('(c) Lateral position',loc='left')
c.legend(loc='upper left',frameon=False)
d.plot(t,sep(t),color='#4d5656',lw=1.4)
d.plot(tmin,smin,'o',color='k',ms=3.5,zorder=5); d.annotate('min %.2f m\nat %.2f s'%(smin,tmin),(tmin,smin),(4.28,0.9),fontsize=7,va='center',arrowprops=dict(arrowstyle='-',lw=0.6))
d.set_ylabel('separation (m)'); d.set_xlabel('t (s)'); d.set_ylim(0,12); d.set_title('(d) Separation',loc='left')
for ax in (c,d):
    ax.axvline(T_THROW,color=EV['throw'][0],ls=EV['throw'][1],lw=1.0)
    ax.axvline(T_CMD,color=EV['cmd'][0],ls=EV['cmd'][1],lw=1.0)
    ax.axvline(tstar,color=EV['ca'][0],ls=EV['ca'][1],lw=0.9)
    ax.grid(alpha=0.25,lw=0.5); ax.set_xlim(3.0,4.7)
plt.setp(c.get_xticklabels(),visible=False)
ev=[Line2D([],[],color=EV['throw'][0],ls=EV['throw'][1],label='throw, %.2f s'%T_THROW),Line2D([],[],color=EV['cmd'][0],ls=EV['cmd'][1],label='threat confirmed, dodge commanded, %.2f s'%T_CMD),Line2D([],[],color=EV['ca'][0],ls=EV['ca'][1],label='closest approach, %.2f s'%tstar)]
d.legend(handles=ev,loc='upper center',bbox_to_anchor=(0.45,-0.3),ncol=1,frameon=False,fontsize=7)
fig.savefig(D+'fig7_revised.png',dpi=300,bbox_inches='tight'); fig.savefig(D+'fig7_revised.pdf',bbox_inches='tight')
