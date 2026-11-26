import numpy as np, json
from PIL import Image
from scipy.optimize import least_squares
D='/home/ibrahim/Documents/PHD/ecsa_dodge_repro/fig7/'
a=np.asarray(Image.open(D+'fig7_original.png').convert('RGB')).astype(int)
G=(np.abs(a-np.array((27,127,58))).sum(2)<60)
bt=lambda px:(px-1201)/((2103-1201)/16); by=lambda py:(601-py)/((601-96)/3.0); byi=lambda y:601-y*(601-96)/3.0; bti=lambda t:1201+t*(2103-1201)/16
# plateau height: centre of green run at columns inside plateau
hs=[]
for t in np.arange(3.97,4.16,0.01):
    ys=np.where(G[int(byi(1.2)):int(byi(0.6)),int(round(bti(t)))])[0]+int(byi(1.2))
    hs.append(by((ys.min()+ys.max())/2))
plat=float(np.median(hs)); print('plateau y = %.3f m (spread %.3f)'%(plat,np.ptp(hs)))
# clean UAV y(t) polyline from per-row edges
L,R=[],[]
for y in np.arange(0.04,plat-0.03,0.02):
    xs=np.where(G[int(round(byi(y))),1415:1600])[0]+1415
    g=np.split(xs,np.where(np.diff(xs)>3)[0]+1)
    if len(g)>=2: L.append((bt((g[0].min()+g[0].max())/2),y)); R.append((bt((g[-1].min()+g[-1].max())/2),y))
L,R=np.array(L),np.array(R)
tl0=np.polyfit(L[:,1],L[:,0],1); tr0=np.polyfit(R[:,1],R[:,0],1)   # t as linear fn of y on each flank
t_rise0,t_rise1=np.polyval(tl0,0),np.polyval(tl0,plat); t_fall0,t_fall1=np.polyval(tr0,plat),np.polyval(tr0,0)
print('rise %.3f->%.3f  plateau %.3f->%.3f  fall ->%.3f'%(t_rise0,t_rise1,t_rise1,t_fall0,t_fall1))
def yu(t): return np.interp(t,[0,t_rise0,t_rise1,t_fall0,t_fall1,99],[0,0,plat,plat,0,0])
sep=np.load(D+'sep_rows.npy')        # rows: sep, t_left, t_right
m=(sep[:,0]>1.25)&(sep[:,0]<3.1)
T=np.r_[sep[m,1],sep[m,2]]; Sv=np.r_[sep[m,0],sep[m,0]]
res=least_squares(lambda p: np.sqrt((p[0]*(T-p[1]))**2+yu(T)**2)-Sv,[11,4.1])
vrel,tstar=res.x; rms=np.sqrt(np.mean(res.fun**2))
print('fit: v_rel = %.2f m/s, t* = %.3f s, rms = %.3f m (n=%d)'%(vrel,tstar,rms,len(T)))
# UAV forward speed from both panels (mid-height edge spacing)
json.dump(dict(plateau=plat,t_rise0=t_rise0,t_rise1=t_rise1,t_fall0=t_fall0,t_fall1=t_fall1,vrel=vrel,tstar=tstar,rms=rms),open(D+'fit_params.json','w'),indent=1)
