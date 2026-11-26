"""Digitize manuscript.v4 Fig. 7 (= PHD/fig1_trajectory.png). Original run data/code are lost."""
import numpy as np, csv
from PIL import Image
D='/home/ibrahim/Documents/PHD/ecsa_dodge_repro/fig7/'
a=np.asarray(Image.open(D+'fig7_original.png').convert('RGB')).astype(int)
def near(rgb,tol=40): return (np.abs(a-np.array(rgb)).sum(2)<tol)
# calibrations from tick marks
bt=lambda px:(px-1201)/((2103-1201)/16); by=lambda py:(601-py)/((601-96)/3.0)
ax=lambda px:(px-151)/((1019-151)/25); ay=lambda py:(349-py)/((418-280)/4)
G=near((27,127,58),60); S=near((127,140,141),25); R=near((192,57,43),60); K=near((89,89,89),15)
def w(name,rows,hdr):
    with open(D+name,'w',newline='') as f: c=csv.writer(f); c.writerow(hdr); c.writerows(rows)
# (b) UAV y(t): per column, centre of green extent (frame interior only, skip legend box region)
rows=[]
for x in range(1204,2100):
    ys=np.where(G[66:632,x])[0]+66
    ys=ys[~((x>1850)&(ys<200))]               # legend swatch
    if len(ys): rows.append((round(bt(x),4),round(by((ys.min()+ys.max())/2),4),round(by(ys.max()),4),round(by(ys.min()),4)))
w('b_uav_y.csv',rows,['t','y_mid','y_lo','y_hi'])
# (b) separation: per row, left/right branch t
rows=[]
for y in range(66,600):
    xs=np.where(S[y,1204:1850])[0]+1204
    if len(xs)==0: continue
    gaps=np.where(np.diff(xs)>4)[0]
    if len(gaps)==0: rows.append((round(by(y),4),round(bt(xs.mean()),4),''))
    else: rows.append((round(by(y),4),round(bt(xs[:gaps[0]+1].mean()),4),round(bt(xs[gaps[0]+1:].mean()),4)))
w('b_separation.csv',rows,['sep','t_left','t_right'])
# (b) dotted vertical lines + ball red pixels
cols=[x for x in range(1204,2100) if K[66:632,x].sum()>30]
print('dotted line columns -> t:',[(x,round(bt(x),3)) for x in cols])
ry,rx=np.where(R[66:632,1204:2100]); print('ball red pixels (b): t range',round(bt(rx.min()+1204),3),round(bt(rx.max()+1204),3),'y',round(by(ry.min()+66),3),round(by(ry.max()+66),3))
# (a) UAV path y(x)
rows=[]
for x in range(154,1050):
    ys=np.where(G[270:428,x])[0]+270
    ys=ys[ys>300] if x>800 else ys            # avoid legend line row region check below
    if len(ys): rows.append((round(ax(x),4),round(ay((ys.min()+ys.max())/2),4)))
w('a_uav_path.csv',rows,['x','y'])
ry,rx=np.where(R[270:428,154:1050]); print('ball red pixels (a): x',sorted(set(np.round(ax(rx+154),1)))[:40])
