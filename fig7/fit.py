import numpy as np, json
from PIL import Image
D='/home/ibrahim/Documents/PHD/ecsa_dodge_repro/fig7/'
a=np.asarray(Image.open(D+'fig7_original.png').convert('RGB')).astype(int)
near=lambda rgb,tol:(np.abs(a-np.array(rgb)).sum(2)<tol)
G=near((27,127,58),60); S=near((127,140,141),25)
bt=lambda px:(px-1201)/((2103-1201)/16); by=lambda py:(601-py)/((601-96)/3.0); byi=lambda y:601-y*(601-96)/3.0
ax=lambda px:(px-151)/((1019-151)/25); ay=lambda py:(349-py)/((418-280)/4); ayi=lambda y:349-y*(418-280)/4
def edges(M,row,c0,c1):
    xs=np.where(M[row,c0:c1])[0]+c0
    if len(xs)==0: return None
    g=np.split(xs,np.where(np.diff(xs)>3)[0]+1); return [ (s.min()+s.max())/2 for s in g]
# (b) UAV bump per row (exclude dotted line col ~1410 by starting at 1415)
bb=[]
for y in np.arange(0.06,0.86,0.04):
    e=edges(G,int(round(byi(y))),1415,1600)
    if e and len(e)>=2: bb.append((y,bt(e[0]),bt(e[-1])))
plat=[bt(x) for x in range(1415,1600) if G[int(byi(0.9)),x]]
print('(b) bump rows y,t_left,t_right:'); [print(' %.2f %.3f %.3f'%r) for r in bb]
print('(b) plateau t: %.3f-%.3f'%(min(plat),max(plat)))
ab=[]
for y in np.arange(0.06,0.86,0.04):
    e=edges(G,int(round(ayi(y))),300,420)
    if e: ab.append((y,[round(ax(v),3) for v in e]))
print('(a) bump rows y -> x centres:'); [print(' %.2f'%y,v) for y,v in ab]
platx=[ax(x) for x in range(300,420) if G[int(round(ayi(0.9))),x]]
print('(a) plateau x: %.3f-%.3f'%(min(platx),max(platx)))
# path endpoints in (a)
cols=[x for x in range(152,1052) if G[int(round(ayi(0))),x]]
print('(a) y=0 green x range: %.2f-%.2f'%(ax(min(cols)),ax(max(cols))))
# separation branches per row -> |dx|
sep=[]
for py in range(66,560):
    e=edges(S,py,1404,1700)
    if e and len(e)==2: sep.append((by(py),bt(e[0]),bt(e[1])))
sep=np.array(sep); np.save(D+'sep_rows.npy',sep)
for lo,hi in [(1.3,3.1)]:
    m=(sep[:,0]>lo)&(sep[:,0]<hi)
    pl=np.polyfit(sep[m,1],sep[m,0],1); pr=np.polyfit(sep[m,2],sep[m,0],1)
    print('sep slope left %.2f m/s, right %.2f m/s (raw sep, %s-%s m)'%(pl[0],pr[0],lo,hi))
print('sep minimum visible: %.3f m'%sep[:,0].min())
