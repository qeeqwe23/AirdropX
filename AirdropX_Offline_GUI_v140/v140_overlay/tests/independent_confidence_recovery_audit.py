import math

def conf(w,r,sigma,sigma_floor=.08,snr_half=2.5,abs_half=.35,rate_half=.30,rate_sigma_ref=.60,p=4):
    sigma=max(sigma,sigma_floor); z=abs(w)/sigma
    snr=(z**p)/(z**p+snr_half**p) if z else 0.0
    a=abs(w); ac=(a**p)/(a**p+abs_half**p) if a else 0.0
    rc=(abs(r)**p)/(abs(r)**p+rate_half**p) if r else 0.0
    rc*=1/(1+(sigma/rate_sigma_ref)**2)
    return snr*ac,rc
cases=[
    ('calm-like',0.20,0.10,0.20),
    ('small-real-wind',1.0,0.10,0.20),
    ('5mps',5.0,0.10,0.30),
    ('12mps',12.0,0.10,0.30),
    ('ramp-cross-zero',0.05,1.0,0.25),
]
rows=[]
for name,w,r,s in cases:
    cw,cr=conf(w,r,s); rows.append((name,cw,cr,cw*w,cr*r))
assert rows[0][1] < 0.10, rows[0]
assert rows[2][1] > 0.99, rows[2]
assert rows[3][1] > 0.99, rows[3]
assert rows[4][2] > 0.80, rows[4]
levels=[0,.25,.5,.75,1]
qmax=[1.5,4.0,1.5,2.0,3.0,1.0,1.0]; rmax=[.80,.35]
for s in levels:
    q=[1+s*(v-1) for v in qmax]; rr=[1+s*(v-1) for v in rmax]
    assert min(q)>0 and min(rr)>0
print('v1.3.2 independent confidence/recovery audit')
for row in rows: print('%-18s windConf=%7.4f rateConf=%7.4f effectiveWind=%8.4f effectiveRate=%8.4f'%row)
print('recovery_levels=',levels)
print('max_q_multiplier=',qmax)
print('max_r_multiplier=',rmax)
print('PASS=1')
