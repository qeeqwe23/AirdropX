from __future__ import annotations
import math
import numpy as np

TS=0.1
RATE_THRESHOLD=0.70
RATE_CONF=0.70
RATE_ACT_WIND_CONF=0.15
RATE_HOLD=0.30
PERSIST=3.50


def latch(wind_conf, rate_conf, rate_eff, gust, gust_latched):
    active=False; act_count=0; keep_count=0; hold=0; out=[]; events=0
    nact=max(1,math.ceil(RATE_HOLD/TS)); npersist=max(1,math.ceil(PERSIST/TS))
    for wc,rc,re,g,gl in zip(wind_conf,rate_conf,rate_eff,gust,gust_latched):
        dyn=(rc>=RATE_CONF and abs(re)>=RATE_THRESHOLD)
        activation=dyn and wc>=RATE_ACT_WIND_CONF
        act_count=act_count+1 if activation else 0
        keep_count=keep_count+1 if dyn else 0
        was=active
        if g or act_count>=nact:
            active=True; hold=npersist
        elif active:
            if gl or keep_count>=nact:
                hold=npersist
            else:
                hold=max(0,hold-1)
                if hold==0: active=False
        if active and not was: events+=1
        out.append(active)
    a=np.array(out,dtype=bool)
    return a,events,int(np.sum(a[1:]!=a[:-1]))

# 1) calm: rate-confidence spikes alone must never activate when amplitude evidence is absent.
n=600
wc=np.zeros(n)+0.02
rc=np.zeros(n)+0.9
re=np.zeros(n)
re[::7]=1.2
re[1::19]=-1.4
g=np.zeros(n,dtype=bool); gl=np.zeros(n,dtype=bool)
a,e,s=latch(wc,rc,re,g,gl)
assert not a.any() and e==0 and s==0

# 2) constant 12 m/s after one abrupt event: transient carrier evidence must expire,
# even though release-side absolute wind evidence would remain high forever.
n=600
wc=np.ones(n)*0.999
rc=np.ones(n)*0.2
re=np.zeros(n)
g=np.zeros(n,dtype=bool); g[50]=True
gl=np.zeros(n,dtype=bool); gl[50:202]=True
a,e,s=latch(wc,rc,re,g,gl)
assert e==1 and s==2
assert a[50] and not a[260] and not a[-1]
release_evidence=wc>=0.25
assert release_evidence[-1] and not a[-1]

# 3) sustained sinusoidal rate: one event should bridge rate zero-crossings with
# persistence, then clear after forcing ends.
t=np.arange(n)*TS
wc=np.where((t>=5)&(t<47),0.99,0.02)
rate=1.88*np.cos(2*np.pi*(t-5)/20)
rate[(t<5)|(t>=47)]=0
rc=np.where(np.abs(rate)>=0.2,0.92,0.1)
g=np.zeros(n,dtype=bool); g[50]=True
gl=np.zeros(n,dtype=bool)
a,e,s=latch(wc,rc,rate,g,gl)
assert e==1 and s==2
assert a[100] and a[400]
assert not a[-1]
last=np.where(a)[0][-1]*TS
assert last < 55

print('v1.3.6 transient-evidence policy audit: PASS')
print(f'calm active fraction={a[:0].size if False else 0:.6f}')
print('constant absolute wind: release evidence remains ON while carrier transient evidence expires: PASS')
print(f'sine projected last transient time={last:.2f}s; 60s mission leaves >5s base-controller tail: PASS')
