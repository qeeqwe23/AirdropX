import math

def replay(wconf,rconf,west,Ts=0.1):
    act_conf=0.25; release_conf=0.08; act_hold=0.30; persist=0.80; rate_extend=0.70; gust_conf=0.35; gust_thresh=0.85*1.5
    active=False; strong=0; hold=0; out=[]; gusts=0; prev=None
    actN=max(1,math.ceil(act_hold/Ts)); persN=max(1,math.ceil(persist/Ts))
    for wc,rc,w in zip(wconf,rconf,west):
        delta=0 if prev is None else w-prev
        gust=prev is not None and wc>=gust_conf and abs(delta)>=gust_thresh
        gusts += int(gust)
        strong=strong+1 if wc>=act_conf else 0
        if gust or strong>=actN:
            active=True; hold=persN
        elif active:
            if wc>=release_conf or rc>=rate_extend:
                hold=persN
            else:
                hold=max(0,hold-1)
                if hold==0: active=False
        out.append(active); prev=w
    return out,gusts

# Calm: deliberately high rate confidence spikes but no persistent amplitude evidence.
n=200
wc=[0.01]*n; rc=[0.85 if i%4==0 else 0.3 for i in range(n)]; w=[0.2*math.sin(i) for i in range(n)]
a,g=replay(wc,rc,w)
assert not any(a) and g==0
# Gradual real wind: persistent amplitude evidence activates after 0.3 s.
wc=[0.4]*10; rc=[0.4]*10; w=[0.4+0.1*i for i in range(10)]
a,g=replay(wc,rc,w); assert a[2] and g==0
# Abrupt qualified gust activates immediately.
wc=[0.01,0.6,0.6]; rc=[0.2]*3; w=[0,2,2]
a,g=replay(wc,rc,w); assert a[1] and g==1
print('PASS: calm rate-noise cannot activate disturbance path')
print('PASS: persistent amplitude evidence activates after 0.30 s')
print('PASS: qualified abrupt gust activates immediately')
print('3/3 PASS')
