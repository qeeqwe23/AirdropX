import math
import numpy as np

G=9.80665
CAL_H=20.0
CAL_V=78.6
CAL_R=150.7649
DT=0.01
H=200.0
VG=50.0
VZ=0.0
TS=0.1

def drag_calibration():
    T=math.sqrt(2*CAL_H/G)
    c=6.8e-4
    for _ in range(12):
        z=1+c*CAL_V*T
        f=math.log(z)/c-CAL_R
        df=(c*(CAL_V*T/z)-math.log(z))/(c*c)
        c-=f/df
    return c
C=drag_calibration()

def impact(x,h,vx,vz,wind0=0.0,rate=0.0):
    t=0.0; x0=x
    for _ in range(math.ceil(30/DT)):
        w=max(-20.0,min(20.0,wind0+max(-3,min(3,rate))*t))
        vrel=vx-w
        ax=-C*vrel*abs(vrel)
        vz2=vz-G*DT
        vx2=vx+ax*DT
        h2=h+0.5*(vz+vz2)*DT
        x2=x+0.5*(vx+vx2)*DT
        if h2<=0:
            frac=h/(h-h2)
            return x+frac*(x2-x), t+frac*DT
        x,h,vx,vz=x2,h2,vx2,vz2; t+=DT
    raise RuntimeError('no impact')

base_impact,fall=impact(0,H,VG,VZ)
range0=base_impact
sample_step=VG*TS

# Sweep every possible phase between a target and the 5 m release grid.
phases=np.linspace(0,sample_step,10001,endpoint=False)
legacy=[]; frac=[]; taus=[]
for phase in phases:
    target=1200.0+phase
    # Current grid point immediately before ideal release position.
    ideal_x=target-range0
    x0=math.floor(ideal_x/sample_step)*sample_step
    # v1.3.0/v1.2.1-style nearest-sample gate: first grid impact within half a sample.
    xs=x0
    while impact(xs,H,VG,VZ)[0] < target-0.5*sample_step:
        xs+=sample_step
    legacy.append(impact(xs,H,VG,VZ)[0]-target)
    # v1.3.1: target crossing within [x0,x0+Vg*Ts], solved as fractional timer.
    p0=impact(x0,H,VG,VZ)[0]
    p1=impact(x0+sample_step,H,VG,VZ)[0]
    tau=TS*(target-p0)/(p1-p0)
    tau=max(0,min(TS,tau))
    xf=x0+VG*tau
    frac.append(impact(xf,H,VG,VZ)[0]-target)
    taus.append(tau)
legacy=np.asarray(legacy); frac=np.asarray(frac); taus=np.asarray(taus)

print('AirdropX v1.3.1 independent fractional-release audit')
print(f'drag_per_m={C:.12g}')
print(f'calm_H200_Vg50_ballistic_range_m={range0:.9f}')
print(f'fall_time_s={fall:.9f}')
print(f'MPC_sample_s={TS:.3f}')
print(f'release_grid_spacing_at_50mps_m={sample_step:.3f}')
print(f'legacy_quantization_worst_abs_m={np.max(np.abs(legacy)):.9f}')
print(f'legacy_quantization_rms_m={math.sqrt(float(np.mean(legacy**2))):.9f}')
print(f'fractional_worst_abs_m={np.max(np.abs(frac)):.12g}')
print(f'fractional_rms_m={math.sqrt(float(np.mean(frac**2))):.12g}')
print(f'tau_min_s={taus.min():.9f}')
print(f'tau_max_s={taus.max():.9f}')
assert abs(range0-289.04378)<0.02
assert np.max(np.abs(legacy)) <= 2.500001
assert np.max(np.abs(legacy)) > 2.49
assert np.max(np.abs(frac)) < 1e-8
assert taus.min() >= -1e-12 and taus.max() <= TS+1e-12
print('PASS=1')
