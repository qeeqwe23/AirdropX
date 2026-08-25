import math
from pathlib import Path

G=9.80665
H_CAL=20.0
V_CAL=78.6
R_CAL=150.7649
M=300.0
RHO0=1.225
HS=8500.0
DT=0.005


def new_range(cda, h=H_CAL, vx=V_CAL, vz=0.0, wind=0.0, max_t=35.0):
    x=0.0
    n=int(math.ceil(max_t/DT))
    for _ in range(n):
        rho=RHO0*math.exp(-max(h,0.0)/HS)
        vrx=vx-wind; vrz=vz; sp=math.hypot(vrx,vrz); kd=0.5*rho*cda/M
        ax=-kd*sp*vrx; az=-G-kd*sp*vrz
        vx2=vx+ax*DT; vz2=vz+az*DT
        x2=x+0.5*(vx+vx2)*DT; h2=h+0.5*(vz+vz2)*DT
        if h2<=0:
            f=h/(h-h2)
            return x+f*(x2-x), (_+f)*DT
        x,h,vx,vz=x2,h2,vx2,vz2
    raise RuntimeError('no impact')

lo,hi=0.01,2.0
for _ in range(70):
    mid=0.5*(lo+hi); r,_=new_range(mid)
    if r>R_CAL: lo=mid
    else: hi=mid
cda=0.5*(lo+hi)
r_cal,t_cal=new_range(cda)

# old v1.2.1 predictor calibration/plant structure
T=math.sqrt(2*H_CAL/G)
c=6.8e-4
for _ in range(12):
    z=1+c*V_CAL*T
    f=math.log(z)/c-R_CAL
    df=(c*(V_CAL*T/z)-math.log(z))/(c*c)
    c=c-f/df

def old_predict(h=200.0,vx=50.0,vz=0.0,wind=0.0,dt=0.01,max_t=30.0):
    x=0.0;t=0.0
    for k in range(int(math.ceil(max_t/dt))):
        vrel=vx-wind
        ax=-c*vrel*abs(vrel)
        vz2=vz-G*dt; vx2=vx+ax*dt
        h2=h+0.5*(vz+vz2)*dt; x2=x+0.5*(vx+vx2)*dt
        if h2<=0:
            frac=h/(h-h2)
            return x+frac*(x2-x),t+frac*dt
        x,h,vx,vz,t=x2,h2,vx2,vz2,t+dt
    raise RuntimeError('old no impact')

new200,tnew=new_range(cda,h=200.0,vx=50.0,vz=0.0,wind=0.0)
old200,told=old_predict()
mismatch=new200-old200

checks=[
    ('independent calibration error < 1e-6 m', abs(r_cal-R_CAL)<1e-6),
    ('CdA in sane positive range', 0.05<cda<1.5),
    ('200m independent plant impacts', 0<tnew<35 and new200>0),
    ('200m old predictor impacts', 0<told<30 and old200>0),
    ('structural mismatch is nontrivial', abs(mismatch)>0.5),
    ('structural mismatch is not catastrophic', abs(mismatch)<20.0),
]
for n,v in checks: print(f"{'PASS' if v else 'FAIL'}  {n}")
print(f"\nCdA_m2={cda:.12f}")
print(f"calibration_range_m={r_cal:.12f}")
print(f"calibration_error_m={r_cal-R_CAL:.12e}")
print(f"old_predictor_200m_range_m={old200:.9f}")
print(f"independent_truth_200m_range_m={new200:.9f}")
print(f"structural_range_difference_m={mismatch:.9f}")
print(f"old_predictor_200m_fall_time_s={told:.9f}")
print(f"independent_truth_200m_fall_time_s={tnew:.9f}")
if not all(v for _,v in checks): raise SystemExit(1)
