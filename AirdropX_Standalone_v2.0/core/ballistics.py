from __future__ import annotations
import math

def params():
    g=9.80665; H=20.; V=78.6; R=150.7649; T=math.sqrt(2*H/g); c=6.8e-4
    for _ in range(12):
        z=1+c*V*T; f=math.log(z)/c-R; df=(c*(V*T/z)-math.log(z))/(c*c); c-=f/df
    return dict(g=g,drag=c,dt=.01,max_t=30.,max_w=20.,max_rate=3.)
P=params()

def integrate(x,h,vx,vz,wind_fun):
    x0=x; t=0.; dt=P['dt']
    if h<=0: return dict(impact_x_m=x,fall_time_s=0.,range_m=0.,path=[(x,0.)],path_timed=[(0.,x,0.)])
    path=[(x,h)]; path_timed=[(0.,x,h)]
    for k in range(int(P['max_t']/dt)+1):
        w=wind_fun(t); vr=vx-w; ax=-P['drag']*vr*abs(vr)
        vz2=vz-P['g']*dt; vx2=vx+ax*dt; h2=h+.5*(vz+vz2)*dt; x2=x+.5*(vx+vx2)*dt
        if k%10==0:
            path.append((x2,max(0.,h2))); path_timed.append((t+dt,x2,max(0.,h2)))
        if h2<=0:
            frac=h/(h-h2); xi=x+frac*(x2-x); ti=t+frac*dt; path.append((xi,0.)); path_timed.append((ti,xi,0.))
            return dict(impact_x_m=xi,fall_time_s=ti,range_m=xi-x0,path=path,path_timed=path_timed)
        x,h,vx,vz=x2,h2,vx2,vz2; t+=dt
    raise RuntimeError("Cargo did not reach ground within 30 s")

def predict(release):
    w0=float(release['wind_est_mps']); rate=max(-P['max_rate'],min(P['max_rate'],float(release['wind_rate_est_mps2'])))
    wf=lambda tau:max(-P['max_w'],min(P['max_w'],w0+rate*tau))
    return integrate(release['x_m'],release['h_m'],release['vx_ground_mps'],release['vz_up_mps'],wf)

def fractional_release(release: dict, target_m: float, Ts: float, oracle_dt_s: float = 1/120, min_impact_advance_m: float = 0.25):
    """Causal v1.3.1-style sub-sample release scheduler, aligned to JSBSim's grid."""
    now=predict(release)
    def extrap(tau):
        r=dict(release)
        r['x_m']=float(release['x_m'])+float(release['vx_ground_mps'])*tau
        r['h_m']=max(0.,float(release['h_m'])+float(release['vz_up_mps'])*tau)
        r['wind_est_mps']=float(np_clip(float(release['wind_est_mps'])+float(release['wind_rate_est_mps2'])*tau,-P['max_w'],P['max_w']))
        return r
    nxt=predict(extrap(Ts)); advance=nxt['impact_x_m']-now['impact_x_m']
    out=dict(release_now=False,release_within_sample=False,tau_s=float('nan'),estimated_impact_at_release_m=float('nan'),scheduler_residual_m=float('nan'),mode='wait')
    if now['impact_x_m']>=target_m:
        out.update(release_now=True,tau_s=0.,estimated_impact_at_release_m=now['impact_x_m'],scheduler_residual_m=now['impact_x_m']-target_m,mode='immediate_boundary'); return out
    if not math.isfinite(advance) or advance<min_impact_advance_m or nxt['impact_x_m']<target_m: return out
    tau=float(np_clip(Ts*(target_m-now['impact_x_m'])/advance,0.,Ts))
    slope=advance/max(Ts,1e-12)
    for _ in range(2):
        pi=predict(extrap(tau))
        if abs(slope)<1e-6: break
        tau=float(np_clip(tau-(pi['impact_x_m']-target_m)/slope,0.,Ts))
    tau=round(tau/oracle_dt_s)*oracle_dt_s; tau=float(np_clip(tau,0.,Ts))
    if tau>=Ts-1e-6: return out
    pi=predict(extrap(tau))
    out.update(release_within_sample=True,tau_s=tau,estimated_impact_at_release_m=pi['impact_x_m'],scheduler_residual_m=pi['impact_x_m']-target_m,mode='fractional_timer')
    return out

def np_clip(x,a,b):
    return a if x<a else b if x>b else x
