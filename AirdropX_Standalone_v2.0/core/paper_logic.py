from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .model_bank import Vertex
from .mpc import DenseBoxMPC, MPCSolution
from .wind import wind_increment_preview


def smooth_step(x:float,a:float,b:float)->float:
    if b<=a: return float(x>=b)
    z=float(np.clip((float(x)-a)/(b-a),0.0,1.0))
    return z*z*(3.0-2.0*z)


def wind_confidence(wind_est:float,wind_rate:float,wind_sigma:float,wind_snr_half:float=2.5,wind_abs_half:float=.35,rate_half:float=.30)->dict:
    sigma=max(float(wind_sigma),.08); z=abs(float(wind_est))/sigma; p=4.0
    snr=(z**p)/(z**p+wind_snr_half**p)
    a=abs(float(wind_est)); abs_conf=(a**p)/(a**p+wind_abs_half**p)
    wc=float(np.clip(snr*abs_conf,0,1))
    rr=abs(float(wind_rate)); rate_signal=(rr**p)/(rr**p+rate_half**p)
    rate_unc=1.0/(1.0+(float(wind_sigma)/.60)**2)
    rc=float(np.clip(rate_signal*rate_unc,0,1))
    return dict(wind_confidence=wc,rate_confidence=rc,wind_effective_mps=wc*float(wind_est),wind_rate_effective_mps2=rc*float(wind_rate))


def adaptive_guide_filter(state:float|None,measurement:float,sigma:float,Ts:float,tau:float=.20,fast_sigma:float=3.0)->tuple[float,float]:
    m=float(measurement)
    if state is None or not math.isfinite(state): return m,m
    a=1.0-math.exp(-Ts/max(tau,1e-12))
    if abs(m-state)>=fast_sigma*max(float(sigma),.05): a=max(a,.85)
    state=state+a*(m-state)
    return state,state


@dataclass(frozen=True)
class PaperOptions:
    recovery_onset: float=.80
    recovery_full: float=3.00
    recovery_release: float=.55
    recovery_decay_s: float=1.50
    residual_alpha: float=.70
    residual_memory_s: float=.8
    residual_clip_norm: float=.25
    residual_deadband_norm: float=.02
    residual_full_norm: float=.12
    gust_step_threshold_mps: float=1.5
    gust_trigger_min_wind_conf: float=.35
    gust_trigger_delta_fraction: float=.85
    recovery_quiet_hold_s: float=.6
    carrier_rate_threshold_mps2: float=.70
    carrier_rate_confidence: float=.70
    carrier_rate_activation_wind_confidence: float=.15
    carrier_rate_hold_s: float=.30
    carrier_transient_persistence_s: float=3.50
    release_wind_evidence_confidence: float=.25
    energy_max_altitude_shift_m: float=3.0
    energy_max_altitude_shift_fraction: float=.02
    energy_va_onset_norm: float=.75
    energy_va_full_norm: float=2.5
    energy_throttle_high: float=.985
    energy_throttle_low: float=.015
    energy_reference_tau_s: float=.35
    energy_reference_decay_s: float=.8
    input_snap_tolerance: float=1e-7


class PaperCarrierController:
    """Faithful runtime port of the v1.3.6-Paper transient-evidence carrier path.

    No new control branch is introduced here. Calm/settled constant wind uses
    the base certified MPC exactly; the existing recovery cost bank is selected
    only after causal transient evidence, as in the Paper mission.
    """
    LEVELS=np.array([0.,.25,.5,.75,1.])
    MAX_Q=np.array([.85,4.0,.90,.90,1.20,1.,1.])
    MAX_R=np.array([.55,.85])

    def __init__(self,vertices:list[Vertex],gw_maps:np.ndarray,Ts:float=.1,opts:PaperOptions|None=None):
        self.vertices=vertices; self.gw=np.asarray(gw_maps,float); self.Ts=float(Ts); self.opts=opts or PaperOptions()
        self.base=[DenseBoxMPC(v,100) for v in vertices]
        self.recovery=[]
        for cfg,v in enumerate(vertices):
            bank=[]
            for level in self.LEVELS:
                if level==0:
                    bank.append(None); continue
                qm=1.+level*(self.MAX_Q-1.); rm=1.+level*(self.MAX_R-1.)
                bank.append(self.base[cfg].reweighted(qm,rm))
            self.recovery.append(bank)
        self.warm=np.zeros(200); self.d_hat=np.zeros(7); self.prev=None; self.observer_holdoff=0
        self.recovery_state=0.; self.gust_latched=False; self.recovery_quiet_count=0; self.energy_href_state=0.
        self.carrier_transient=False; self.rate_activation_count=0; self.rate_keep_count=0; self.transient_hold_count=0; self.transient_event_count=0
        self.prev_wind_est=None; self.guide_vg=None; self.guide_va=None; self.guide_vz=None
        self.last_wind=None

    def reset_transition(self,old_cfg:int,new_cfg:int,warm_source=None):
        old=self.base[int(old_cfg)]; new=self.base[int(new_cfg)]
        source=self.warm if warm_source is None else np.asarray(warm_source,float)
        self.warm=DenseBoxMPC.rebase_warm(source,old,new)
        self.d_hat=np.zeros(7); self.prev=None; self.observer_holdoff=1

    def observe(self,obs:dict,wind_obs:dict)->dict:
        o=self.opts; Ts=self.Ts
        we=float(wind_obs['wind_est_mps']); wr=float(wind_obs['wind_rate_est_mps2']); ws=float(wind_obs['wind_sigma_mps'])
        wc=wind_confidence(we,wr,ws)
        delta=0. if self.prev_wind_est is None or not math.isfinite(self.prev_wind_est) else we-self.prev_wind_est
        self.prev_wind_est=we
        gust=wc['wind_confidence']>=o.gust_trigger_min_wind_conf and abs(delta)>=o.gust_trigger_delta_fraction*o.gust_step_threshold_mps
        rate_dynamic=wc['rate_confidence']>=o.carrier_rate_confidence and abs(wc['wind_rate_effective_mps2'])>=o.carrier_rate_threshold_mps2
        rate_activation=rate_dynamic and wc['wind_confidence']>=o.carrier_rate_activation_wind_confidence
        self.rate_activation_count=self.rate_activation_count+1 if rate_activation else 0
        self.rate_keep_count=self.rate_keep_count+1 if rate_dynamic else 0
        hold=max(1,math.ceil(o.carrier_rate_hold_s/Ts)); persist=max(1,math.ceil(o.carrier_transient_persistence_s/Ts))
        was=self.carrier_transient
        if gust or self.rate_activation_count>=hold:
            self.carrier_transient=True; self.transient_hold_count=persist
        elif self.carrier_transient:
            if self.gust_latched or self.rate_keep_count>=hold:
                self.transient_hold_count=persist
            else:
                self.transient_hold_count=max(0,self.transient_hold_count-1)
                if self.transient_hold_count==0: self.carrier_transient=False
        if self.carrier_transient and not was: self.transient_event_count+=1
        self.guide_vg,_=adaptive_guide_filter(self.guide_vg,obs['Vg_long_mps'],.10,Ts)
        self.guide_va,_=adaptive_guide_filter(self.guide_va,obs['Va_mps'],.15,Ts)
        self.guide_vz,_=adaptive_guide_filter(self.guide_vz,obs['Vz_up_mps'],.07,Ts)
        out=dict(wind_est_mps=we,wind_rate_est_mps2=wr,wind_sigma_mps=ws,delta_wind_est_mps=delta,gust_trigger=gust,disturbance_evidence=self.carrier_transient,release_wind_evidence=wc['wind_confidence']>=o.release_wind_evidence_confidence,guide_vg_mps=self.guide_vg,guide_va_mps=self.guide_va,guide_vz_mps=self.guide_vz,**wc)
        self.last_wind=out
        return out

    def _residual_update(self,cfg:int,x_ctrl:np.ndarray,wind:dict):
        base=self.base[cfg]; dx=base.state_error(x_ctrl,base.xr); o=self.opts
        if wind['disturbance_evidence'] and self.prev is not None and self.prev['cfg']==cfg and self.observer_holdoff<=0:
            dw=wind['wind_effective_mps']-self.prev['wind_effective']
            pp=self.prev; pred=pp['ctrl'].A@pp['dx']+pp['ctrl'].B@pp['du']+self.gw[cfg]*dw
            innov=dx-pred; lim=o.residual_clip_norm*base.ss; innov=np.clip(innov,-lim,lim)
            n=float(np.max(np.abs(innov)/base.ss)); conf=smooth_step(n,o.residual_deadband_norm,o.residual_full_norm)
            self.d_hat=o.residual_alpha*self.d_hat+(1-o.residual_alpha)*(conf*innov)
        elif self.observer_holdoff>0:
            self.observer_holdoff-=1; self.d_hat=np.zeros(7)
        elif not wind['disturbance_evidence'] and not self.gust_latched:
            self.d_hat=np.zeros(7)
        return dx

    def solve(self,cfg:int,x_ctrl:np.ndarray,wind:dict)->tuple[MPCSolution,dict]:
        cfg=int(cfg); base=self.base[cfg]; dx=self._residual_update(cfg,x_ctrl,wind); o=self.opts; Ts=self.Ts
        if wind['gust_trigger']:
            self.gust_latched=True; self.recovery_quiet_count=0
        va_n=abs(dx[1])/base.ss[1]; h_n=abs(dx[0])/base.ss[0]; metric=max(va_n,.5*h_n)
        state_sev=smooth_step(metric,o.recovery_onset,o.recovery_full)
        target=state_sev if self.gust_latched else 0.
        if target>=self.recovery_state: self.recovery_state=target
        else: self.recovery_state=max(target,self.recovery_state*math.exp(-Ts/o.recovery_decay_s))
        if self.gust_latched and metric<=o.recovery_release:
            self.recovery_quiet_count+=1
            if self.recovery_quiet_count>=max(1,round(o.recovery_quiet_hold_s/Ts)): self.gust_latched=False
        else: self.recovery_quiet_count=0

        j=int(np.argmin(np.abs(self.LEVELS-self.recovery_state))); level=float(self.LEVELS[j]) if self.recovery_state>0 else 0.
        ctrl=base if level==0 else self.recovery[cfg][j]
        mpc_w=wind['wind_effective_mps'] if wind['disturbance_evidence'] else 0.
        mpc_r=wind['wind_rate_effective_mps2'] if wind['disturbance_evidence'] else 0.
        dw=wind_increment_preview(mpc_w,mpc_r,wind['wind_sigma_mps'],ctrl.N,Ts)
        decay=np.exp(-np.arange(ctrl.N)*Ts/o.residual_memory_s)
        g_seq=self.gw[cfg][:,None]*dw[None,:]+self.d_hat[:,None]*decay[None,:]
        use_transient=bool(wind['disturbance_evidence'] or self.gust_latched or level>0)
        if not use_transient:
            ctrl=base; g_seq=np.zeros((7,base.N)); sol=base.solve(x_ctrl,warm=self.warm)
        else:
            sol=ctrl.solve(x_ctrl,g_seq=g_seq,warm=self.warm)

        # Existing Paper energy-aware second pass only; no extra standalone tuning.
        desired=0.
        if self.gust_latched and level>=.5 and sol.feasible:
            va_signed=dx[1]/base.ss[1]; thr=float(sol.u[1])
            if (va_signed<=-o.energy_va_onset_norm and thr>=o.energy_throttle_high) or (va_signed>=o.energy_va_onset_norm and thr<=o.energy_throttle_low):
                amp=smooth_step(abs(va_signed),o.energy_va_onset_norm,o.energy_va_full_norm)
                cap=min(o.energy_max_altitude_shift_m,o.energy_max_altitude_shift_fraction*max(abs(base.xr[0]),1.))
                desired=cap*np.sign(va_signed)*amp*max(level,.5)
        alpha=1-math.exp(-Ts/(o.energy_reference_tau_s if abs(desired)>0 else o.energy_reference_decay_s))
        self.energy_href_state += alpha*(desired-self.energy_href_state)
        cap=min(o.energy_max_altitude_shift_m,o.energy_max_altitude_shift_fraction*max(abs(base.xr[0]),1.))
        self.energy_href_state=float(np.clip(self.energy_href_state,-cap,cap))
        if level>0 and abs(self.energy_href_state)>=1e-4:
            xr=base.xr.copy(); xr[0]+=self.energy_href_state
            e=ctrl.solve(x_ctrl,g_seq=g_seq,warm=self.warm,xref_override=xr)
            if e.feasible: sol=e

        if not sol.feasible: return sol,dict(level=level,recovery_state=self.recovery_state,g_seq=g_seq,dx=dx)
        # Physical snap is only numerical clipping to the already-hard bounds.
        raw=np.asarray(sol.u,float); phys=np.clip(raw,[-1.,0.],[1.,1.])
        if float(np.max(np.abs(raw-phys)))>o.input_snap_tolerance: raise RuntimeError('MPC physical input exceeded hard bound beyond numerical tolerance')
        du=phys-base.ur
        sol.u=phys; sol.du=du; sol.U[:2]=du
        diag=dict(level=level,recovery_state=self.recovery_state,gust_latched=self.gust_latched,gust_trigger=wind['gust_trigger'],disturbance_evidence=wind['disturbance_evidence'],wind_mpc_active=use_transient,forecast_dwind1=float(dw[0]),residual_norm=float(np.max(np.abs(self.d_hat)/base.ss)),energy_href_shift_m=self.energy_href_state,g_seq=g_seq,dx=dx,base=base)
        return sol,diag

    def after_step(self,cfg:int,sol:MPCSolution,diag:dict,wind:dict,transitioned:bool=False,new_cfg:int|None=None):
        base=self.base[int(cfg)]
        if transitioned:
            self.reset_transition(int(cfg),int(new_cfg),warm_source=sol.U)
            return
        self.prev=dict(ctrl=base,dx=np.asarray(diag['dx'],float).copy(),du=np.asarray(sol.du,float).copy(),wind_effective=float(wind['wind_effective_mps']),cfg=int(cfg))
        self.warm=DenseBoxMPC.shift_warm(sol.U,base.m)
