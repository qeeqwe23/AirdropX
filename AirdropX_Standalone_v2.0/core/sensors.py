from __future__ import annotations
import math
import numpy as np

class PaperSensor:
    """v1.3.6-Paper unbiased white-noise sensor path, ported to Python."""
    def __init__(self, Ts: float, seed: int):
        self.Ts=Ts; self.rng=np.random.default_rng(seed); self.initialized=False
        self.est={}
        self.sig=dict(pos=.30,vg=.10,vz=.07,h=.20,va=.15,theta=math.radians(.03),q=math.radians(.01),n1=.05,n2=.05)
        self.tau=dict(pos=.15,vel=.12,h=.15,va=.10,att=.08,eng=.10)

    @staticmethod
    def _blend(a,b,k):
        d=math.atan2(math.sin(b-a),math.cos(b-a)); return a+k*d

    def step(self, truth: dict):
        r=self.rng; s=self.sig
        m=dict(pos=truth['pos_n_m']+s['pos']*r.normal(), vg=truth['Vg_long_mps']+s['vg']*r.normal(), vz=truth['Vz_up_mps']+s['vz']*r.normal(),
               h=truth['h_m']+s['h']*r.normal(), va=max(.1,truth['Va_mps']+s['va']*r.normal()), theta=truth['theta_rad']+s['theta']*r.normal(),
               q=truth['q_radps']+s['q']*r.normal(), n1=truth['N1']+s['n1']*r.normal(), n2=truth['N2']+s['n2']*r.normal())
        if not self.initialized:
            self.est=m.copy(); self.est['gamma']=math.atan2(m['vz'],max(abs(m['vg']),1e-3)); self.initialized=True
        else:
            T=self.Ts
            alpha={k:1-math.exp(-T/v) for k,v in self.tau.items()}
            e=self.est
            e['vg'] += alpha['vel']*(m['vg']-e['vg']); e['vz'] += alpha['vel']*(m['vz']-e['vz'])
            pp=e['pos']+e['vg']*T; e['pos']=pp+alpha['pos']*(m['pos']-pp)
            hp=e['h']+e['vz']*T; e['h']=hp+alpha['h']*(m['h']-hp)
            e['va'] += alpha['va']*(m['va']-e['va']); e['theta']=self._blend(e['theta'],m['theta'],alpha['att'])
            e['q'] += alpha['att']*(m['q']-e['q']); e['n1'] += alpha['eng']*(m['n1']-e['n1']); e['n2'] += alpha['eng']*(m['n2']-e['n2'])
            gm=math.atan2(e['vz'],max(abs(e['vg']),1e-3)); e['gamma']=self._blend(e['gamma'],gm,alpha['vel'])
        e=self.est
        x=np.array([e['h'],e['va'],e['gamma'],e['theta'],e['q'],e['n1'],e['n2']],dtype=float)
        return {"x_est":x,"pos_n_m":e['pos'],"Vg_long_mps":e['vg'],"Vz_up_mps":e['vz'],"Va_mps":e['va']}
