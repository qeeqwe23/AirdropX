from __future__ import annotations
import math
import numpy as np
from .app_config import WindConfig

class WindProfile:
    def __init__(self, cfg: WindConfig):
        self.cfg = cfg
        rng=np.random.default_rng(cfg.random_seed)
        self._turb_amp=rng.normal(size=6); self._turb_phase=rng.uniform(0,2*math.pi,size=6)

    def value(self, t: float) -> float:
        c = self.cfg
        if c.mode == "formal":
            return self._formal(c.kind, t, c)
        a = c.along_track_mps()
        k = c.kind
        if k == "calm": return 0.0
        if k == "constant": return a
        if k == "step": return a if t >= c.start_s else 0.0
        if k == "bidirectional_step":
            if t < c.start_s: return 0.0
            if t < c.start_s + 10: return a
            if t < c.start_s + 20: return -a
            return 0.375 * a
        if k == "ramp":
            if t < c.start_s: return 0.0
            z = min(1.0, max(0.0, (t-c.start_s)/max(c.ramp_s, 1e-6)))
            return a * (2*z - 1)
        if k == "sine":
            if t < c.start_s: return 0.0
            return a * math.sin(2*math.pi*(t-c.start_s)/max(c.period_s, 1e-6))
        if k == "turbulence":
            # Pure deterministic band-limited gust for a given seed, so querying cargo future
            # wind never changes the carrier's later wind history.
            freqs=np.array([0.035,0.061,0.097,0.143,0.211,0.307])
            z=float(np.sum(self._turb_amp*np.sin(2*math.pi*freqs*t+self._turb_phase)))
            z/=max(1e-9,float(np.linalg.norm(self._turb_amp)))
            return float(np.clip(a + 0.35*max(abs(a),2.0)*z, -20.0, 20.0))
        raise ValueError(f"Unknown wind kind: {k}")

    @staticmethod
    def _formal(name: str, t: float, c: WindConfig) -> float:
        if name == "calm": return 0.0
        if name == "tailwind_5": return 5.0 if t >= 5 else 0.0
        if name == "headwind_5": return -5.0 if t >= 5 else 0.0
        if name == "tailwind_12": return 12.0 if t >= 5 else 0.0
        if name == "headwind_12": return -12.0 if t >= 5 else 0.0
        if name == "step_bidirectional":
            if t < 5: return 0.0
            if t < 15: return 8.0
            if t < 25: return -8.0
            return 3.0
        if name == "ramp_minus10_plus10":
            if t < 5: return 0.0
            if t < 25: return -10.0 + (t-5.0)
            return 10.0
        if name == "sine_longitudinal":
            if t < 5: return 0.0
            if t < c.forcing_end_s:
                return 2.0 + 6.0*math.sin(2*math.pi*(t-5.0)/20.0)
            if t < c.forcing_end_s + c.settle_ramp_s:
                w0 = 2.0 + 6.0*math.sin(2*math.pi*(c.forcing_end_s-5.0)/20.0)
                z = (t-c.forcing_end_s)/c.settle_ramp_s
                return w0*0.5*(1.0+math.cos(math.pi*z))
            return 0.0
        raise ValueError(f"Unknown formal wind: {name}")

class WindEstimator:
    """Direct port of v1.1.1 two-state adaptive Kalman longitudinal observer."""
    def __init__(self, Ts: float, sigma_ground=0.10, sigma_air=0.15, sigma_vz=0.07):
        self.Ts=Ts; self.sg=sigma_ground; self.sa=sigma_air; self.sv=sigma_vz
        self.x=np.zeros(2); self.P=np.diag([16.0,4.0]); self.F=np.array([[1.,Ts],[0.,1.]])
        q=1.0; self.Q=q*np.array([[Ts**3/3,Ts**2/2],[Ts**2/2,Ts]])
        self.high=0

    def step(self, Va: float, Vz_up: float, Vg: float):
        self.x=self.F@self.x; self.P=self.F@self.P@self.F.T+self.Q
        valid=np.isfinite([Va,Vz_up,Vg]).all() and Va>0 and Vg>=0 and abs(Vz_up)<Va
        raw=float('nan'); nis=float('nan'); step=False
        if valid:
            vh2=Va*Va-Vz_up*Vz_up
            valid=vh2>25.0
        if valid:
            Vah=math.sqrt(vh2); raw=Vg-Vah
            R=self.sg**2+(Va/Vah*self.sa)**2+(-Vz_up/Vah*self.sv)**2
            innov=raw-self.x[0]; S=self.P[0,0]+R; nis=innov*innov/S
            immediate=nis>64.0
            self.high=self.high+1 if nis>16.0 else 0
            if immediate or self.high>=2:
                self.x=np.array([raw,0.]); self.P=np.diag([max(R,0.04),4.]); self.high=0; step=True
            else:
                K=self.P[:,0]/S; self.x=self.x+K*innov
                H=np.array([[1.,0.]]); I=np.eye(2); Kc=K[:,None]
                self.P=(I-Kc@H)@self.P@(I-Kc@H).T + R*(Kc@Kc.T)
                self.P=0.5*(self.P+self.P.T)
        return {"wind_est_mps":float(self.x[0]),"wind_rate_est_mps2":float(self.x[1]),"wind_sigma_mps":math.sqrt(max(float(self.P[0,0]),0.0)),"raw_wind_mps":raw,"nis":nis,"step_detected":step}


def wind_increment_preview(wind_est_mps: float, wind_rate_mps2: float, wind_sigma_mps: float, N: int, Ts: float, rate_cap_mps2: float = 3.0, rate_memory_s: float = 2.0, wind_abs_cap_mps: float = 20.0) -> np.ndarray:
    """Port of airdropx_phys_mpc_wind_forecast_v130: causal delta-wind only."""
    r=float(np.clip(wind_rate_mps2,-rate_cap_mps2,rate_cap_mps2))
    confidence=1.0/(1.0+(float(wind_sigma_mps)/1.0)**2)
    confidence=float(np.clip(confidence,0.25,1.0)); r*=confidence
    dw=np.zeros(int(N),dtype=float); w=float(wind_est_mps)
    for i in range(int(N)):
        rate=r*math.exp(-(i*Ts)/rate_memory_s)
        step=rate*Ts
        wn=float(np.clip(w+step,-wind_abs_cap_mps,wind_abs_cap_mps))
        dw[i]=wn-w; w=wn
    return dw
