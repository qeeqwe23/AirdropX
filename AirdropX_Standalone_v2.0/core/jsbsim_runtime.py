from __future__ import annotations
from pathlib import Path
import math
import numpy as np

FT_TO_M=0.3048
SLUG_TO_KG=14.59390294
SLUGFT2_TO_KGM2=1.3558179483314004

class JSBSimPlant:
    """Direct JSBSim Python binding wrapper. It never launches MATLAB or a MEX host."""
    def __init__(self,asset_root: str|Path,dt:float=1/120,aircraft="MQ9_Reaper"):
        try:
            import jsbsim
        except Exception as exc:
            raise RuntimeError("缺少 JSBSim Python runtime。请重新运行 Build_Standalone.bat。") from exc
        self.jsbsim=jsbsim; self.root=Path(asset_root); self.dt=float(dt); self.aircraft=aircraft
        if not (self.root/'aircraft').is_dir() or not (self.root/'engine').is_dir():
            raise RuntimeError(f"JSBSim 运行资产不完整：{self.root}")
        self.fdm=jsbsim.FGFDMExec(str(self.root))
        if hasattr(self.fdm,'set_debug_level'): self.fdm.set_debug_level(0)
        self.fdm.set_aircraft_path('aircraft'); self.fdm.set_engine_path('engine'); self.fdm.set_systems_path('systems')
        if not self.fdm.load_model(aircraft): raise RuntimeError(f"JSBSim 无法加载 {aircraft}")
        self.fdm.set_dt(self.dt); self.pos_n=0.; self.time=0.; self.cfg=0; self._cargo=[]
        for i in range(4): self._cargo.append(self.get(f'inertia/pointmass-weight-lbs[{i}]',0.0))
        if any(x<=0 for x in self._cargo): raise RuntimeError("MQ9_Reaper 模型未暴露 4 个 cargo point mass")

    def get(self,key,default=0.):
        try:
            v=float(self.fdm[key]); return v if math.isfinite(v) else default
        except Exception: return default

    def set(self,key,val,required=False):
        try:
            self.fdm[key]=float(val); return True
        except Exception:
            if required: raise RuntimeError(f"JSBSim property not writable: {key}")
            return False

    def _set_cfg(self,cfg:int):
        cfg=int(cfg)
        for i,w in enumerate(self._cargo):
            self.set(f'inertia/pointmass-weight-lbs[{i}]',0.0 if i<cfg else w,True)
        self.cfg=cfg

    def _set_ic(self,x):
        self.set('ic/h-agl-ft',x[0]/FT_TO_M,True); self.set('ic/vt-fps',x[1]/FT_TO_M,True); self.set('ic/gamma-rad',x[2],True)
        self.set('ic/alpha-rad',x[3]-x[2],True); self.set('ic/beta-rad',0.); self.set('ic/phi-rad',0.); self.set('ic/psi-true-rad',0.)
        self.set('ic/p-rad_sec',0.); self.set('ic/q-rad_sec',x[4]); self.set('ic/r-rad_sec',0.)
        self.set('ic/vw-north-fps',0.); self.set('ic/vw-east-fps',0.); self.set('ic/vw-down-fps',0.)

    def _seed_engine(self,n1:float,n2:float,throttle:float):
        # Physics bank carries the steady-engine trim state. Prefer exact property seeding;
        # if a particular JSBSim build exposes those properties read-only, start-running +
        # trim throttle still gives the engine model a physically valid state.
        self.set('propulsion/set-running',-1.0)
        self.set('propulsion/engine[0]/set-running',1.0)
        self.set('propulsion/engine/set-running',1.0)
        self.set('propulsion/engine[0]/n1',n1); self.set('propulsion/engine/n1',n1)
        self.set('propulsion/engine[0]/n2',n2); self.set('propulsion/engine/n2',n2)
        self.apply_control([0.0,throttle])

    def initialize(self,xref,uref,cfg=0):
        x=np.asarray(xref,float).reshape(7); u=np.asarray(uref,float).reshape(2); self._set_cfg(cfg); self._set_ic(x)
        if not self.fdm.run_ic(): raise RuntimeError("JSBSim RunIC failed")
        self._seed_engine(x[5],x[6],u[1]); self.apply_control(u); self.set_wind(0.)
        self.pos_n=0.; self.time=0.
        return self.read()

    def apply_control(self,u):
        e=float(np.clip(u[0],-1,1)); t=float(np.clip(u[1],0,1))
        self.set('fcs/elevator-cmd-norm',e); self.set('fcs/elevator-pos-norm',e)
        self.set('fcs/throttle-cmd-norm',t); self.set('fcs/throttle-pos-norm',t)
        self.set('fcs/throttle-cmd-norm[0]',t); self.set('fcs/throttle-pos-norm[0]',t)

    def set_wind(self,w):
        self.set('atmosphere/wind-north-fps',float(w)/FT_TO_M); self.set('atmosphere/wind-east-fps',0.); self.set('atmosphere/wind-down-fps',0.)

    def step(self,u,cfg,wind,Ts=.1):
        if int(cfg)!=self.cfg: self._set_cfg(int(cfg))
        self.apply_control(u); n=max(1,int(round(Ts/self.dt)))
        for _ in range(n):
            self.set_wind(wind)
            if not self.fdm.run(): raise RuntimeError("JSBSim Run failed")
            self.pos_n += self.get('velocities/v-north-fps')*FT_TO_M*self.dt; self.time += self.dt
        return self.read()

    def read(self):
        n1=self.get('propulsion/engine[0]/n1',self.get('propulsion/engine/n1',80.)); n2=self.get('propulsion/engine[0]/n2',self.get('propulsion/engine/n2',80.))
        x=np.array([self.get('position/h-agl-ft')*FT_TO_M,self.get('velocities/vtrue-fps')*FT_TO_M,self.get('flight-path/gamma-rad'),self.get('attitude/theta-rad'),self.get('velocities/q-rad_sec'),n1,n2])
        return dict(x=x,h_m=x[0],Va_mps=x[1],gamma_rad=x[2],theta_rad=x[3],q_radps=x[4],N1=x[5],N2=x[6],pos_n_m=self.pos_n,
                    Vg_long_mps=self.get('velocities/v-north-fps')*FT_TO_M,Vz_up_mps=-self.get('velocities/v-down-fps')*FT_TO_M,
                    mass_kg=self.get('inertia/mass-slugs')*SLUG_TO_KG,cg_x_m=self.get('inertia/cg-x-in')*.0254,Iyy_kgm2=self.get('inertia/iyy-slugs_ft2')*SLUGFT2_TO_KGM2)


def calibrate_delta_wind_maps(asset_root: str|Path, models, probes=(0.5,1.0,2.0), Ts=.1):
    """Port of v1.3.0 JSBSim +/-wind central-difference calibration."""
    rows=[]; maps=[]; plant=JSBSimPlant(asset_root)
    for cfg,model in enumerate(models):
        cols=[]
        for h in probes:
            plant.initialize(model.xref,model.uref,cfg); xp=plant.step(model.uref,cfg,+float(h),Ts)['x']
            plant.initialize(model.xref,model.uref,cfg); xm=plant.step(model.uref,cfg,-float(h),Ts)['x']
            ep=np.asarray(xp)-model.xref; em=np.asarray(xm)-model.xref
            for q in (ep,em):
                for i in (2,3): q[i]=math.atan2(math.sin(q[i]),math.cos(q[i]))
            cols.append((ep-em)/(2*float(h)))
        G=np.column_stack(cols); j=int(np.argmin(np.abs(np.asarray(probes)-1.0))); Gw=G[:,j]
        spread=np.max(np.abs(G-Gw[:,None]),axis=1); spread_norm=float(np.max(spread/np.maximum(model.state_scale,1e-12)))
        if not np.isfinite(Gw).all() or Gw[1]>=-0.20:
            raise RuntimeError(f"JSBSim 风扰动标定失败 cfg{cfg}: 预期顺风增量降低 Va，实际 Gw(Va)={Gw[1]:+.6g}")
        maps.append(Gw); rows.append(dict(cfg=cfg,Gw_Va=float(Gw[1]),probe_spread_norm=spread_norm))
    return np.vstack(maps),rows
