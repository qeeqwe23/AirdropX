from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math,time
import numpy as np

from .app_config import MissionConfig
from .model_bank import ModelBank
from .jsbsim_runtime import JSBSimPlant,calibrate_delta_wind_maps
from .sensors import PaperSensor
from .wind import WindProfile,WindEstimator
from .ballistics import predict,integrate,fractional_release,P as BALLISTIC_PARAMS
from .paper_logic import PaperCarrierController
from .result_store import make_output_root,save


@dataclass
class LiveFrame:
    t: float
    x_m: float
    h_m: float
    h_est_m: float
    va_mps: float
    pitch_deg: float
    wind_mps: float
    wind_est_mps: float
    cfg: int
    elevator: float
    throttle: float
    target_m: float|None
    cargo_paths: list


class StandaloneSimulation:
    def __init__(self,app_root:Path):
        self.root=Path(app_root); self.stop_requested=False
        self.bank=ModelBank(self.root/'assets/controller/controller_bank.npz')
        self.plant_assets=self.root/'assets/jsbsim'; self._gw_cache={}

    def stop(self): self.stop_requested=True

    @staticmethod
    def _release_state(rs:dict,tau:float)->dict:
        out=dict(rs)
        out['x_m']=float(rs['x_m'])+float(rs['vx_ground_mps'])*tau
        out['h_m']=max(0.,float(rs['h_m'])+float(rs['vz_up_mps'])*tau)
        out['wind_est_mps']=float(np.clip(float(rs['wind_est_mps'])+float(rs['wind_rate_est_mps2'])*tau,-BALLISTIC_PARAMS['max_w'],BALLISTIC_PARAMS['max_w']))
        return out

    def run(self,cfg:MissionConfig,frame_cb=None,log_cb=None,progress_cb=None):
        errs=cfg.validate()
        if errs: raise ValueError('\n'.join(errs))
        self.stop_requested=False
        log=log_cb or (lambda s:None); emit=frame_cb or (lambda f:None); progress=progress_cb or (lambda x:None)
        out=make_output_root(cfg.output_root); Ts=.1; N=int(math.ceil(cfg.duration_s/Ts))+1
        models=[self.bank.vertex(cfg.target_altitude_m,cfg.target_speed_mps,i) for i in range(5)]

        key=(round(float(cfg.target_altitude_m),6),round(float(cfg.target_speed_mps),6))
        if key in self._gw_cache:
            gw_maps,gw_rows=self._gw_cache[key]
            log('[INIT] 复用当前软件进程内已验证的 H/V delta-wind Gw 缓存。')
        else:
            log('[INIT] 用独立 JSBSim 对 5 个载荷配置标定论文 delta-wind Gw（不调用 MATLAB）。')
            gw_maps,gw_rows=calibrate_delta_wind_maps(self.plant_assets,models,Ts=Ts)
            self._gw_cache[key]=(gw_maps,gw_rows)
        log('[INIT] Gw PASS: '+', '.join(f"cfg{r['cfg']} Va={r['Gw_Va']:+.4f}" for r in gw_rows))

        paper=PaperCarrierController(models,gw_maps,Ts)
        plant=JSBSimPlant(self.plant_assets)
        truth=plant.initialize(models[0].xref,models[0].uref,0)
        sensor=PaperSensor(Ts,cfg.sensor_noise_seed)
        wind_prof=WindProfile(cfg.wind); wind_est=WindEstimator(Ts)
        cfgid=0; target_idx=0; targets=[cfg.target_start_m+i*cfg.target_spacing_m for i in range(4)]
        series=[]; cargo=[]; cargo_visuals=[]; wall=time.perf_counter(); qp_ok=0; stopped=False
        log(f"[START] standalone JSBSim + Physics-MPC v1.3.6-Paper | H={cfg.target_altitude_m:g} m V={cfg.target_speed_mps:g} m/s | {cfg.envelope_label}")

        for k in range(N):
            if self.stop_requested:
                stopped=True; log('[STOP] 用户请求停止。'); break
            t=k*Ts; cfg_sample=cfgid; wind=float(wind_prof.value(t))

            # Plant truth is stimulus/scoring only. Paper sensor output is the controller path.
            obs=sensor.step(truth)
            eo=wind_est.step(obs['Va_mps'],obs['Vz_up_mps'],obs['Vg_long_mps'])
            wi=paper.observe(obs,eo)

            # Release guidance happens before the carrier solve, matching v1.3.6-Paper.
            scheduled=None; immediate_release=False; release_row=None
            if target_idx<4:
                rs=dict(
                    x_m=float(obs['pos_n_m']),
                    h_m=max(0.,float(obs['x_est'][0])),
                    vx_ground_mps=float(wi['guide_vg_mps']),
                    vz_up_mps=float(wi['guide_vz_mps']),
                    wind_est_mps=float(wi['wind_effective_mps']),
                    wind_rate_est_mps2=float(wi['wind_rate_effective_mps2']),
                )
                sr=fractional_release(rs,targets[target_idx],Ts)
                if sr['release_now']:
                    # Boundary release: configuration changes before this sample's MPC solve.
                    old_cfg=cfgid; release_t=t; tau=0.0; truth_release=truth
                    est_rel=self._release_state(rs,tau); pred_rel=predict(est_rel)
                    tr=integrate(float(truth_release['pos_n_m']),float(truth_release['h_m']),float(truth_release['Vg_long_mps']),float(truth_release['Vz_up_mps']),lambda age:float(wind_prof.value(release_t+age)))
                    release_row=dict(index=target_idx+1,target_m=targets[target_idx],release_t_s=release_t,release_phase_s=0.0,fractional_release=False,scheduler_mode=sr['mode'],scheduler_residual_m=sr['scheduler_residual_m'],release_x_est_m=est_rel['x_m'],release_h_est_m=est_rel['h_m'],predicted_impact_m=pred_rel['impact_x_m'],truth_impact_m=tr['impact_x_m'],landing_error_m=tr['impact_x_m']-targets[target_idx],fall_time_s=tr['fall_time_s'])
                    cargo.append(release_row); cargo_visuals.append((release_t,tr['path_timed']))
                    log(f"[DROP] #{target_idx+1} t={release_t:.3f}s phase=0 target={targets[target_idx]:.1f}m impact={tr['impact_x_m']:.2f}m error={release_row['landing_error_m']:+.2f}m")
                    target_idx+=1; cfgid=min(4,cfgid+1); immediate_release=True
                    paper.reset_transition(old_cfg,cfgid)
                elif sr['release_within_sample']:
                    scheduled=dict(sr=sr,rs=rs,index=target_idx,target=targets[target_idx],old_cfg=cfgid)

            # Existing Paper transient-evidence/recovery logic; calm/settled constant wind falls back to base MPC.
            solve_cfg=cfgid
            sol,diag=paper.solve(solve_cfg,obs['x_est'],wi); qp_ok+=int(sol.feasible)
            if not sol.feasible: raise RuntimeError(f"MPC QP failed at t={t:.2f}s cfg={solve_cfg}")

            truth_next=None; fractional_happened=False
            if scheduled is not None:
                sr=scheduled['sr']; rs=scheduled['rs']; old_cfg=scheduled['old_cfg']; tau=float(sr['tau_s']); release_t=t+tau
                truth_release=plant.step(sol.u,old_cfg,float(wind_prof.value(t)),tau) if tau>1e-9 else truth
                est_rel=self._release_state(rs,tau); pred_rel=predict(est_rel)
                tr=integrate(float(truth_release['pos_n_m']),float(truth_release['h_m']),float(truth_release['Vg_long_mps']),float(truth_release['Vz_up_mps']),lambda age:float(wind_prof.value(release_t+age)))
                release_row=dict(index=target_idx+1,target_m=targets[target_idx],release_t_s=release_t,release_phase_s=tau,fractional_release=True,scheduler_mode=sr['mode'],scheduler_residual_m=sr['scheduler_residual_m'],release_x_est_m=est_rel['x_m'],release_h_est_m=est_rel['h_m'],predicted_impact_m=pred_rel['impact_x_m'],truth_impact_m=tr['impact_x_m'],landing_error_m=tr['impact_x_m']-targets[target_idx],fall_time_s=tr['fall_time_s'])
                cargo.append(release_row); cargo_visuals.append((release_t,tr['path_timed']))
                log(f"[DROP] #{target_idx+1} t={release_t:.3f}s phase={tau:.4f}s target={targets[target_idx]:.1f}m impact={tr['impact_x_m']:.2f}m error={release_row['landing_error_m']:+.2f}m")
                target_idx+=1; cfgid=min(4,cfgid+1); fractional_happened=True
                rem=Ts-tau
                truth_next=plant.step(sol.u,cfgid,float(wind_prof.value(release_t)),rem) if rem>1e-9 else truth_release
                paper.after_step(old_cfg,sol,diag,wi,transitioned=True,new_cfg=cfgid)
            elif k<N-1:
                # Boundary release, if any, is applied by passing the new cfg here.
                truth_next=plant.step(sol.u,cfgid,wind,Ts)
                paper.after_step(solve_cfg,sol,diag,wi)

            row=dict(
                t_s=t,pos_n_m=float(truth['pos_n_m']),h_truth_m=float(truth['h_m']),h_est_m=float(obs['x_est'][0]),
                Va_truth_mps=float(truth['Va_mps']),Va_est_mps=float(obs['x_est'][1]),theta_est_rad=float(obs['x_est'][3]),q_est_radps=float(obs['x_est'][4]),
                wind_truth_mps=wind,wind_est_mps=float(eo['wind_est_mps']),wind_rate_est_mps2=float(eo['wind_rate_est_mps2']),wind_sigma_mps=float(eo['wind_sigma_mps']),
                wind_confidence=float(wi['wind_confidence']),wind_rate_confidence=float(wi['rate_confidence']),wind_effective_mps=float(wi['wind_effective_mps']),wind_rate_effective_mps2=float(wi['wind_rate_effective_mps2']),
                cfg=cfg_sample,controller_cfg=solve_cfg,elevator_cmd=float(sol.u[0]),throttle_cmd=float(sol.u[1]),mass_kg=float(truth['mass_kg']),qp_solve_ms=1000*sol.solve_time_s,
                drop_event=1 if release_row else 0,release_phase_s=float(release_row['release_phase_s']) if release_row else float('nan'),fractional_release=1 if fractional_happened else 0,
                carrier_transient_evidence=1 if wi['disturbance_evidence'] else 0,gust_trigger=1 if wi['gust_trigger'] else 0,recovery_level=float(diag.get('level',0.)),energy_href_shift_m=float(diag.get('energy_href_shift_m',0.)),residual_observer_norm=float(diag.get('residual_norm',0.)),
            )
            series.append(row)

            visible_cargo=[]
            display_t=t
            for release_t,path_timed in cargo_visuals:
                if display_t+1e-9<release_t: continue
                age=display_t-release_t; pts=[(px,ph) for tau,px,ph in path_timed if tau<=age+1e-9]
                if pts: visible_cargo.append(pts)
            emit(LiveFrame(t,float(truth['pos_n_m']),float(truth['h_m']),float(obs['x_est'][0]),float(obs['x_est'][1]),math.degrees(float(obs['x_est'][3])),wind,float(eo['wind_est_mps']),cfgid,float(sol.u[0]),float(sol.u[1]),targets[target_idx] if target_idx<4 else None,visible_cargo))
            progress(min(1.,k/max(1,N-1)))
            if truth_next is not None: truth=truth_next

            due=wall+(k+1)*Ts/max(cfg.realtime_factor,1e-9); delay=due-time.perf_counter()
            if delay>0: time.sleep(delay)

        lands=[abs(float(x['landing_error_m'])) for x in cargo]
        hdev=[abs(float(r['h_est_m'])-cfg.target_altitude_m) for r in series]
        mission_completed=not stopped and len(series)==N
        summary=dict(
            software_success=mission_completed,mission_completed=mission_completed,stopped_by_user=stopped,
            controller="Physics-MPC v1.3.6-Paper standalone numerical port",paper_logic_port=True,formal_recertification_required_after_runtime_port=True,
            validated_envelope=True,paper_baseline_point=cfg.is_paper_baseline,drops_completed=len(cargo),
            max_landing_error_m=max(lands) if lands else None,rms_landing_error_m=math.sqrt(sum(x*x for x in lands)/len(lands)) if lands else None,
            max_height_error_m=max(hdev) if hdev else None,qp_success_fraction=qp_ok/max(1,len(series)),
            carrier_transient_event_count=paper.transient_event_count,wind_disturbance_calibration=gw_rows,
            matlab_runtime_used=False,jsbsim_runtime="python/native",
        )
        save(out,cfg,series,cargo,summary)
        log(("[DONE] " if mission_completed else "[STOPPED] ")+str(out))
        progress(1. if mission_completed else min(1.,len(series)/max(1,N)))
        return out,summary
