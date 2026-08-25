from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np

# Running as a script: place app root on sys.path.
import sys
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))

from core.model_bank import ModelBank
from core.mpc import DenseBoxMPC
from core.jsbsim_runtime import JSBSimPlant, calibrate_delta_wind_maps


def _norm(x, ref, scale):
    e=DenseBoxMPC.state_error(np.asarray(x,float),np.asarray(ref,float))
    return float(np.max(np.abs(e)/np.maximum(np.asarray(scale,float),1e-12)))


def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--app-root',type=Path,default=ROOT)
    ns=ap.parse_args(); root=ns.app_root.resolve()
    bank=ModelBank(root/'assets/controller/controller_bank.npz')
    H,V=200.0,50.0
    models=[bank.vertex(H,V,c) for c in range(5)]

    # 1) Direct JSBSim wind semantics must agree with the validated longitudinal sign.
    gw,rows=calibrate_delta_wind_maps(root/'assets/jsbsim',models,Ts=.1)
    if gw.shape!=(5,7) or not np.isfinite(gw).all():
        raise RuntimeError('bad Gw smoke result')

    # 2) The Python/native wrapper must reproduce the stored 200/50 trim closely
    # before MPC is allowed to hide an initialization error.
    p=JSBSimPlant(root/'assets/jsbsim')
    truth=p.initialize(models[0].xref,models[0].uref,0)
    mass0=truth['mass_kg']
    init_norm=_norm(truth['x'],models[0].xref,models[0].state_scale)
    if init_norm>0.10:
        raise RuntimeError(f'JSBSim initial trim mismatch is too large: normalized={init_norm:.6f}')

    open_loop_peak=init_norm
    for _ in range(10):
        truth=p.step(models[0].uref,0,0.,.1)
        open_loop_peak=max(open_loop_peak,_norm(truth['x'],models[0].xref,models[0].state_scale))
    if open_loop_peak>0.25:
        raise RuntimeError(f'200/50 cfg0 trim cannot hold for 1 s without MPC: normalized={open_loop_peak:.6f}')

    # 3) Closed-loop QP must solve from a clean reinitialization.
    truth=p.initialize(models[0].xref,models[0].uref,0)
    c=DenseBoxMPC(models[0],100); closed_loop_peak=0.; solve_ms=[]
    for _ in range(10):
        closed_loop_peak=max(closed_loop_peak,_norm(truth['x'],models[0].xref,models[0].state_scale))
        sol=c.solve(truth['x'],np.zeros((7,100)))
        if not sol.feasible: raise RuntimeError('standalone MPC smoke QP failed')
        solve_ms.append(1000*sol.solve_time_s)
        truth=p.step(sol.u,0,0.,.1)
    if closed_loop_peak>0.25:
        raise RuntimeError(f'zero-wind closed-loop state deviated too far: normalized={closed_loop_peak:.6f}')

    # 4) One released cargo must change plant mass by about the modelled 300 kg.
    x1=p.initialize(models[1].xref,models[1].uref,1)
    mass1=x1['mass_kg']; dm=mass0-mass1
    if not (250.0 <= dm <= 350.0):
        raise RuntimeError(f'cargo mass step is not ~300 kg: {dm:.3f} kg')

    print('STANDALONE_RUNTIME_SMOKE_PASS')
    print(f'initial_trim_normalized={init_norm:.6f}')
    print(f'open_loop_1s_peak_normalized={open_loop_peak:.6f}')
    print(f'closed_loop_1s_peak_normalized={closed_loop_peak:.6f}')
    print(f'mass_drop_kg={dm:.6f}')
    print(f'qp_max_ms={max(solve_ms):.3f}')
    print("GwVa="+" ".join(f"{r['Gw_Va']:+.6f}" for r in rows))
    return 0
if __name__=='__main__': raise SystemExit(main())
