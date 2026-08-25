from __future__ import annotations

import sys
import numpy as np
from pathlib import Path
from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


def app_root() -> Path:
    if getattr(sys, "frozen", False):
        # PyInstaller 6 onedir keeps bundled data under sys._MEIPASS (normally _internal).
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).resolve().parent)).resolve()
    return Path(__file__).resolve().parent


def _norm(x, ref, scale) -> float:
    from core.mpc import DenseBoxMPC

    e = DenseBoxMPC.state_error(np.asarray(x, float), np.asarray(ref, float))
    return float(np.max(np.abs(e) / np.maximum(np.asarray(scale, float), 1e-12)))


def run_runtime_smoke(root: Path) -> int:
    from core.model_bank import ModelBank
    from core.mpc import DenseBoxMPC
    from core.jsbsim_runtime import JSBSimPlant, calibrate_delta_wind_maps

    bank = ModelBank(root / "assets/controller/controller_bank.npz")
    models = [bank.vertex(200.0, 50.0, c) for c in range(5)]
    gw, rows = calibrate_delta_wind_maps(root / "assets/jsbsim", models, Ts=0.1)
    if gw.shape != (5, 7) or not np.isfinite(gw).all():
        raise RuntimeError("bad Gw smoke result")

    plant = JSBSimPlant(root / "assets/jsbsim")
    truth = plant.initialize(models[0].xref, models[0].uref, 0)
    mass0 = truth["mass_kg"]
    init_norm = _norm(truth["x"], models[0].xref, models[0].state_scale)
    if init_norm > 0.10:
        raise RuntimeError(f"JSBSim initial trim mismatch is too large: normalized={init_norm:.6f}")

    ctrl = DenseBoxMPC(models[0], 100)
    closed_loop_peak = init_norm
    solve_ms = []
    for _ in range(10):
        sol = ctrl.solve(truth["x"], np.zeros((7, 100)))
        if not sol.feasible:
            raise RuntimeError("standalone MPC smoke QP failed")
        solve_ms.append(1000.0 * sol.solve_time_s)
        truth = plant.step(sol.u, 0, 0.0, 0.1)
        closed_loop_peak = max(closed_loop_peak, _norm(truth["x"], models[0].xref, models[0].state_scale))
    if closed_loop_peak > 0.25:
        raise RuntimeError(f"zero-wind closed-loop state deviated too far: normalized={closed_loop_peak:.6f}")

    x1 = plant.initialize(models[1].xref, models[1].uref, 1)
    dm = mass0 - x1["mass_kg"]
    if not (250.0 <= dm <= 350.0):
        raise RuntimeError(f"cargo mass step is not about 300 kg: {dm:.3f} kg")

    print("AIRDROPX_STANDALONE_EXE_SMOKE_PASS")
    print(f"initial_trim_normalized={init_norm:.6f}")
    print(f"closed_loop_1s_peak_normalized={closed_loop_peak:.6f}")
    print(f"mass_drop_kg={dm:.6f}")
    print(f"qp_max_ms={max(solve_ms):.3f}")
    print("GwVa=" + " ".join(f"{r['Gw_Va']:+.6f}" for r in rows))
    return 0


if __name__ == "__main__":
    if "--smoke-runtime" in sys.argv:
        raise SystemExit(run_runtime_smoke(app_root()))
    app = QApplication(sys.argv)
    app.setApplicationName("AirdropX Standalone")
    w = MainWindow(app_root())
    w.show()
    raise SystemExit(app.exec())
