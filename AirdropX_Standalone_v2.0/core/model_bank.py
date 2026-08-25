from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are

from .app_config import (
    VALIDATED_ALTITUDE_MIN_M,
    VALIDATED_ALTITUDE_MAX_M,
    VALIDATED_SPEED_MIN_MPS,
    VALIDATED_SPEED_MAX_MPS,
)


@dataclass
class Vertex:
    A: np.ndarray
    B: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    P: np.ndarray
    xref: np.ndarray
    uref: np.ndarray
    state_scale: np.ndarray
    input_scale: np.ndarray
    K: np.ndarray
    rho: float
    N: int


class ModelBank:
    """Runtime view of the validated HxV Physics-MPC anchor bank.

    Off-grid scheduling follows the already-validated v0.9.0 rule:
      1) bilinear interpolation of A/B/xtrim/utrim over the four HxV anchors;
      2) common Q/R and Bryson scales are not interpolated/tuned;
      3) terminal P/K are recomputed from the interpolated A/B with DARE;
      4) Np=Nc=100 is fixed.
    """

    def __init__(self, path: str | Path):
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"Standalone controller asset missing: {p}")
        z = np.load(p, allow_pickle=False)
        self.H = np.asarray(z["heights"], float)
        self.V = np.asarray(z["speeds"], float)
        self.A = np.asarray(z["A"], float)
        self.B = np.asarray(z["B"], float)
        self.Q = np.asarray(z["Q"], float)
        self.R = np.asarray(z["R"], float)
        self.P = np.asarray(z["P"], float)
        self.xr = np.asarray(z["xref"], float)
        self.ur = np.asarray(z["uref"], float)
        self.ss = np.asarray(z["state_scale"], float)
        self.ins = np.asarray(z["input_scale"], float)
        self.K = np.asarray(z["K"], float)
        self.rho = np.asarray(z["rho"], float)
        self.N = np.asarray(z["N"], float)

        if self.A.shape[:3] != (len(self.H), len(self.V), 5) or self.A.shape[-2:] != (7, 7) or self.B.shape[-2:] != (7, 2):
            raise RuntimeError(f"Malformed controller bank dimensions: A={self.A.shape}, B={self.B.shape}")
        for name, arr in [
            ("A", self.A), ("B", self.B), ("Q", self.Q), ("R", self.R), ("P", self.P),
            ("xref", self.xr), ("uref", self.ur), ("state_scale", self.ss), ("input_scale", self.ins),
        ]:
            if not np.isfinite(arr).all():
                raise RuntimeError(f"Controller bank contains non-finite {name}")
        if (
            self.H.min() > VALIDATED_ALTITUDE_MIN_M + 1e-9
            or self.H.max() < VALIDATED_ALTITUDE_MAX_M - 1e-9
            or self.V.min() > VALIDATED_SPEED_MIN_MPS + 1e-9
            or self.V.max() < VALIDATED_SPEED_MAX_MPS - 1e-9
        ):
            raise RuntimeError(
                f"Controller bank does not cover validated envelope: H={self.H.min()}..{self.H.max()}, V={self.V.min()}..{self.V.max()}"
            )

        # The validated controller is one common Q/R/scaling kernel. Refuse a
        # malformed asset rather than silently creating cfg/H/V-specific tuning.
        self.common_Q = self.Q[0, 0, 0].copy()
        self.common_R = self.R[0, 0, 0].copy()
        self.common_ss = self.ss[0, 0, 0].copy()
        self.common_ins = self.ins[0, 0, 0].copy()
        for name, arr, ref in [
            ("Q", self.Q, self.common_Q),
            ("R", self.R, self.common_R),
            ("state_scale", self.ss, self.common_ss),
            ("input_scale", self.ins, self.common_ins),
        ]:
            if float(np.max(np.abs(arr - ref))) > 1e-10:
                raise RuntimeError(f"Controller bank is not one unified common kernel: {name} differs across anchors")
        if not np.allclose(self.N, 100.0, atol=0.0, rtol=0.0):
            raise RuntimeError("Standalone release requires the validated fixed Np=Nc=100 horizon")

    @staticmethod
    def _bracket(grid: np.ndarray, val: float) -> tuple[int, int]:
        if val < grid[0] - 1e-9 or val > grid[-1] + 1e-9:
            raise ValueError("request outside model bank")
        hi = int(np.searchsorted(grid, val, side="right"))
        hi = min(max(hi, 1), len(grid) - 1)
        lo = hi - 1
        if abs(val - grid[lo]) < 1e-12:
            hi = lo
        if abs(val - grid[hi]) < 1e-12:
            lo = hi
        return lo, hi

    @staticmethod
    def _dare_terminal(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        P = solve_discrete_are(A, B, Q, R)
        # MATLAB bank stores K for u = -K*x, so closed loop is A-B*K.
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        rho = float(np.max(np.abs(np.linalg.eigvals(A - B @ K))))
        if not np.isfinite(P).all() or not np.isfinite(K).all() or not np.isfinite(rho) or rho >= 1.0:
            raise RuntimeError(f"Interpolated DARE terminal solution is not stabilizing (rho={rho})")
        return np.asarray(P, float), np.asarray(K, float), rho

    def _bilinear(self, arr: np.ndarray, i0: int, i1: int, j0: int, j1: int, c: int, ah: float, av: float) -> np.ndarray:
        x00 = arr[i0, j0, c]
        x10 = arr[i1, j0, c]
        x01 = arr[i0, j1, c]
        x11 = arr[i1, j1, c]
        return (1 - ah) * (1 - av) * x00 + ah * (1 - av) * x10 + (1 - ah) * av * x01 + ah * av * x11

    def vertex(self, H: float, V: float, cfg: int) -> Vertex:
        i0, i1 = self._bracket(self.H, float(H))
        j0, j1 = self._bracket(self.V, float(V))
        c = int(cfg)
        if not 0 <= c <= 4:
            raise ValueError("cfg must be 0..4")
        ah = 0.0 if i0 == i1 else (float(H) - self.H[i0]) / (self.H[i1] - self.H[i0])
        av = 0.0 if j0 == j1 else (float(V) - self.V[j0]) / (self.V[j1] - self.V[j0])

        A = self._bilinear(self.A, i0, i1, j0, j1, c, ah, av)
        B = self._bilinear(self.B, i0, i1, j0, j1, c, ah, av)
        xref = self._bilinear(self.xr, i0, i1, j0, j1, c, ah, av)
        uref = self._bilinear(self.ur, i0, i1, j0, j1, c, ah, av)
        P, K, rho = self._dare_terminal(A, B, self.common_Q, self.common_R)
        return Vertex(
            A=A,
            B=B,
            Q=self.common_Q.copy(),
            R=self.common_R.copy(),
            P=P,
            xref=xref,
            uref=uref,
            state_scale=self.common_ss.copy(),
            input_scale=self.common_ins.copy(),
            K=K,
            rho=rho,
            N=100,
        )
