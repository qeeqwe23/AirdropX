from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import h5py
import numpy as np

H_REQUIRED = np.arange(20.0, 200.0 + 0.1, 10.0)
V_REQUIRED = np.array([45.0, 50.0, 55.0, 60.0, 65.0])
CFG_REQUIRED = np.arange(5)


@dataclass
class BankInfo:
    path: Path
    heights: np.ndarray
    speeds: np.ndarray
    cfgs: np.ndarray
    count: int
    keys: frozenset[tuple[float, float, int]]

    @property
    def covers_release_envelope(self) -> bool:
        required = {
            (round(float(h), 8), round(float(v), 8), int(c))
            for h in H_REQUIRED for v in V_REQUIRED for c in CFG_REQUIRED
        }
        return required.issubset(self.keys)


def _scalar(ds) -> float:
    return float(np.asarray(ds[()]).reshape(-1)[0])


def _vec(ds, n: int) -> np.ndarray:
    x = np.asarray(ds[()], dtype=float).reshape(-1)
    if x.size != n:
        raise ValueError(f"expected vector of length {n}, got {x.shape}")
    return x


def _mat(ds, rows: int, cols: int) -> np.ndarray:
    x = np.asarray(ds[()], dtype=float)
    # MATLAB v7.3 arrays are exposed transposed through HDF5 for non-vector matrices.
    if x.shape == (cols, rows):
        x = x.T
    elif x.shape != (rows, cols):
        raise ValueError(f"expected {rows}x{cols}, got {x.shape}")
    return x


def inspect_bank(path: Path) -> BankInfo:
    hs, vs, cs = [], [], []
    with h5py.File(path, "r") as f:
        if "vertices" not in f:
            raise ValueError("not a Physics-MPC bank: missing vertices")
        refs = np.asarray(f["vertices"][()]).reshape(-1)
        for r in refs:
            g = f[r]
            hs.append(_scalar(g["H"]))
            vs.append(_scalar(g["V"]))
            cs.append(int(round(_scalar(g["cfgId"]))))
    keys=frozenset((round(float(h),8),round(float(v),8),int(c)) for h,v,c in zip(hs,vs,cs))
    return BankInfo(path, np.unique(hs), np.unique(vs), np.unique(cs), len(hs), keys)


def discover_bank(project_root: Path) -> BankInfo:
    candidates = []
    result_root = project_root / "matlab" / "results"
    if not result_root.is_dir():
        raise FileNotFoundError(f"MATLAB results folder not found: {result_root}")
    patterns = (
        "physics_full_envelope_bank_diagnostic.mat",
        "physics_full_envelope_bank.mat",
        "physics_bank.mat",
    )
    for pattern in patterns:
        for p in result_root.rglob(pattern):
            try:
                candidates.append(inspect_bank(p))
            except Exception:
                continue
    valid = [x for x in candidates if x.covers_release_envelope]
    if not valid:
        lines = ["No model bank covers H=20..200 m and V=45..65 m/s on the validated grid."]
        for x in sorted(candidates, key=lambda z: z.count, reverse=True)[:12]:
            lines.append(f"  {x.path} : {x.count} vertices, H={x.heights.tolist()}, V={x.speeds.tolist()}")
        raise RuntimeError("\n".join(lines))
    # Prefer the richest/newest valid bank, while keeping deterministic selection.
    valid.sort(key=lambda x: (x.count, x.path.stat().st_mtime_ns), reverse=True)
    return valid[0]

def export_bank(source: Path, output: Path, allow_partial: bool = False) -> BankInfo:
    info = inspect_bank(source)
    if not allow_partial and not info.covers_release_envelope:
        raise RuntimeError(
            f"Bank does not cover release envelope: {source}\n"
            f"vertices={info.count}, H={info.heights.tolist()}, V={info.speeds.tolist()}"
        )
    # Release software intentionally exports only the validated v0.9 anchor grid,
    # even if a research bank contains extra points outside that interval.
    H = np.array(H_REQUIRED, dtype=float) if not allow_partial else np.sort(info.heights)
    V = np.array(V_REQUIRED, dtype=float) if not allow_partial else np.sort(info.speeds)
    cfgs = np.array([0, 1, 2, 3, 4], dtype=int)
    nh, nv, nc = len(H), len(V), len(cfgs)

    A = np.full((nh, nv, nc, 7, 7), np.nan)
    B = np.full((nh, nv, nc, 7, 2), np.nan)
    Q = np.full((nh, nv, nc, 7, 7), np.nan)
    R = np.full((nh, nv, nc, 2, 2), np.nan)
    P = np.full((nh, nv, nc, 7, 7), np.nan)
    K = np.full((nh, nv, nc, 2, 7), np.nan)
    xref = np.full((nh, nv, nc, 7), np.nan)
    uref = np.full((nh, nv, nc, 2), np.nan)
    ss = np.full((nh, nv, nc, 7), np.nan)
    ins = np.full((nh, nv, nc, 2), np.nan)
    rho = np.full((nh, nv, nc), np.nan)
    N = np.full((nh, nv, nc), np.nan)

    index_h = {round(float(x), 8): i for i, x in enumerate(H)}
    index_v = {round(float(x), 8): j for j, x in enumerate(V)}
    with h5py.File(source, "r") as f:
        refs = np.asarray(f["vertices"][()]).reshape(-1)
        for r in refs:
            g = f[r]
            h = _scalar(g["H"]); v = _scalar(g["V"]); c = int(round(_scalar(g["cfgId"])))
            hk=round(h,8); vk=round(v,8)
            if c not in cfgs or hk not in index_h or vk not in index_v:
                continue
            i = index_h[hk]; j = index_v[vk]
            A[i,j,c] = _mat(g["lin"]["Ad"], 7, 7)
            B[i,j,c] = _mat(g["lin"]["Bd"], 7, 2)
            Q[i,j,c] = _mat(g["Q"], 7, 7)
            R[i,j,c] = _mat(g["R"], 2, 2)
            P[i,j,c] = _mat(g["terminal"]["P"], 7, 7)
            K[i,j,c] = _mat(g["terminal"]["K"], 2, 7)
            xref[i,j,c] = _vec(g["trim"]["x"], 7)
            uref[i,j,c] = _vec(g["trim"]["u"], 2)
            ss[i,j,c] = _vec(g["qrMeta"]["StateScale"], 7)
            ins[i,j,c] = _vec(g["qrMeta"]["InputScale"], 2)
            rho[i,j,c] = _scalar(g["terminal"]["rho"])
            N[i,j,c] = 100.0

    required = (A,B,Q,R,P,K,xref,uref,ss,ins,rho,N)
    if not allow_partial and any(not np.isfinite(x).all() for x in required):
        raise RuntimeError("Selected bank has missing/non-finite grid entries after export")

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        format_version=np.array([2], dtype=np.int32),
        source_bank_name=np.array([source.name]),
        heights=H, speeds=V, cfgs=cfgs,
        A=A, B=B, Q=Q, R=R, P=P, K=K,
        xref=xref, uref=uref, state_scale=ss, input_scale=ins,
        rho=rho, N=N,
    )
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description="Export MATLAB v7.3 Physics-MPC bank to standalone NPZ.")
    ap.add_argument("--project-root", type=Path)
    ap.add_argument("--input", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--allow-partial", action="store_true", help="Developer/test only; not accepted by the released app.")
    ns = ap.parse_args()
    if bool(ns.project_root) == bool(ns.input):
        ap.error("pass exactly one of --project-root or --input")
    info = discover_bank(ns.project_root.resolve()) if ns.project_root else inspect_bank(ns.input.resolve())
    export_bank(info.path, ns.output.resolve(), allow_partial=ns.allow_partial)
    print(f"EXPORTED {info.path}")
    print(f"vertices={info.count} H={info.heights.tolist()} V={info.speeds.tolist()}")
    print(f"output={ns.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



