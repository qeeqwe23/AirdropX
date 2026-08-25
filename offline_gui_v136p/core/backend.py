from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import os
import re
import shutil
import subprocess
import threading
from typing import Callable

from .app_config import MissionConfig


@dataclass
class BackendPaths:
    app_root: Path
    project_root: Path
    matlab_exe: Path


def _candidate_matlab_paths() -> list[Path]:
    out: list[Path] = []
    env = os.environ.get("AIRDROPX_MATLAB_EXE") or os.environ.get("MATLAB_EXE")
    if env:
        out.append(Path(env))
    out.extend([
        Path(r"D:\MATLAB R2026a\matlab\bin\matlab.exe"),
        Path(r"C:\Program Files\MATLAB\R2026a\bin\matlab.exe"),
        Path(r"C:\Program Files\MATLAB\R2025b\bin\matlab.exe"),
    ])
    which = shutil.which("matlab")
    if which:
        out.append(Path(which))
    return out


def find_matlab(explicit: str = "") -> Path:
    if explicit:
        p = Path(explicit)
        if p.exists():
            return p
        raise FileNotFoundError(f"指定 MATLAB 不存在: {p}")
    for p in _candidate_matlab_paths():
        if p.exists():
            return p
    raise FileNotFoundError("未找到 MATLAB。请在界面中指定 matlab.exe，或设置 AIRDROPX_MATLAB_EXE。")


def find_project_root(app_root: Path, explicit: str = "") -> Path:
    if explicit:
        p = Path(explicit).resolve()
        if (p / "matlab").is_dir():
            return p
        raise FileNotFoundError(f"项目根目录没有 matlab/ 子目录: {p}")
    candidates = [app_root, app_root.parent, Path.cwd()]
    for p in candidates:
        if (p / "matlab").is_dir() and (p / "aircraft").is_dir():
            return p.resolve()
    raise FileNotFoundError("未找到 AirdropX 项目根目录。请在界面中指定。")


def matlab_quote(value: str | Path) -> str:
    return str(value).replace("'", "''")


class MatlabBackend:
    def __init__(self, app_root: str | Path):
        self.app_root = Path(app_root).resolve()
        self.process: subprocess.Popen[str] | None = None
        self._lock = threading.Lock()

    def _entry_name(self, cfg: MissionConfig) -> str:
        return "airdropx_gui_backend_entry_v136p" if cfg.backend_mode == "v136p" else "airdropx_gui_backend_entry_v140"

    def preflight(self, cfg: MissionConfig) -> tuple[BackendPaths | None, list[str]]:
        issues: list[str] = []
        try:
            project_root = find_project_root(self.app_root, cfg.project_root)
        except Exception as exc:
            issues.append(str(exc))
            return None, issues
        try:
            matlab_exe = find_matlab(cfg.matlab_exe)
        except Exception as exc:
            issues.append(str(exc))
            return None, issues

        if cfg.backend_mode == "v136p":
            required = [
                project_root / "matlab" / "airdrop" / "airdropx_wind_airdrop_mission_v136p.m",
                project_root / "matlab" / "gui_bridge" / "airdropx_gui_backend_entry_v136p.m",
                project_root / "matlab" / "gui_bridge" / "airdropx_gui_prepare_model_v136p.m",
                project_root / "matlab" / "wind" / "airdropx_wind_profile_gui.m",
            ]
        else:
            required = [
                project_root / "matlab" / "airdrop" / "airdropx_wind_airdrop_mission_v140.m",
                project_root / "matlab" / "gui_bridge" / "airdropx_gui_backend_entry_v140.m",
                project_root / "matlab" / "gui_bridge" / "airdropx_gui_prepare_model_v140.m",
            ]
        for p in required:
            if not p.exists():
                issues.append(f"缺少集成文件: {p}")

        if cfg.model_policy == "auto_cache":
            builder = project_root / "matlab" / "phys_mpc" / "airdropx_phys_build_bank.m"
            if not builder.exists():
                issues.append(f"自由 H/V 模式缺少建模函数: {builder}")
        return BackendPaths(self.app_root, project_root, matlab_exe), issues

    def run(
        self,
        cfg: MissionConfig,
        log_cb: Callable[[str], None] | None = None,
        progress_cb: Callable[[float], None] | None = None,
    ) -> Path:
        errors = cfg.validate()
        if errors:
            raise ValueError("\n".join(errors))
        paths, issues = self.preflight(cfg)
        if issues or paths is None:
            raise RuntimeError("\n".join(issues))

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        result_dir = "offline_gui_v136p" if cfg.backend_mode == "v136p" else "offline_gui_v140"
        output_root = Path(cfg.output_root) if cfg.output_root else paths.project_root / "matlab" / "results" / result_dir / stamp
        output_root.mkdir(parents=True, exist_ok=True)
        cfg.output_root = str(output_root)
        cfg.project_root = str(paths.project_root)
        cfg.matlab_exe = str(paths.matlab_exe)
        config_path = output_root / "gui_mission_config.json"
        cfg.save(config_path)

        entry = self._entry_name(cfg)
        mcmd = (
            f"cd('{matlab_quote(paths.project_root)}'); "
            f"addpath(fullfile(pwd,'matlab','gui_bridge')); "
            f"{entry}('{matlab_quote(paths.project_root)}','{matlab_quote(config_path)}');"
        )
        cmd = [str(paths.matlab_exe), "-batch", mcmd]
        if log_cb:
            log_cb(f"[GUI] 输出目录: {output_root}")
            log_cb(f"[GUI] MATLAB: {paths.matlab_exe}")
            log_cb(f"[GUI] 后端: {cfg.backend_label}")
            log_cb(f"[GUI] H={cfg.target_altitude_m:g} m, V={cfg.target_speed_mps:g} m/s")
            log_cb(f"[GUI] Wind={cfg.wind.kind}, along={cfg.wind.along_track_mps():+.3f} m/s")
            if cfg.backend_mode == "v136p" and cfg.wind.mode == "formal":
                log_cb("[GUI] 论文预设将使用 v1.3.6-Paper 对应场景的固定传感器噪声种子。")

        with self._lock:
            self.process = subprocess.Popen(
                cmd,
                cwd=str(paths.project_root),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        assert self.process.stdout is not None
        progress_re = re.compile(r"\[GUI_PROGRESS\]\s*([0-9.]+)")
        for raw in self.process.stdout:
            line = raw.rstrip()
            if log_cb and line:
                log_cb(line)
            m = progress_re.search(line)
            if m and progress_cb:
                try:
                    progress_cb(max(0.0, min(1.0, float(m.group(1)))))
                except ValueError:
                    pass
        rc = self.process.wait()
        with self._lock:
            self.process = None
        if rc != 0:
            raise RuntimeError(f"MATLAB 后端退出，返回码 {rc}。请查看任务日志。")
        if not (output_root / "wind_airdrop_timeseries.csv").exists():
            raise RuntimeError("后端结束但没有生成 wind_airdrop_timeseries.csv。")
        if progress_cb:
            progress_cb(1.0)
        return output_root

    def stop(self) -> None:
        with self._lock:
            p = self.process
        if p is not None and p.poll() is None:
            p.terminate()
