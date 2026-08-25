from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import json
import math

APP_NAME = "AirdropX 独立空投仿真软件"
APP_VERSION = "2.0.0-rc1"
CONTROLLER_NAME = "Physics-MPC v1.3.6-Paper standalone port"

VALIDATED_ALTITUDE_MIN_M = 20.0
VALIDATED_ALTITUDE_MAX_M = 200.0
VALIDATED_SPEED_MIN_MPS = 45.0
VALIDATED_SPEED_MAX_MPS = 65.0
PAPER_BASELINE_H_M = 200.0
PAPER_BASELINE_V_MPS = 50.0

FORMAL_SCENARIOS = {
    "论文-无风": "calm",
    "论文-顺风 5 m/s": "tailwind_5",
    "论文-逆风 5 m/s": "headwind_5",
    "论文-顺风 12 m/s": "tailwind_12",
    "论文-逆风 12 m/s": "headwind_12",
    "论文-双向阶跃": "step_bidirectional",
    "论文--10 到 +10 Ramp": "ramp_minus10_plus10",
    "论文-正弦纵向风": "sine_longitudinal",
}
FORMAL_SENSOR_SEEDS = {
    "calm": 101,
    "tailwind_5": 102,
    "headwind_5": 103,
    "tailwind_12": 104,
    "headwind_12": 105,
    "step_bidirectional": 106,
    "ramp_minus10_plus10": 107,
    "sine_longitudinal": 108,
}

CUSTOM_WIND_TYPES = {
    "无风": "calm",
    "恒定风": "constant",
    "阶跃风": "step",
    "双向阶跃风": "bidirectional_step",
    "渐变风 Ramp": "ramp",
    "正弦阵风": "sine",
    "复合阵风/湍流": "turbulence",
}

@dataclass
class WindConfig:
    mode: str = "custom"
    kind: str = "calm"
    speed_mps: float = 0.0
    direction_from_deg: float = 0.0
    flight_heading_deg: float = 0.0
    start_s: float = 5.0
    ramp_s: float = 10.0
    period_s: float = 20.0
    forcing_end_s: float = 45.0
    settle_ramp_s: float = 2.0
    random_seed: int = 2401

    def along_track_mps(self) -> float:
        # Meteorological FROM direction -> along-track wind, tailwind positive.
        rel = math.radians(self.direction_from_deg - self.flight_heading_deg)
        return -float(self.speed_mps) * math.cos(rel)

@dataclass
class MissionConfig:
    target_altitude_m: float = PAPER_BASELINE_H_M
    target_speed_mps: float = PAPER_BASELINE_V_MPS
    duration_s: float = 55.0
    target_start_m: float = 1200.0
    target_spacing_m: float = 80.0
    target_positions_m: list[float] | None = None
    sensor_noise_seed: int = 101
    wind: WindConfig = field(default_factory=WindConfig)
    realtime_factor: float = 1.0
    output_root: str = ""

    def drop_targets(self) -> list[float]:
        if self.target_positions_m is not None:
            return [float(x) for x in self.target_positions_m]
        return [float(self.target_start_m + i * self.target_spacing_m) for i in range(4)]

    def validate(self) -> list[str]:
        e: list[str] = []
        if not (VALIDATED_ALTITUDE_MIN_M <= self.target_altitude_m <= VALIDATED_ALTITUDE_MAX_M):
            e.append(f"目标高度仅允许 {VALIDATED_ALTITUDE_MIN_M:g}~{VALIDATED_ALTITUDE_MAX_M:g} m。")
        if not (VALIDATED_SPEED_MIN_MPS <= self.target_speed_mps <= VALIDATED_SPEED_MAX_MPS):
            e.append(f"目标速度仅允许 {VALIDATED_SPEED_MIN_MPS:g}~{VALIDATED_SPEED_MAX_MPS:g} m/s。")
        if self.duration_s < 30.0:
            e.append("任务时长不得少于 30 s。")
        if not (0.0 <= self.wind.speed_mps <= 20.0):
            e.append("自定义风速限制为 0~20 m/s。")
        if not (0.0 <= self.wind.direction_from_deg <= 360.0):
            e.append("风向必须在 0~360 度。")
        targets = self.drop_targets()
        if len(targets) != 4 or not all(math.isfinite(x) for x in targets):
            e.append("必须给出 4 个有限的投放目标位置。")
        else:
            mission_reach = float(self.target_speed_mps) * float(self.duration_s) + 600.0
            if any(x < 100.0 or x > 6500.0 for x in targets):
                e.append("投放目标位置需在 100~6500 m 的合理范围内。")
            if any(targets[i+1] - targets[i] < 20.0 for i in range(3)):
                e.append("4 个投放目标必须递增，且相邻间距至少 20 m。")
            if targets[-1] > mission_reach:
                e.append(f"最后一个目标 {targets[-1]:.0f} m 超出当前速度/时长的可达范围，建议延长任务或减小目标距离。")
        if self.wind.mode == "formal" and self.wind.kind == "sine_longitudinal":
            need = self.wind.forcing_end_s + self.wind.settle_ramp_s + 5.0
            if self.duration_s < need:
                e.append(f"论文正弦风任务时长至少 {need:g} s。")
        if self.realtime_factor not in (1.0, 2.0, 5.0):
            e.append("仿真倍率仅支持 1x / 2x / 5x。")
        return e

    @property
    def is_paper_baseline(self) -> bool:
        return abs(self.target_altitude_m - PAPER_BASELINE_H_M) < 1e-9 and abs(self.target_speed_mps - PAPER_BASELINE_V_MPS) < 1e-9

    @property
    def envelope_label(self) -> str:
        return "论文基准点" if self.is_paper_baseline else "已验证 HxV 连续包线"

    def to_dict(self) -> dict:
        d = asdict(self)
        d["drop_targets_m"] = self.drop_targets()
        d["controller"] = CONTROLLER_NAME
        d["validated_envelope"] = {
            "altitude_m": [VALIDATED_ALTITUDE_MIN_M, VALIDATED_ALTITUDE_MAX_M],
            "speed_mps": [VALIDATED_SPEED_MIN_MPS, VALIDATED_SPEED_MAX_MPS],
        }
        d["wind"]["along_track_mps"] = self.wind.along_track_mps()
        return d

    def save(self, path: str | Path) -> None:
        p = Path(path); p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
