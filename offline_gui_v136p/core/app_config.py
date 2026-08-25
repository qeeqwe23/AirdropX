from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
import json
import math

BACKEND_OPTIONS = {
    "Physics-MPC v1.3.6-Paper（默认/论文）": "v136p",
    "Physics-MPC v1.4.0（实验）": "v140",
}

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
    mode: str = "custom"  # custom | formal
    kind: str = "calm"
    speed_mps: float = 0.0
    direction_from_deg: float = 0.0
    flight_heading_deg: float = 0.0
    start_s: float = 5.0
    ramp_s: float = 10.0
    period_s: float = 20.0
    forcing_end_s: float = 45.0
    settle_ramp_s: float = 2.0

    def along_track_mps(self) -> float:
        """Project meteorological wind-from direction onto the flight track.

        Positive is tailwind and negative is headwind, matching the longitudinal
        wind convention used by the v1.3.6/v1.4.0 AirdropX missions.
        """
        rel = math.radians(self.direction_from_deg - self.flight_heading_deg)
        return -float(self.speed_mps) * math.cos(rel)


@dataclass
class MissionConfig:
    backend_mode: str = "v136p"  # v136p | v140
    target_altitude_m: float = 200.0
    target_speed_mps: float = 50.0
    fuel_scale: float = 1.0
    duration_s: float = 55.0
    target_start_m: float = 1200.0
    target_spacing_m: float = 80.0
    sensor_noise_seed: int = 1
    wind: WindConfig = field(default_factory=WindConfig)
    project_root: str = ""
    matlab_exe: str = ""
    output_root: str = ""
    model_policy: str = "auto_cache"  # auto_cache | certified_only

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.backend_mode not in {"v136p", "v140"}:
            errors.append("后端必须是 v1.3.6-Paper 或 v1.4.0。")
        if not (5.0 <= self.target_altitude_m <= 10000.0):
            errors.append("目标高度必须在 5~10000 m 之间。")
        if not (10.0 <= self.target_speed_mps <= 150.0):
            errors.append("目标速度必须在 10~150 m/s 之间。")
        if not (0.0 <= self.wind.speed_mps <= 60.0):
            errors.append("风速必须在 0~60 m/s 之间。")
        if not (0.0 <= self.wind.direction_from_deg <= 360.0):
            errors.append("风向必须在 0~360 度之间。")
        if self.duration_s < 30.0:
            errors.append("任务时长必须不少于 30 s。")
        if self.wind.mode == "formal" and self.wind.kind == "sine_longitudinal":
            min_sine_duration = self.wind.forcing_end_s + self.wind.settle_ramp_s + 5.0
            if self.duration_s < min_sine_duration:
                errors.append(f"论文正弦风任务时长必须不少于 {min_sine_duration:g} s。")
        if not (0.0 <= self.fuel_scale <= 1.2):
            errors.append("FuelScale 必须在 0~1.2 之间。")
        return errors

    @property
    def backend_label(self) -> str:
        return "Physics-MPC v1.3.6-Paper" if self.backend_mode == "v136p" else "Physics-MPC v1.4.0"

    def to_dict(self) -> dict:
        d = asdict(self)
        d["wind"]["along_track_mps"] = self.wind.along_track_mps()
        return d

    def save(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "MissionConfig":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        wind_raw = raw.pop("wind", {})
        wind_raw.pop("along_track_mps", None)
        raw["wind"] = WindConfig(**wind_raw)
        return cls(**raw)
