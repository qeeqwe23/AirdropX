from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv


def _f(row: dict[str, str], key: str, default: float = float("nan")) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


@dataclass
class MissionResult:
    timeseries: list[dict[str, str]]
    cargo: list[dict[str, str]]
    summary_text: str
    output_root: Path

    def series(self, key: str) -> list[float]:
        return [_f(r, key) for r in self.timeseries]

    def metric_from_summary(self, key: str) -> str | None:
        prefix = key + "="
        for line in self.summary_text.splitlines():
            if line.startswith(prefix):
                return line[len(prefix):].strip()
        return None


def load_result(output_root: str | Path) -> MissionResult:
    root = Path(output_root)
    ts_path = root / "wind_airdrop_timeseries.csv"
    cargo_path = root / "cargo_impacts.csv"
    summary_path = root / "wind_airdrop_summary.txt"
    if not ts_path.exists():
        raise FileNotFoundError(f"缺少结果文件: {ts_path}")
    with ts_path.open("r", encoding="utf-8-sig", newline="") as f:
        ts = list(csv.DictReader(f))
    cargo: list[dict[str, str]] = []
    if cargo_path.exists():
        with cargo_path.open("r", encoding="utf-8-sig", newline="") as f:
            cargo = list(csv.DictReader(f))
    summary = summary_path.read_text(encoding="utf-8", errors="replace") if summary_path.exists() else ""
    return MissionResult(ts, cargo, summary, root)
