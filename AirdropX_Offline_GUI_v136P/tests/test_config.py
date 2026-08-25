from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.app_config import MissionConfig, WindConfig


def test_default_backend_is_paper():
    cfg = MissionConfig()
    assert cfg.backend_mode == "v136p"
    assert cfg.validate() == []


def test_headwind_projection():
    w = WindConfig(speed_mps=8.0, direction_from_deg=0.0, flight_heading_deg=0.0)
    assert abs(w.along_track_mps() + 8.0) < 1e-12


def test_tailwind_projection():
    w = WindConfig(speed_mps=8.0, direction_from_deg=180.0, flight_heading_deg=0.0)
    assert abs(w.along_track_mps() - 8.0) < 1e-12


def test_v140_remains_optional():
    cfg = MissionConfig(backend_mode="v140")
    assert cfg.validate() == []


if __name__ == "__main__":
    test_default_backend_is_paper()
    test_headwind_projection()
    test_tailwind_projection()
    test_v140_remains_optional()
    print("test_config PASS")