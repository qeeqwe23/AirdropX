from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.app_config import MissionConfig, WindConfig


def test_software_is_fixed_to_paper():
    cfg = MissionConfig()
    assert cfg.backend_mode == "v136p"
    assert cfg.backend_label == "Physics-MPC v1.3.6-Paper"
    assert cfg.validate() == []


def test_non_paper_backend_rejected():
    cfg = MissionConfig(backend_mode="other")
    assert cfg.validate()


def test_headwind_projection():
    w = WindConfig(speed_mps=8.0, direction_from_deg=0.0, flight_heading_deg=0.0)
    assert abs(w.along_track_mps() + 8.0) < 1e-12


def test_tailwind_projection():
    w = WindConfig(speed_mps=8.0, direction_from_deg=180.0, flight_heading_deg=0.0)
    assert abs(w.along_track_mps() - 8.0) < 1e-12


if __name__ == "__main__":
    test_software_is_fixed_to_paper()
    test_non_paper_backend_rejected()
    test_headwind_projection()
    test_tailwind_projection()
    print("test_config PASS")