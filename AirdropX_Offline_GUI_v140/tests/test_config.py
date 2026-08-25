from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.app_config import MissionConfig, WindConfig

def main():
    # North wind while flying north => headwind (negative in v1.4.0 convention).
    w=WindConfig(speed_mps=8,direction_from_deg=0,flight_heading_deg=0)
    assert abs(w.along_track_mps()+8)<1e-9
    w.direction_from_deg=180
    assert abs(w.along_track_mps()-8)<1e-9
    c=MissionConfig(target_altitude_m=200,target_speed_mps=50,wind=w)
    assert not c.validate()
    print("config tests PASS")
if __name__=='__main__': main()
