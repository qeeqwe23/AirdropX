import unittest
from core.app_config import MissionConfig,VALIDATED_ALTITUDE_MIN_M,VALIDATED_ALTITUDE_MAX_M,VALIDATED_SPEED_MIN_MPS,VALIDATED_SPEED_MAX_MPS

class ConfigTests(unittest.TestCase):
    def test_boundaries_are_allowed(self):
        for h in (VALIDATED_ALTITUDE_MIN_M,VALIDATED_ALTITUDE_MAX_M):
            for v in (VALIDATED_SPEED_MIN_MPS,VALIDATED_SPEED_MAX_MPS):
                self.assertEqual(MissionConfig(target_altitude_m=h,target_speed_mps=v).validate(),[])
    def test_outside_envelope_is_blocked(self):
        self.assertTrue(MissionConfig(target_altitude_m=19.999,target_speed_mps=50).validate())
        self.assertTrue(MissionConfig(target_altitude_m=200.001,target_speed_mps=50).validate())
        self.assertTrue(MissionConfig(target_altitude_m=100,target_speed_mps=44.999).validate())
        self.assertTrue(MissionConfig(target_altitude_m=100,target_speed_mps=65.001).validate())
    def test_custom_drop_targets_are_used(self):
        cfg=MissionConfig(target_positions_m=[900,980,1130,1210])
        self.assertEqual(cfg.drop_targets(),[900.0,980.0,1130.0,1210.0])
        self.assertEqual(cfg.validate(),[])
    def test_custom_drop_targets_are_guarded(self):
        self.assertTrue(MissionConfig(target_positions_m=[900,980,990,1210]).validate())
        self.assertTrue(MissionConfig(target_positions_m=[90,980,1130,1210]).validate())
        self.assertTrue(MissionConfig(duration_s=30,target_speed_mps=45,target_positions_m=[900,980,1130,2600]).validate())
