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
