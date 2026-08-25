import unittest
from core.app_config import WindConfig
from core.wind import wind_event_markers

class WindEventMarkerTests(unittest.TestCase):
    def test_formal_tailwind_step_marks_appearance(self):
        events=wind_event_markers(WindConfig(mode='formal',kind='tailwind_5'),75.0)
        self.assertIn((5.0,'风出现 +5 m/s'),events)

    def test_formal_bidirectional_marks_all_jumps(self):
        events=wind_event_markers(WindConfig(mode='formal',kind='step_bidirectional'),75.0)
        self.assertEqual([t for t,_ in events],[5.0,15.0,25.0])

    def test_sine_marks_disappearance(self):
        events=wind_event_markers(WindConfig(mode='formal',kind='sine_longitudinal',forcing_end_s=45.0,settle_ramp_s=2.0),75.0)
        self.assertIn((47.0,'风消失 0 m/s'),events)

if __name__ == '__main__':
    unittest.main()
