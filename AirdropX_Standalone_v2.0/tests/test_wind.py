import unittest
import numpy as np
from core.wind import wind_increment_preview

class WindTests(unittest.TestCase):
    def test_constant_wind_has_no_persistent_carrier_injection(self):
        self.assertTrue(np.allclose(wind_increment_preview(12.0,0.0,0.2,100,0.1),0.0))
    def test_rate_is_causal_and_decays(self):
        d=wind_increment_preview(0.0,2.0,0.0,100,0.1)
        self.assertGreater(d[0],0); self.assertGreater(d[0],d[20]); self.assertTrue(np.isfinite(d).all())
