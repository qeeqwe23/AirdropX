import math
import unittest
from core.simulation import StandaloneSimulation

class MonteCarloImpactTests(unittest.TestCase):
    def test_impact_samples_are_finite(self):
        state=dict(x_m=950.0,h_m=196.0,vx_ground_mps=50.0,vz_up_mps=0.0,wind_est_mps=5.0,wind_rate_est_mps2=0.0)
        samples=StandaloneSimulation._impact_mc_samples(state,0.25,1234,n=16)
        self.assertEqual(len(samples),16)
        self.assertTrue(all(math.isfinite(x) for x in samples))
        self.assertGreater(max(samples)-min(samples),0.01)

if __name__=='__main__': unittest.main()
