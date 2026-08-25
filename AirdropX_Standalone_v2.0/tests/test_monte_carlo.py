import math
import unittest
from core.simulation import StandaloneSimulation

class MonteCarloImpactTests(unittest.TestCase):
    def test_impact_samples_are_finite_xy_points(self):
        state=dict(x_m=950.0,h_m=196.0,vx_ground_mps=50.0,vz_up_mps=0.0,wind_est_mps=5.0,wind_rate_est_mps2=0.0)
        samples=StandaloneSimulation._impact_mc_samples(state,0.25,1234,n=16)
        self.assertEqual(len(samples),16)
        xs=[p['x_m'] for p in samples]
        ys=[p['y_m'] for p in samples]
        self.assertTrue(all(math.isfinite(x) for x in xs))
        self.assertTrue(all(math.isfinite(y) for y in ys))
        self.assertGreater(max(xs)-min(xs),0.01)
        self.assertGreater(max(ys)-min(ys),0.01)

if __name__=='__main__': unittest.main()