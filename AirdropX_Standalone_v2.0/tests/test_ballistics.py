import unittest
from core.ballistics import predict,fractional_release

class BallisticsTests(unittest.TestCase):
    def test_fractional_release_finds_mid_sample_crossing(self):
        r=dict(x_m=0.,h_m=200.,vx_ground_mps=50.,vz_up_mps=0.,wind_est_mps=0.,wind_rate_est_mps2=0.)
        now=predict(r)['impact_x_m']
        r1=dict(r); r1['x_m']=5.
        nxt=predict(r1)['impact_x_m']
        target=.5*(now+nxt)
        s=fractional_release(r,target,.1)
        self.assertTrue(s['release_within_sample'])
        self.assertAlmostEqual(s['tau_s'],.05,places=6)
        self.assertLess(abs(s['scheduler_residual_m']),1e-6)
