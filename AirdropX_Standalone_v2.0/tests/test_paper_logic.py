from __future__ import annotations
import sys,unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from core.paper_logic import wind_confidence,smooth_step
from core.wind import WindProfile
from core.app_config import WindConfig
from core.model_bank import Vertex
from core.mpc import DenseBoxMPC

class PaperLogicTests(unittest.TestCase):
    def test_formal_wind_profiles_match_v136_samples(self):
        cases={
            'calm':[(0,0),(30,0)],
            'tailwind_5':[(4.9,0),(5,5),(20,5)],
            'headwind_12':[(4.9,0),(5,-12),(20,-12)],
            'step_bidirectional':[(4.9,0),(5,8),(14.9,8),(15,-8),(24.9,-8),(25,3)],
            'ramp_minus10_plus10':[(4.9,0),(5,-10),(15,0),(25,10),(30,10)],
        }
        for name,pts in cases.items():
            w=WindProfile(WindConfig(mode='formal',kind=name))
            for t,y in pts: self.assertAlmostEqual(w.value(t),y,places=12,msg=f'{name}@{t}')
        s=WindProfile(WindConfig(mode='formal',kind='sine_longitudinal',forcing_end_s=45,settle_ramp_s=2))
        self.assertAlmostEqual(s.value(5),2.0,12)
        self.assertAlmostEqual(s.value(10),8.0,12)
        self.assertAlmostEqual(s.value(47),0.0,12)

    def test_confidence_zero_wind_disappears(self):
        c=wind_confidence(0.0,0.0,.2)
        self.assertEqual(c['wind_effective_mps'],0.0); self.assertEqual(c['wind_rate_effective_mps2'],0.0)
        c=wind_confidence(5.0,2.0,.2)
        self.assertGreater(c['wind_confidence'],.95); self.assertGreater(c['rate_confidence'],.85)

    def test_smooth_step_endpoints(self):
        self.assertEqual(smooth_step(.5,.8,3.),0.0); self.assertEqual(smooth_step(3.,.8,3.),1.0)

    def test_recovery_reweight_keeps_terminal_P_and_dynamics(self):
        z=np.load(ROOT/'tests/data/reference_v50_developer_only.npz',allow_pickle=False); i=-1;c=0
        v=Vertex(A=z['A'][i,0,c],B=z['B'][i,0,c],Q=z['Q'][i,0,c],R=z['R'][i,0,c],P=z['P'][i,0,c],xref=z['xref'][i,0,c],uref=z['uref'][i,0,c],state_scale=z['state_scale'][i,0,c],input_scale=z['input_scale'][i,0,c],K=z['K'][i,0,c],rho=float(z['rho'][i,0,c]),N=100)
        base=DenseBoxMPC(v,100); rec=base.reweighted(np.array([.85,4,.9,.9,1.2,1,1]),np.array([.55,.85]))
        self.assertTrue(np.shares_memory(base.Phi,rec.Phi)); self.assertTrue(np.shares_memory(base.Gdist,rec.Gdist))
        self.assertLess(np.max(np.abs(base.P-rec.P)),1e-15)
        self.assertLess(np.max(np.abs(rec.Qbar[-7:,-7:]-base.P)),1e-15)
        self.assertGreater(np.max(np.abs(rec.Q-base.Q)),0)

if __name__=='__main__': unittest.main()
