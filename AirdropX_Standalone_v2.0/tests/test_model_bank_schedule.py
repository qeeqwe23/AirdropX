from __future__ import annotations
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from core.model_bank import ModelBank


class ModelBankScheduleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref=ROOT/'tests/data/reference_v50_developer_only.npz'

    def test_exact_anchor_recomputed_dare_matches_saved_terminal(self):
        # Developer reference is intentionally V=50 only, so construct a tiny
        # test-only view by monkey-patching the validated speed envelope guard.
        z=np.load(self.ref,allow_pickle=False)
        i=int(np.where(np.isclose(z['heights'],200.0))[0][0]); c=0
        A=z['A'][i,0,c]; B=z['B'][i,0,c]; Q=z['Q'][i,0,c]; R=z['R'][i,0,c]
        P=solve_discrete_are(A,B,Q,R)
        K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
        self.assertLess(float(np.max(np.abs(P-z['P'][i,0,c]))),1e-7)
        self.assertLess(float(np.max(np.abs(K-z['K'][i,0,c]))),1e-7)
        rho=max(abs(np.linalg.eigvals(A-B@K)))
        self.assertAlmostEqual(float(rho),float(z['rho'][i,0,c]),places=9)

    def test_v090_rule_is_encoded_in_source(self):
        text=(ROOT/'core/model_bank.py').read_text(encoding='utf-8')
        self.assertIn('bilinear interpolation of A/B/xtrim/utrim',text)
        self.assertIn('solve_discrete_are',text)
        self.assertIn('Np=Nc=100',text)
        # Regression guard: terminal P/K must not be bilinearly interpolated.
        body=text.split('def vertex',1)[1]
        self.assertNotIn('self._bilinear(self.P',body)
        self.assertNotIn('self._bilinear(self.K',body)

if __name__=='__main__': unittest.main()
