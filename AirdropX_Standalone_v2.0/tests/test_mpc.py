import unittest
from pathlib import Path
import numpy as np
from core.model_bank import Vertex
from core.mpc import DenseBoxMPC

ROOT=Path(__file__).resolve().parents[1]

class MPCTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        z=np.load(ROOT/'tests/data/reference_v50_developer_only.npz',allow_pickle=False); idx=(-1,0,0)
        cls.v=Vertex(z['A'][idx],z['B'][idx],z['Q'][idx],z['R'][idx],z['P'][idx],z['xref'][idx],z['uref'][idx],z['state_scale'][idx],z['input_scale'][idx],z['K'][idx],float(z['rho'][idx]),int(round(float(z['N'][idx]))))
    def test_trim_returns_trim_input(self):
        c=DenseBoxMPC(self.v,100); s=c.solve(self.v.xref)
        self.assertTrue(s.feasible); self.assertLess(np.max(np.abs(s.u-self.v.uref)),2e-5)
    def test_zero_disturbance_is_equivalent(self):
        x=self.v.xref+0.02*self.v.state_scale
        a=DenseBoxMPC(self.v,100).solve(x)
        b=DenseBoxMPC(self.v,100).solve(x,np.zeros((7,100)))
        self.assertTrue(a.feasible and b.feasible); self.assertLess(np.max(np.abs(a.u-b.u)),2e-6)
