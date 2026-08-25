import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class ExportReferenceTests(unittest.TestCase):
    def test_v50_reference_has_known_verified_trim(self):
        z=np.load(ROOT/'tests/data/reference_v50_developer_only.npz',allow_pickle=False)
        self.assertEqual(z['A'].shape,(19,1,5,7,7))
        self.assertEqual(z['B'].shape,(19,1,5,7,2))
        self.assertEqual(z['K'].shape,(19,1,5,2,7))
        x=z['xref'][-1,0,0]; u=z['uref'][-1,0,0]
        self.assertAlmostEqual(x[0],200.0,8); self.assertAlmostEqual(x[1],50.0,8)
        self.assertAlmostEqual(x[3],0.110261472,7)
        self.assertAlmostEqual(x[5],73.171502,5); self.assertAlmostEqual(x[6],84.669430,5)
        self.assertAlmostEqual(u[0],-0.33817557,7); self.assertAlmostEqual(u[1],0.61673575,7)

class RuntimeBankGuardTests(unittest.TestCase):
    def test_partial_v50_reference_is_rejected_by_runtime(self):
        from core.model_bank import ModelBank
        with self.assertRaises(RuntimeError):
            ModelBank(ROOT/'tests/data/reference_v50_developer_only.npz')
