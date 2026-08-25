from __future__ import annotations
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from core.app_config import FORMAL_SCENARIOS,FORMAL_SENSOR_SEEDS

class ReleaseGuardTests(unittest.TestCase):
    def test_formal_scenarios_use_frozen_paper_seeds(self):
        self.assertEqual(set(FORMAL_SCENARIOS.values()),set(FORMAL_SENSOR_SEEDS))
        self.assertEqual([FORMAL_SENSOR_SEEDS[x] for x in FORMAL_SCENARIOS.values()],list(range(101,109)))

    def test_exporter_requires_full_cartesian_anchor_grid(self):
        from tools.export_controller_bank import BankInfo,H_REQUIRED,V_REQUIRED,CFG_REQUIRED
        keys={(round(float(h),8),round(float(v),8),int(c)) for h in H_REQUIRED for v in V_REQUIRED for c in CFG_REQUIRED}
        info=BankInfo(Path('dummy'),H_REQUIRED,V_REQUIRED,CFG_REQUIRED,len(keys),frozenset(keys))
        self.assertTrue(info.covers_release_envelope)
        # Same count and same unique axis coverage can still be malformed if one
        # required combination is missing and another unrelated key replaces it.
        bad=set(keys); bad.remove(next(iter(bad))); bad.add((999.0,999.0,0))
        malformed=BankInfo(Path('dummy'),H_REQUIRED,V_REQUIRED,CFG_REQUIRED,len(bad),frozenset(bad))
        self.assertFalse(malformed.covers_release_envelope)

if __name__=='__main__': unittest.main()
