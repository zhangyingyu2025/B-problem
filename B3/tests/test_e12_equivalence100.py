"""Permanent extended action-equivalence regression (100 independent cases)."""
from datetime import datetime, timezone
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
from e12_risk_experiment import OUT, run_batch


class ExtendedEquivalenceTests(unittest.TestCase):
    def test_100_fresh_cases_exact_six_fallback(self):
        output = OUT/('equivalence-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))
        self.assertTrue(run_batch('equivalence', output, workers=4))


if __name__ == '__main__':
    unittest.main()
