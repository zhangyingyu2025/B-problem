import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'code'))
from e12_risk_report import quantile, tail, pair_summary
from e12_risk_experiment import definitions, selector_for


class RiskStatisticsTests(unittest.TestCase):
    def test_linear_quantile_and_cvar(self):
        self.assertEqual(quantile([0, 10], .95), 9.5)
        self.assertEqual(quantile([7], .99), 7)
        self.assertEqual(tail(list(range(200)))['CVaR95'], 194.5)
        self.assertEqual(tail(list(range(21)))['tail_count'], 2)
        self.assertIsNone(quantile([], .95))

    def test_fixed_splits_and_selectors(self):
        eq, dev = definitions('equivalence'), definitions('development')
        self.assertEqual(len(eq), 100); self.assertEqual(len(dev), 200)
        self.assertEqual({x['case']['seed'] for x in dev}, set(range(27182800, 27183000)))
        self.assertEqual(len({x['group'] for x in eq}), 5)
        self.assertFalse({x['case']['seed'] for x in eq} & {x['case']['seed'] for x in dev})
        for t in (2, 4, 6):
            for k in range(17):
                self.assertEqual(selector_for('T'+str(t))(k).n, 7 if k >= t else 6)

    def test_paired_ties_not_wins(self):
        rows = [{'seed': i, 'qualified_pair': True, 'delta_time_per_source_s': d,
                 'delta_movement_m': 0, 'delta_measurements': 0, 'delta_switches': 0, 'delta_clear_failures': 0}
                for i, d in enumerate((-1, 0, 2, 3))]
        result = pair_summary(rows)
        self.assertEqual(result['win_rate'], .25)
        self.assertEqual(result['tie_rate'], .25)
        self.assertEqual(result['worst_seed'], 3)


if __name__ == '__main__':
    unittest.main()
