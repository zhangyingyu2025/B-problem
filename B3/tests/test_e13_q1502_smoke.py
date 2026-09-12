"""Smoke checks for experimental E13-Q1502. Requires the current E13-R tree."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))

from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client
from solver_diag import DiagSolver, build_diag


class Q1502SmokeTests(unittest.TestCase):
    def test_variant_decodes_to_150m_two_diagnostic_channels(self):
        # Construct without running a case; parser is independent of the environment.
        obj = object.__new__(DiagSolver)
        # Parsing itself lives in __init__, so verify via a real one-source facade-free case.
        case = {'seed': 1, 'sources': [{'channel': 1, 'x': 400., 'y': 0., 'radius': 1200.}]}
        client = Client(OfflineTransport(case, error_mode='zero'), 'offline-test')
        client.enter()
        solver = build_diag(client, 'Q1502')
        self.assertEqual(solver.diag_r, 150.0)
        self.assertEqual(solver.diag_m, 2)
        client.exit()

    def test_three_fresh_reference_cases_all_clear(self):
        for seed in (53001400, 53001401, 53001402):
            case = make_case(seed, pattern='random')
            client = Client(OfflineTransport(case, error_mode='fixed_field'), 'offline-test')
            client.enter()
            solver = build_diag(client, 'Q1502')
            solver.run_all()
            client.exit()
            truth = {s['channel'] for s in case['sources']}
            self.assertTrue(solver.discovery_complete(), seed)
            self.assertEqual(solver.env.cleared, truth, seed)


if __name__ == '__main__':
    unittest.main()
