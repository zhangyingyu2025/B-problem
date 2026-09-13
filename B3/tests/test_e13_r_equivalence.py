"""Permanent check that retained R reproduces its frozen 100-case action traces."""
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
from solver import build_e13
from offline_environment import OfflineTransport
from protocol_adapter import Client
from e12_risk_experiment import canonical, verify_protected

FROZEN = ROOT/'B3/结果/e13_OR_16'


def compare_case(seed):
    case = json.loads((FROZEN/f'{seed}-case.json').read_text(encoding='utf-8'))
    saved = json.loads((FROZEN/f'{seed}-R-result.json').read_text(encoding='utf-8'))
    old = [json.loads(x) for x in (ROOT/saved['action_log']).read_text(encoding='utf-8').splitlines()]
    client = Client(OfflineTransport(case, error_mode='fixed_field'), 'offline-test')
    client.enter()
    engine = build_e13(client)
    engine.run_all()
    client.exit()
    metrics = {**client.stats, 'virtual_time_s': client.virtual_time_s}
    return {'seed': seed, 'events_equal': canonical(old) == canonical(client.events),
            'metrics_equal': metrics == saved['metrics'],
            'all_cleared': engine.env.cleared == {s['channel'] for s in case['sources']}}


class RetainedRTests(unittest.TestCase):
    @unittest.skipUnless((FROZEN/'52001500-case.json').is_file(), 'Old frozen raw dataset removed by requested 2026-09-13 cleanup; not a passing equivalence result')
    def test_saved_100_cases_remain_exactly_equivalent(self):
        verify_protected()
        with ProcessPoolExecutor(max_workers=4) as pool:
            rows = list(pool.map(compare_case, range(52001500, 52001600)))
        failed = [r for r in rows if not (r['events_equal'] and r['metrics_equal'] and r['all_cleared'])]
        self.assertEqual(failed, [])
        verify_protected()


if __name__ == '__main__':
    unittest.main()
