"""Phase A correctness tests, not an E12 performance experiment."""
import copy
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
from coverage_templates import CoverageTemplate, E11_TEMPLATE, SEVEN_TEMPLATE, default_selector
from e12_coverage_hook import E11, E12Solver, build_offline_solver
from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client

OUT = ROOT/'B3/结果/e12_phase_a'
TRACE = ROOT/'B3/结果/offline_logs'/('e12-equivalence-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))


def canonical(events):
    """Remove only request/session identity and wall-clock metadata; exact equality."""
    ignored = {'request_id', 'session_id', 'wall_elapsed_s', 'real_timestamp_ms'}
    def clean(x):
        if isinstance(x, dict):
            return {k: clean(v) for k, v in x.items() if k not in ignored}
        if isinstance(x, (tuple, list)):
            return [clean(v) for v in x]
        return x
    return clean(events)


def execute(case, label, baseline=False, selector=None):
    transport = OfflineTransport(case, error_mode='endpoints' if case['pattern'] == 'boundary' else 'fixed_field')
    client = Client(transport, 'offline-test', TRACE/(label+'.jsonl'), session_id=label)
    client.enter()
    solver = build_offline_solver(client, baseline=baseline, selector=selector)
    solver.run_all()
    client.exit()
    assert solver.discovery_complete()
    assert len(transport.cleared) == len(case['sources'])
    assert set(solver.tracks) == transport.cleared
    return solver, client, canonical(client.events)


class TemplateTests(unittest.TestCase):
    def test_analytic_endpoints(self):
        c = SEVEN_TEMPLATE.certificate()
        self.assertAlmostEqual(c['d1000_m'], 446.181, places=3)
        self.assertAlmostEqual(c['d1800_m'], 995.148, places=3)
        self.assertGreater(c['reception_margin_m'], 4.8)
        self.assertAlmostEqual(c['open_route_m'], 6237.637893, places=3)

    def test_all_rotations_midlines_and_random(self):
        rng = random.Random(1200701)
        for deg in SEVEN_TEMPLATE.rotation_candidates:
            sites = [(0., 0.)]+SEVEN_TEMPLATE.outer_sites(deg)
            points = []
            for r in (1000, 1800):
                for k in range(7):
                    a = math.radians(deg+(k+.5)*360/7)
                    points.append((r*math.cos(a), r*math.sin(a)))
            for _ in range(1000):
                r, a = 1800*math.sqrt(rng.random()), rng.random()*2*math.pi
                points.append((r*math.cos(a), r*math.sin(a)))
            self.assertLessEqual(max(min(math.dist(p, q) for q in sites) for p in points), 1000)

    def test_invalid_and_unsafe_templates(self):
        for args in ((5, 1005, (0,)), (8, 945, (0,)), (7, 900, (0,)),
                     (7, float('nan'), (0,)), (7, 1005, ()), (7, 1005, (0, 0)),
                     (7, 1005, (60,)), (6, 1130, (0,)), (6, 1129, tuple(range(0, 60, 5)))):
            with self.subTest(args=args), self.assertRaises(ValueError):
                CoverageTemplate(*args)

    def test_default_selector_and_shared_methods(self):
        for k0 in range(17):
            self.assertEqual(default_selector(k0), E11_TEMPLATE)
        for method in ('run_search', 'run_all', 'joint_phase', 'finish', 'transit_measure',
                       'local_clear_from_mec', 'safe_bank', 'discovery_complete'):
            self.assertIs(getattr(E12Solver, method), getattr(E11.E11Solver, method))

    def test_eighth_station_is_required(self):
        c = Client(OfflineTransport(make_case(1200702)), 'offline-test')
        solver = build_offline_solver(c)
        solver.coverage_template = SEVEN_TEMPLATE
        solver.sites = [(0., 0.)]+SEVEN_TEMPLATE.outer_sites(0)
        solver.checked = {ch: set(range(7)) for ch in range(1, 21)}
        self.assertFalse(solver.discovery_complete())
        calls = []
        def absent(p, ch, site_idx=None, reason=''):
            calls.append(ch)
            solver.handle(p, ch, {'measure_result': 'no_signal'}, site_idx, reason)
        solver.measure = absent
        solver.scan_site(7, solver.sites[7])
        self.assertEqual(len(calls), 20)
        self.assertTrue(solver.discovery_complete())

    def test_five_outer_sites_impossible(self):
        self.assertGreater(1800*math.sin(math.pi/5), 1000)

    def test_seven_rational_upper_bound(self):
        # pi < 22/7; alternating cosine series gives a strict rational lower bound.
        x = Fraction(22, 49)
        cos_lower = 1-x*x/2+x**4/24-x**6/720
        for radius in (1000, 1800):
            upper_squared = radius**2+1005**2-2*radius*1005*cos_lower
            self.assertLess(upper_squared, 1000**2)

    def test_frozen_files_unchanged(self):
        manifest = json.loads((OUT/'e11_frozen_manifest.json').read_text(encoding='utf-8'))
        for item in manifest['files']:
            self.assertEqual(hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest(), item['sha256'], item['path'])

    def test_comparator_detects_decision_changes(self):
        event = {'path': '/measure', 'payload': {'position': {'x': 1., 'y': 2.}, 'channel': 3, 'request_id': 'a'},
                 'response': {'measure_result': 'direction', 'svd_deg': 10., 'virtual_time_s': 5}, 'decision_reason': 'discovery'}
        same = copy.deepcopy(event); same['payload']['request_id'] = 'b'
        self.assertEqual(canonical([event]), canonical([same]))
        for field in ('position', 'channel', 'response', 'order'):
            changed = copy.deepcopy(event)
            if field == 'position': changed['payload']['position']['x'] += 1e-12
            if field == 'channel': changed['payload']['channel'] += 1
            if field == 'response': changed['response']['svd_deg'] += .01
            if field == 'order': changed['decision_reason'] = 'recovery'
            self.assertNotEqual(canonical([event]), canonical([changed]))
        self.assertNotEqual(canonical([event, changed]), canonical([changed, event]))


class ActionEquivalenceTests(unittest.TestCase):
    def test_default_and_forced_six_exact_actions(self):
        cases = [make_case(seed) for seed in (24494904, 1200601, 1200602)]
        cases += [make_case(1200610+i, pattern=pattern) for i, pattern in enumerate(('boundary', 'cluster', 'origin'))]
        near = make_case(1200613, count=16)
        for source in near['sources']:
            source.update(x=0, y=0)
        cases.append(near)
        records = []
        for case in cases:
            seed = case['seed']
            with self.subTest(seed=seed):
                base, bc, ba = execute(case, f'{seed}-E11', baseline=True)
                for mode in ('default', 'forced6'):
                    seen = []
                    def force(k0):
                        seen.append(k0)
                        return E11_TEMPLATE
                    solver, client, actions = execute(case, f'{seed}-{mode}', selector=force if mode == 'forced6' else None)
                    self.assertEqual(ba, actions)
                    self.assertEqual(bc.stats, client.stats)
                    self.assertEqual(bc.virtual_time_s, client.virtual_time_s)
                    self.assertEqual(base.rotation, solver.rotation)
                    if mode == 'forced6':
                        self.assertEqual(len(seen), 0 if case is near else 1)
                    records.append({'seed': seed, 'pattern': case['pattern'], 'mode': mode,
                                    'actions': len(actions), 'exact_equal': True, 'rotation': solver.rotation,
                                    'k0': solver.k0, 'virtual_time_s': client.virtual_time_s,
                                    'canonical_sha256': hashlib.sha256(json.dumps(actions, sort_keys=True).encode()).hexdigest()})
                    OUT.mkdir(parents=True, exist_ok=True)
                    (OUT/'action_equivalence.json').write_text(json.dumps({'scope': 'correctness_only_not_performance_selection',
                        'trace_directory': TRACE.relative_to(ROOT).as_posix(), 'records': records}, indent=2)+'\n', encoding='utf-8')

    def test_seven_smoke_and_channel_evidence(self):
        records = []
        for pattern in ('random', 'boundary'):
            case = make_case(1200703, count=10, pattern=pattern)
            solver, client, actions = execute(case, 'seven-'+pattern, selector=lambda k0: SEVEN_TEMPLATE)
            self.assertEqual(solver.coverage_template.n, 7)
            self.assertEqual(len(solver.sites), 8)
            for ch in set(range(1, 21))-set(solver.tracks):
                self.assertEqual(solver.checked[ch], set(range(8)))
            records.append({'pattern': pattern, 'cleared': len(solver.env.cleared), 'total': 10, 'actions': len(actions)})
        (OUT/'seven_smoke.json').write_text(json.dumps(records, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    unittest.main()
