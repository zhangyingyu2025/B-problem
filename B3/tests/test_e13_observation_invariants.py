"""Synthetic truth is used solely by assertions after online decisions.

The observer never returns truth to the solver or changes a decision.
"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
from solver import build_e13
from offline_environment import make_case, OfflineTransport
from protocol_adapter import Client


class ObservationInvariantTests(unittest.TestCase):
    def test_outer_regions_retain_truth_and_shrink_on_stress_cases(self):
        total_updates = 0
        for i, pattern in enumerate(('random', 'boundary', 'cluster', 'origin') * 2):
            case = make_case(54001000+i, pattern=pattern)
            truth = {s['channel']: (s['x'], s['y']) for s in case['sources']}
            for variant in ('M', 'O'):
                with self.subTest(seed=case['seed'], pattern=pattern, variant=variant):
                    transport = OfflineTransport(case, error_mode='endpoints' if i < 4 else 'fixed_field')
                    client = Client(transport, 'offline-test')
                    client.enter()
                    engine = build_e13(client, variant)
                    handle = engine.handle
                    last_radius = {}

                    def observe(*args, **kwargs):
                        nonlocal total_updates
                        handle(*args, **kwargs)
                        channel = args[1]
                        if channel not in engine.tracks or channel in engine.env.cleared:
                            return
                        g = engine.tracks[channel].get('geometry')
                        if not g or g['status'] != 'bounded':
                            return
                        total_updates += 1
                        point = truth[channel]
                        poly = g['polygon']
                        for a, b in zip(poly, poly[1:]+poly[:1]):
                            length = math.dist(a, b)
                            if length < 1e-12:
                                continue
                            signed = ((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]))/length
                            self.assertGreaterEqual(signed, -1e-6)
                        center, radius = g['mec']
                        self.assertLessEqual(math.dist(point, center), radius+1e-6)
                        if channel in last_radius:
                            self.assertLessEqual(radius, last_radius[channel]+1e-6)
                        last_radius[channel] = radius

                    engine.handle = observe
                    engine.run_all()
                    self.assertEqual(set(truth), transport.cleared)
                    self.assertTrue(engine.discovery_complete())
                    self.assertEqual(engine.guaranteed_clear_failures, 0)
                    self.assertEqual(engine.b1plus_empty, 0)
                    client.exit()
        self.assertGreater(total_updates, 200)


if __name__ == '__main__':
    unittest.main()
