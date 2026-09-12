import copy
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B4/code'))
sys.path.insert(0, str(ROOT/'B3/code'))
from offline_environment_b4 import OfflineTransportB4, make_case, PATTERNS, wrap180
from offline_environment import OfflineTransport
from protocol_adapter import Client


def source(kind='omni', angle=None, x=0., y=0., radius=1000., channel=1):
    return {'kind': kind, 'direction_deg': angle, 'x': x, 'y': y, 'radius': radius, 'channel': channel}


class ProtocolTests(unittest.TestCase):
    def setup_env(self, s=None, mode='fixed_field'):
        self.env = OfflineTransportB4({'seed': 123, 'sources': [s or source()]}, error_mode=mode)
        self.client = Client(self.env, 'offline-test', session_id='unit')
        self.client.enter()
        return self.client

    def payload(self, rid='manual', **kw):
        return {'arena_id': 'default', 'robot_id': 'offline-test', 'request_id': rid, **kw}

    def test_initial_state(self):
        c = self.setup_env(); self.assertEqual((c.position, c.channel, c.virtual_time_s), ((0., 0.), 1, 0))

    def test_omni_radius_inclusive(self):
        c = self.setup_env(); self.assertEqual(c.measure((1000., 0.), 1)['measure_result'], 'direction')

    def test_omni_radius_outside(self):
        c = self.setup_env(); self.assertEqual(c.measure((1000.0001, 0.), 1)['measure_result'], 'no_signal')

    def test_directional_positive_boundary(self):
        c = self.setup_env(source('directional', 0.)); self.assertEqual(c.measure((0., 100.), 1)['measure_result'], 'direction')

    def test_directional_negative_boundary(self):
        c = self.setup_env(source('directional', 0.)); self.assertEqual(c.measure((0., -100.), 1)['measure_result'], 'direction')

    def test_directional_just_outside(self):
        c = self.setup_env(source('directional', 0.)); self.assertEqual(c.measure((-1e-5, 100.), 1)['measure_result'], 'no_signal')

    def test_directional_angle_wrap(self):
        c = self.setup_env(source('directional', 359.)); self.assertEqual(c.measure((100., 0.), 1)['measure_result'], 'direction')

    def test_near_boundary(self):
        c = self.setup_env(); r = c.measure((5., 0.), 1)
        self.assertEqual(r['measure_result'], 'near'); self.assertNotIn('svd_deg', r)

    def test_near_backside_is_no_signal(self):
        c = self.setup_env(source('directional', 0.)); self.assertEqual(c.measure((-1., 0.), 1)['measure_result'], 'no_signal')

    def test_source_vertex_is_near_for_any_orientation(self):
        c = self.setup_env(source('directional', 240.)); self.assertEqual(c.measure((0., 0.), 1)['measure_result'], 'near')

    def test_clear_backside_at_boundary(self):
        c = self.setup_env(source('directional', 0.)); self.assertEqual(c.clear((-20., 0.), 1)['clear_result'], 'success')

    def test_clear_outside_boundary(self):
        c = self.setup_env(); self.assertEqual(c.clear((20.0001, 0.), 1)['clear_result'], 'no_target_in_range')

    def test_clear_preserves_measurement_channel(self):
        c = self.setup_env(); c.measure((100., 0.), 2); c.clear((0., 0.), 1)
        self.assertEqual((c.channel, self.env.channel, c.stats['switches']), (2, 2, 1))

    def test_absent_channel(self):
        c = self.setup_env(); self.assertEqual(c.measure((0., 0.), 20)['measure_result'], 'no_signal')

    def test_cleared_source_no_signal(self):
        c = self.setup_env(); c.clear((0., 0.), 1)
        self.assertEqual(c.measure((0., 0.), 1)['measure_result'], 'no_signal')
        self.assertEqual(c.clear((0., 0.), 1)['clear_result'], 'no_target_in_range')

    def test_repeated_reading_fixed(self):
        c = self.setup_env(); a = c.measure((123., 456.), 1); b = c.measure((123., 456.), 1)
        self.assertEqual(a['svd_deg'], b['svd_deg'])

    def test_error_bound_and_quantization(self):
        for mode in ('fixed_field', 'endpoints', 'zero'):
            c = self.setup_env(mode=mode)
            for k in range(100):
                p = (500*math.cos(k*.1), 500*math.sin(k*.1))
                r = c.measure(p, 1)['svd_deg']; true = math.degrees(math.atan2(-p[1], -p[0]))
                self.assertLessEqual(abs(wrap180(r-true)), 1+1e-12)
                self.assertAlmostEqual(r*100, round(r*100), places=8)

    def test_documented_time_example(self):
        c = self.setup_env(source(x=-1700))
        c.measure((300, 400), 1); self.assertEqual(c.virtual_time_s, 105)
        c.measure((300, 400), 2); self.assertEqual(c.virtual_time_s, 111)
        c.clear((300, 0), 3); self.assertEqual(c.virtual_time_s, 194)
        c.measure((300, 0), 2); self.assertEqual(c.virtual_time_s, 199)
        c.exit(); self.assertEqual(c.virtual_time_s, 199)

    def test_b3_omni_business_equivalence(self):
        case = {'seed': 123, 'sources': [source(x=500)]}
        e3 = OfflineTransport(case); e4 = OfflineTransportB4(case)
        for index, (path, p, ch) in enumerate([('/enter', None, None), ('/measure', (0,0), 1),
                ('/measure', (300,400), 2), ('/clear', (500,0), 1), ('/measure', (500,0), 1), ('/exit',None,None)]):
            payload = self.payload(str(index))
            if p is not None: payload.update(position={'x':p[0], 'y':p[1]}, channel=ch)
            a, b = e3(path, payload), e4(path, payload)
            self.assertEqual(a[0], b[0]); aa, bb = dict(a[1]), dict(b[1])
            self.assertAlmostEqual(aa.pop('virtual_time_s'), bb.pop('virtual_time_s'), places=5)
            self.assertEqual(aa, bb)

    def test_idempotent_clear_not_repeated(self):
        self.setup_env(); p = self.payload(position={'x':0, 'y':0}, channel=1)
        a = self.env('/clear', p); vt = self.env.vt
        self.assertEqual(a, self.env('/clear', p)); self.assertEqual(self.env.vt, vt)

    def test_id_conflict(self):
        self.setup_env(); p = self.payload(position={'x':0, 'y':0}, channel=1)
        self.env('/clear', p); self.assertEqual(self.env('/measure', p)[0], 409)

    def test_rejection_does_not_advance_or_occupy_id(self):
        self.setup_env(); p = self.payload(position={'x':100, 'y':0}, channel=1, garbage=True)
        self.assertFalse(self.env('/measure', p)[1]['accepted']); self.assertEqual(self.env.vt, 0)
        del p['garbage']; self.assertTrue(self.env('/measure', p)[1]['accepted'])

    def test_invalid_positions(self):
        self.setup_env()
        for value in (True, float('nan'), float('inf'), 2000001, '1'):
            with self.subTest(value=value):
                self.assertEqual(self.env('/measure', self.payload(position={'x':value, 'y':0}, channel=1))[0], 400)

    def test_float_integer_channel_accepted(self):
        self.setup_env(); self.assertTrue(self.env('/measure', self.payload(position={'x':100, 'y':0}, channel=1.0))[1]['accepted'])

    def test_invalid_channels(self):
        self.setup_env()
        for ch in (True, 0, 21, 1.5, '1'):
            self.assertEqual(self.env('/measure', self.payload(position={'x':0, 'y':0}, channel=ch))[0], 400)

    def test_wrong_identity(self):
        self.setup_env(); p = self.payload(); p['robot_id'] = 'wrong'
        self.assertFalse(self.env('/enter', p)[1]['accepted'])

    def test_illegal_identifier(self):
        self.setup_env(); self.assertEqual(self.env('/exit', self.payload('bad\n'))[0], 400)

    def test_exit_retry_and_no_reentry(self):
        self.setup_env(); p = self.payload()
        a = self.env('/exit', p); self.assertEqual(a, self.env('/exit', p))
        self.assertFalse(self.env('/enter', self.payload('new'))[1]['accepted'])

    def test_unknown_path(self):
        self.setup_env(); self.assertEqual(self.env('/measure/', self.payload())[0], 404)

    def test_truth_is_copied(self):
        case = {'sources':[source()]}; env = OfflineTransportB4(case)
        case['sources'][0]['x'] = 999; self.assertEqual(env._sources[1]['x'], 0)

    def test_patterns_reproducible_and_valid(self):
        for pattern in PATTERNS:
            a = make_case(64000000, pattern=pattern)
            self.assertEqual(a, make_case(64000000, pattern=pattern)); OfflineTransportB4(a)
            self.assertEqual(len(a['sources']), len({s['channel'] for s in a['sources']}))


if __name__ == '__main__':
    unittest.main()
