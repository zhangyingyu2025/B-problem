import math
from pathlib import Path
import random
import sys
import unittest
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
from b1plus import intersection, circle_planes
from clear_risk import classify, localization_point
from coverage_certificate import certificate


class B1PlusTests(unittest.TestCase):
    def test_truth_and_circle_support(self):
        rng = random.Random(54000000)
        for i in range(20):
            ang = rng.random()*2*math.pi; r = rng.uniform(800, 1800)
            x = (r*math.cos(ang), r*math.sin(ang))
            station = (x[0]+400, x[1]+500)
            theta = math.degrees(math.atan2(x[1]-station[1], x[0]-station[0]))%360
            obs = [{'x': station[0], 'y': station[1], 'svd_deg': theta}]
            silent = [(x[0]+1600, x[1]+100)]
            g = intersection(obs, [station], silent)
            self.assertEqual(g['status'], 'bounded')
            for a, b, c in circle_planes((0, 0), 1800):
                self.assertLessEqual(a*x[0]+b*x[1], c+1e-8)
            poly = g['polygon']
            for a, b in zip(poly, poly[1:]+poly[:1]):
                self.assertGreaterEqual((b[0]-a[0])*(x[1]-a[1])-(b[1]-a[1])*(x[0]-a[0]), -1e-6)
            self.assertLessEqual(math.dist(x, g['mec'][0]), g['mec'][1]+1e-6)

    def test_policy_does_not_call_large_mec_guaranteed(self):
        g = {'status': 'bounded', 'mec': ((0, 0), 400), 'diameter': 800,
             'polygon': [(-400, 0), (400, 0)], 'axis_ends': [(-400, 0), (400, 0)]}
        self.assertEqual(classify(g, (0, 0))['kind'], 'needs_localization')
        p = localization_point(g, (100, 100))
        self.assertLessEqual(max(math.dist(p, x) for x in g['polygon']), 620)

    def test_voronoi_certificate_without_sampling(self):
        for n, rho in ((6, 1130), (7, 1005)):
            sites = [(0, 0)]+[(rho*math.cos(2*math.pi*k/n), rho*math.sin(2*math.pi*k/n)) for k in range(n)]
            result = certificate(sites)
            self.assertEqual(result['status'], 'covered')
            self.assertLess(result['radius_upper_m'], 1000)
            self.assertEqual(certificate(sites[:-1])['status'], 'not_covered')
        self.assertEqual(certificate([(0, 0)])['status'], 'not_covered')

    def test_silence_before_discovery_is_retained(self):
        from solver import build_e13
        from offline_environment import OfflineTransport, make_case
        from protocol_adapter import Client
        engine = build_e13(Client(OfflineTransport(make_case(54000030)), 'offline-test'))
        engine.handle((0, 0), 1, {'measure_result': 'no_signal'}, 0)
        engine.handle((1130, 0), 1, {'measure_result': 'direction', 'svd_deg': 1.0}, 1)
        self.assertEqual(engine.tracks[1]['nosig'], [(0, 0)])
        self.assertEqual(engine.tracks[1]['geometry']['silent_count'], 1)
        self.assertLessEqual(math.dist((1500, 0), engine.tracks[1]['mec'][0]), engine.tracks[1]['mec'][1])
        engine.env.cleared.add(1)
        engine.handle((2000, 0), 1, {'measure_result': 'no_signal'})
        self.assertEqual(engine.tracks[1]['nosig'], [(0, 0)])

    def test_two_ray_points_cover_small_risky_region(self):
        for i in range(601):
            d = 20+i/10
            for sign in (-1, 1):
                p = (d*math.cos(math.radians(sign)), d*math.sin(math.radians(sign)))
                self.assertLess(min(math.dist(p, (q, 0)) for q in (35, 70)), 20)

    def test_wide_region_measurement_requires_whole_polygon_reception(self):
        g = {'mec': ((0., 0.), 750.), 'axis_ends': [(-750., 0.), (750., 0.)],
             'polygon': [(-750., 0.), (0., 25.), (750., 0.), (0., -25.)]}
        self.assertIsNone(localization_point(g, (100., 100.)))
        p = localization_point(g, (100., 100.), allow_wide=True)
        self.assertIsNotNone(p)
        self.assertLess(max(math.dist(p, v) for v in g['polygon']), 1000)
        g.update(mec=((0., 0.), 1100.), axis_ends=[(-1100., 0.), (1100., 0.)],
                 polygon=[(-1100., 0.), (1100., 0.)])
        self.assertIsNone(localization_point(g, (100., 100.), allow_wide=True))


if __name__ == '__main__': unittest.main()
