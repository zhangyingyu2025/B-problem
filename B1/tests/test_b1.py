"""Deterministic geometric, invariance, failure-oriented and CLI tests."""
from fractions import Fraction as F
from itertools import permutations
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE/'code'))
import b1_geometry as g


def box(x0=0, x1=4, y0=0, y1=3):
    return [(1,0,x1), (-1,0,-x0), (0,1,y1), (0,-1,-y0)]


def triangle_observations():
    return [dict(x=-900,y=0,svd_deg=1),
            dict(x=486,y=-450*math.sqrt(3),svd_deg=121),
            dict(x=468,y=468*math.sqrt(3),svd_deg=241)]


def measurements(target, stations, errors=None):
    errors = errors or [0]*len(stations)
    return [dict(x=x,y=y,svd_deg=(math.degrees(math.atan2(target[1]-y,target[0]-x))+e)%360)
            for (x,y),e in zip(stations,errors)]


def fourier_motzkin_feasible(planes):
    """Independent feasibility oracle: eliminate y, then intersect x intervals."""
    hs=[tuple(map(F,h)) for h in planes]
    upper=[h for h in hs if h[1]>0]; lower=[h for h in hs if h[1]<0]
    one=[(a,c) for a,b,c in hs if not b]
    for au,bu,cu in upper:
        for al,bl,cl in lower:
            one.append((au/bu-al/bl,cu/bu-cl/bl))
    lo=hi=None
    for a,c in one:
        if not a:
            if c<0: return False
        elif a>0:
            hi=c/a if hi is None else min(hi,c/a)
        else:
            lo=c/a if lo is None else max(lo,c/a)
    return lo is None or hi is None or lo<=hi


class FeasibilityTests(unittest.TestCase):
    def test_independent_fourier_motzkin_oracle(self):
        rng=random.Random(20260911)
        for _ in range(160):
            hs=[(rng.randint(-4,4),rng.randint(-4,4),rng.randint(-6,6)) for _ in range(rng.randint(0,9))]
            expected=fourier_motzkin_feasible(hs)
            self.assertEqual(g.feasible_point(hs) is not None,expected,hs)
            if expected:
                cone=[(a,b,0) for a,b,c in hs]
                unbounded=any(fourier_motzkin_feasible(cone+[(a,b,s),(-a,-b,-s)])
                              for a,b,s in [(1,0,1),(1,0,-1),(0,1,1),(0,1,-1)])
                self.assertEqual(g.recession_direction(hs) is not None,unbounded,hs)
    def test_full_plane_halfplane_strip_and_line_have_no_vertices(self):
        cases = [[], [(1,0,1)], [(1,0,1),(-1,0,0)], [(1,0,2),(-1,0,-2)]]
        for hs in cases:
            with self.subTest(hs=hs), patch.object(g, '_vertices', side_effect=AssertionError('must not enumerate')):
                r = g.solve_halfplanes(hs)
                self.assertEqual(r['status'],'unbounded')
                self.assertIsNotNone(r['feasible_point'])

    def test_empty_independent_of_vertices_and_cone(self):
        for hs in [[(1,0,0),(-1,0,-1)],[(0,0,-1)],[(1,0,0),(0,1,0),(-1,-1,-1)]]:
            with self.subTest(hs=hs), patch.object(g, '_vertices', side_effect=AssertionError()), patch.object(g, '_recession', side_effect=AssertionError()):
                self.assertIsNone(g.feasible_point(hs))
                self.assertEqual(g.solve_halfplanes(hs)['status'],'empty')

    def test_tiny_gap_not_silently_expanded(self):
        self.assertIsNone(g.feasible_point([(1,0,0),(-1,0,-1e-13)]))

    def test_zero_normal_tautology(self):
        r=g.solve_halfplanes([(0,0,0),(0,0,100)]+box())
        self.assertEqual(r['status'],'bounded')
        self.assertEqual(r['diagnostics'][0]['indices'],[0,1])

    def test_point_segment_ray(self):
        for hs,status in [(box(2,2,3,3),'point'),(box(0,5,3,3),'segment'),
                          ([(0,1,0),(0,-1,0),(-1,0,0)],'unbounded')]:
            with self.subTest(status=status):
                self.assertEqual(g.solve_halfplanes(hs)['status'],status)

    def test_recession_certificate_exact(self):
        systems=[[],[(1,0,0)],[(1,0,1),(-1,0,0)],[(0,1,0),(0,-1,0),(-1,0,0)]]
        for hs in systems:
            r=g.solve_halfplanes(hs)
            d=tuple(F(x) for x in r['recession_direction_exact'])
            self.assertNotEqual(d,(0,0))
            self.assertTrue(all(F(a)*d[0]+F(b)*d[1]<=0 for a,b,c in hs))
        r=g.solve_halfplanes(box())
        self.assertEqual(len(r['recession_checks']),4)
        self.assertTrue(all(not c['feasible'] for c in r['recession_checks']))

    def test_constraint_permutation(self):
        for hs in permutations(box(3,7,-5,-2)):
            p=g.feasible_point(hs)
            self.assertTrue(all(a*p[0]+b*p[1]<=c+1e-12 for a,b,c in hs))

    def test_bounded_vertex_failure_is_not_empty(self):
        with patch.object(g,'_vertices',return_value=([],[])):
            r=g.solve_halfplanes(box())
            self.assertEqual(r['status'],'numerically_uncertain')
            self.assertEqual(r['algebraic_status'],'bounded')
            self.assertIsNone(r['diameter_m'])


class GeometryTests(unittest.TestCase):
    def test_almost_right_triangle_circle_is_uncertain(self):
        # Vertex (0, 3+epsilon) is not the desired near-circle test; instead use
        # a triangle whose apex is just above a diameter-4 semicircle.
        height=2+1e-11
        r=g.solve_halfplanes([(0,-1,0),(-height,2,0),(height,2,4*height)])
        self.assertEqual(r['status'],'numerically_uncertain')
        self.assertIsNone(r['diameter_circle'])
    def test_rectangle_and_multiple_farthest_pairs(self):
        r=g.solve_halfplanes(box())
        self.assertEqual(r['diameter_m'],5)
        self.assertTrue(r['diameter_circle']['covers'])
        self.assertEqual(r['diameter_circle']['center'],[2,1.5])
        self.assertEqual(r['minimum_enclosing_circle']['radius_m'],2.5)
        for a,b in [((0,0),(4,3)),((0,3),(4,0))]:
            self.assertEqual([(a[i]+b[i])/2 for i in (0,1)],r['diameter_circle']['center'])

    def test_single_point_and_segment_circles(self):
        for hs,diameter in [(box(2,2,3,3),0),(box(-2,3,1,1),5)]:
            r=g.solve_halfplanes(hs)
            self.assertEqual(r['diameter_m'],diameter)
            self.assertTrue(r['diameter_circle']['covers'])
            self.assertTrue(r['circle_crosscheck_passed'])

    def test_rational_acute_triangle(self):
        # vertices (0,0),(4,0),(2,3): D=4, circumcenter=(2,5/6), R=13/6
        r=g.solve_halfplanes([(0,-1,0),(-3,2,0),(3,2,12)])
        self.assertEqual(r['diameter_m'],4)
        self.assertFalse(r['diameter_circle']['covers'])
        self.assertAlmostEqual(r['minimum_enclosing_circle']['radius_m'],13/6)

    def test_realizable_equilateral_counterexample(self):
        obs=triangle_observations()
        r=g.solve_b1(obs)
        self.assertEqual(r['status'],'bounded')
        self.assertEqual(len(r['vertices']),3)
        self.assertAlmostEqual(r['diameter_m'],36,places=9)
        self.assertFalse(r['diameter_circle']['covers'])
        self.assertAlmostEqual(r['minimum_enclosing_circle']['diameter_m'],24*math.sqrt(3),places=9)
        self.assertTrue(r['circle_crosscheck_passed'])
        for v in [(0,0),(36,0),(18,18*math.sqrt(3))]:
            for o in obs:
                actual=math.degrees(math.atan2(v[1]-o['y'],v[0]-o['x']))
                err=(actual-o['svd_deg']+180)%360-180
                self.assertLessEqual(abs(err),1+1e-12)
                self.assertLessEqual(math.dist(v,(o['x'],o['y'])),936+1e-9)

    def test_mec_does_not_control_primary_verdict(self):
        # A deliberately wrong independent MEC must cause a failed crosscheck,
        # not overwrite the independently obtained midpoint-circle verdict.
        v=[(F(0),F(0)),(F(4),F(0)),(F(2),F(3))]
        with patch.object(g,'_minimum_circle',return_value=((F(2),F(0)),F(4),[0,1])):
            r=g._circle_decision(v)
            self.assertFalse(r['diameter_circle']['covers'])
            self.assertFalse(r['circle_crosscheck_passed'])

    def test_near_parallel_active_boundaries_are_uncertain(self):
        hs=[(0,-1,0),(-1e-12,1,-1), (1,0,2e12)]
        r=g.solve_halfplanes(hs)
        self.assertEqual(r['status'],'numerically_uncertain')
        self.assertEqual(r['algebraic_status'],'bounded')
        self.assertIsNone(r['diameter_m'])

    def test_redundant_near_parallel_does_not_spoil_box(self):
        r=g.solve_halfplanes(box()+[(1,1e-13,100)])
        self.assertEqual(r['status'],'bounded')
        self.assertEqual(r['diameter_m'],5)

    def test_circle_check_for_right_and_obtuse_triangle(self):
        for v in [[(0,0),(4,0),(0,3)],[(0,0),(4,0),(1,1)]]:
            r=g._circle_decision([tuple(map(F,p)) for p in v])
            self.assertTrue(r['diameter_circle']['covers'])
            self.assertTrue(r['circle_crosscheck_passed'])


class BearingAndPropertyTests(unittest.TestCase):
    def test_symmetric_two_station_cocircular_regression(self):
        obs=[dict(x=-100,y=-100,svd_deg=45),dict(x=100,y=-100,svd_deg=135)]
        r=g.solve_b1(obs)
        self.assertEqual(r['status'],'bounded')
        self.assertTrue(r['diameter_circle']['covers'])
        self.assertTrue(r['circle_crosscheck_passed'])
        self.assertEqual(len(r['vertices']),4)
    def test_zero_wrap_duplicate_and_no_signal(self):
        obs=measurements((100,0),[(0,0),(100,-100),(200,0)])
        r=g.solve_b1(obs+[obs[0]])
        self.assertEqual(r['status'],'bounded')
        self.assertEqual(r['unique_count'],3)
        self.assertEqual(r['duplicate_indices'],[3])
        hs,_,_=g.observations_to_halfplanes(obs)
        self.assertTrue(all(a*100 <= c+1e-10 for a,b,c in hs))
        with self.assertRaises(ValueError):
            g.solve_b1([{'x':0,'y':0,'measure_result':'no_signal'}])

    def test_backward_rays_do_not_intersect(self):
        r=g.solve_b1([dict(x=1,y=0,svd_deg=0),dict(x=-1,y=0,svd_deg=180)])
        self.assertEqual(r['status'],'empty')

    def test_one_station_unbounded(self):
        self.assertEqual(g.solve_b1([dict(x=0,y=0,svd_deg=359.5)])['status'],'unbounded')

    def test_transform_invariance(self):
        obs=triangle_observations()
        for rotation,scale,shift in [(0,1,(10000,-5000)),(37,1,(0,0)),(0,0.1,(0,0)),(121,10,(50,-100))]:
            t=math.radians(rotation)
            transformed=[]
            for o in obs:
                x,y=o['x'],o['y']
                transformed.append(dict(x=scale*(x*math.cos(t)-y*math.sin(t))+shift[0],
                                        y=scale*(x*math.sin(t)+y*math.cos(t))+shift[1],
                                        svd_deg=(o['svd_deg']+rotation)%360))
            r=g.solve_b1(transformed)
            self.assertEqual(r['status'],'bounded')
            self.assertAlmostEqual(r['diameter_m'],36*scale,delta=1e-7*max(1,scale))
            self.assertFalse(r['diameter_circle']['covers'])

    def test_seeded_true_target_containment_and_monotonicity(self):
        rng=random.Random(20260910)
        for trial in range(30):
            target=(rng.uniform(-300,300),rng.uniform(-300,300))
            stations=[]
            for deg in (0,90,180,270,45):
                t=math.radians(deg+rng.uniform(-10,10)); radius=rng.uniform(200,900)
                stations.append((target[0]+radius*math.cos(t),target[1]+radius*math.sin(t)))
            obs=measurements(target,stations,[rng.uniform(-0.95,0.95) for _ in stations])
            previous=None
            for count in (3,4,5):
                hs,_,_=g.observations_to_halfplanes(obs[:count])
                self.assertTrue(all(a*target[0]+b*target[1]<=c+1e-8 for a,b,c in hs))
                r=g.solve_b1(obs[:count])
                self.assertEqual(r['status'],'bounded', (trial,count))
                self.assertTrue(r['circle_crosscheck_passed'])
                if previous:
                    self.assertLessEqual(r['diameter_m'],previous['diameter_m']+1e-8)
                    self.assertLessEqual(r['minimum_enclosing_circle']['radius_m'],previous['minimum_enclosing_circle']['radius_m']+1e-8)
                    for x,y in r['vertices']:
                        self.assertTrue(all(a*x+b*y<=c+1e-8 for a,b,c in previous['halfplanes']))
                previous=r

    def test_invalid_input(self):
        for obs in [[],[dict(x=float('nan'),y=0,svd_deg=0)], [dict(x=True,y=0,svd_deg=0)],
                    [dict(x=0,y=0,svd_deg=360)], [dict(x=0,y=0,svd_deg=-1)]]:
            with self.assertRaises(ValueError): g.solve_b1(obs)
        for alpha in (0,90,-1,float('inf')):
            with self.assertRaises(ValueError): g.solve_b1(triangle_observations(),alpha)


class CLITests(unittest.TestCase):
    def test_json_svg_reproducibility_and_input_protection(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp); src=base/'case.json'; dest=base/'out.json'; svg=base/'out.svg'
            src.write_text(json.dumps({'observations':triangle_observations(),'data_kind':'synthetic'}),encoding='utf-8')
            before=src.read_bytes()
            command=[sys.executable,str(BASE/'code'/'run_b1.py'),str(src),'--output',str(dest),'--svg',str(svg)]
            for _ in range(2):
                run=subprocess.run(command,capture_output=True,text=True)
                self.assertEqual(run.returncode,0,run.stderr)
                current=dest.read_bytes()
                if _==0: first=current
                else: self.assertEqual(first,current)
            self.assertEqual(src.read_bytes(),before)
            self.assertIn('<svg',svg.read_text(encoding='utf-8'))
            result=json.loads(current)
            self.assertFalse(result['diameter_circle']['covers'])
            collision=subprocess.run(command[:3]+['--output',str(src)],capture_output=True)
            self.assertEqual(collision.returncode,2)
            self.assertEqual(src.read_bytes(),before)

    def test_nan_and_duplicate_json_rejected(self):
        from run_b1 import load_payload
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.json'
            for text in ['{"observations":[],"error_deg":NaN}', '{"observations":[],"observations":[]}']:
                p.write_text(text,encoding='utf-8')
                with self.assertRaises(ValueError): load_payload(p)


if __name__ == '__main__':
    unittest.main(verbosity=2)
