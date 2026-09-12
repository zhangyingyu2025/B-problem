"""Failure-oriented and independent numerical checks. All data synthetic."""
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

BASE=Path(__file__).resolve().parents[1]
ROOT=BASE.parent
sys.path.insert(0,str(BASE/'code'))
from domains import Problem
from geometry import metrics,precise_metrics,minimum_circle,decimal_unit
from b2_solver import evaluate_candidate,solve_b2
from run_b2 import run_file


def problem(**kwargs):
    return Problem({'x':0,'y':0,'svd_deg':0},**kwargs)


class DomainTests(unittest.TestCase):
    def test_inconsistent_and_external_first_point(self):
        self.assertEqual(Problem({'x':4000,'y':0,'svd_deg':0}).status,'inconsistent_observation')
        self.assertEqual(Problem({'x':2200,'y':0,'svd_deg':180}).status,'ready')

    def test_external_tangent_source_set(self):
        p=Problem({'x':3300,'y':0,'svd_deg':180})
        self.assertEqual(p.status,'ready')
        interval=p.bearing_interval((800,550))
        self.assertLess(interval['true_deg'][1]-interval['true_deg'][0],1e-7)

    def test_near_only_source_set_rejected(self):
        self.assertEqual(Problem({'x':1795.01,'y':0,'svd_deg':0}).status,'inconsistent_observation')

    def test_safe_and_effective_are_different(self):
        p=problem()
        self.assertTrue(p.analytic_safe((500,0)))
        self.assertFalse(p.effective_domain((500,0)))
        self.assertTrue(p.effective_domain((800,550)))
        self.assertFalse(p.analytic_safe((0,600)))

    def test_reception_boundary_guard(self):
        p=problem()
        self.assertFalse(p.analytic_safe((800,600))) # exactly 1000 m: guard excludes it
        self.assertTrue(p.analytic_safe((799.9,599.9)))

    def test_fixed_seed_reception_and_no_near(self):
        p=problem();rng=random.Random(20260911)
        for q in [(500,600),(800,550),(775,-625),(900,-400)]:
            self.assertTrue(p.effective_domain(q))
            for _ in range(250):
                r=rng.uniform(5.0001,1500);phi=math.radians(rng.uniform(-1,1))
                g=(r*math.cos(phi),r*math.sin(phi))
                self.assertLessEqual(math.dist(q,g),max(1000,r)+1e-8)
                self.assertGreater(math.dist(q,g),5)

    def test_circle_clipping_shortens_reading_interval(self):
        q=(800,550)
        wide=problem().bearing_interval(q)['observed_deg']
        clipped=Problem({'x':1500,'y':0,'svd_deg':0}).bearing_interval(q)['observed_deg']
        self.assertGreaterEqual(clipped[0],wide[0]-1e-8)
        self.assertLess(clipped[1],wide[1]-1)

    def test_projection_contains_all_sampled_sources(self):
        rng=random.Random(778)
        for obs in [{'x':0,'y':0,'svd_deg':0},{'x':1500,'y':200,'svd_deg':359.5},
                    {'x':2200,'y':100,'svd_deg':180}]:
            p=Problem(obs)
            for q in [(800,550),(775,-625)]:
                lo,hi=p.bearing_interval(q)['observed_deg']
                count=0
                for _ in range(500):
                    r=rng.uniform(5.01,1500);phi=math.radians(rng.uniform(-1,1))
                    g=(r*math.cos(phi),r*math.sin(phi))
                    if not p.in_physical(g): continue
                    theta=math.degrees(math.atan2(g[1]-q[1],g[0]-q[0]))
                    self.assertGreaterEqual(theta-1,lo-1e-7)
                    self.assertLessEqual(theta+1,hi+1e-7)
                    count+=1
                self.assertGreater(count,0)

    def test_bad_inputs(self):
        for obs in [{'x':True,'y':0,'svd_deg':0},{'x':0,'y':0,'svd_deg':360},
                    {'x':0,'y':math.nan,'svd_deg':0},{'x':0,'y':0}]:
            with self.assertRaises(ValueError): Problem(obs)
        with self.assertRaises(ValueError): problem(error_deg=0)
        with self.assertRaises(ValueError): problem(min_range=1600)


class GeometryTests(unittest.TestCase):
    def test_mec_equilateral_not_half_diameter(self):
        center,radius,support=minimum_circle([(0,0),(36,0),(18,18*math.sqrt(3))])
        self.assertAlmostEqual(radius,12*math.sqrt(3),places=10)
        self.assertGreater(radius,18)
        self.assertEqual(len(support),3)

    def test_circle_rectangle_segment_point(self):
        for vertices,expected in [([(0,0),(4,0),(4,3),(0,3)],2.5),
                                   ([(0,0),(10,0)],5), ([(3,5)],0)]:
            self.assertAlmostEqual(minimum_circle(vertices)[1],expected,places=10)

    def test_high_precision_trigonometry(self):
        for angle in [0,1,-1,90,179.9,359.5]:
            c,s=decimal_unit(angle)
            self.assertAlmostEqual(float(c),math.cos(math.radians(angle)),places=14)
            self.assertAlmostEqual(float(s),math.sin(math.radians(angle)),places=14)

    def test_precise_failure_baseline_unbounded(self):
        self.assertEqual(precise_metrics((500,0),0)['status'],'unbounded')
        self.assertGreater(math.dist((0,600),(1500,0)),1500)

    def test_float_vs_independent_precise(self):
        rng=random.Random(333);p=problem()
        for q in [(500,600),(800,-550),(775,625)]:
            lo,hi=p.bearing_interval(q)['observed_deg']
            for _ in range(6):
                theta=rng.uniform(lo,hi)
                a=metrics(q,theta);b=precise_metrics(q,theta)
                self.assertEqual(b['status'],'bounded')
                for key in ['diameter_m','mec_radius_m']:
                    self.assertAlmostEqual(a[key],b[key],places=7)

    def test_degenerate_outer_source_endpoint(self):
        q=(800,550);theta=problem().bearing_interval(q)['observed_deg'][0]
        self.assertLess(metrics(q,theta)['diameter_m'],1e-7)


class EvaluationTests(unittest.TestCase):
    def test_adaptive_bounds_cover_dense_independent_grid(self):
        p=problem();q=(800,550);r=evaluate_candidate(p,q,192,0.5)
        self.assertEqual(r['high_precision_check']['status'],'passed')
        lo,hi=r['source_interval']['observed_deg']
        dense={'diameter_m':0.,'mec_radius_m':0.}
        for i in range(1001):
            v=metrics(q,lo+(hi-lo)*i/1000)
            for k in dense: dense[k]=max(dense[k],v[k])
        for k,label in [('diameter_m','worst_diameter'),('mec_radius_m','worst_mec_radius')]:
            self.assertLessEqual(dense[k],r[label]['envelope_upper_m']+1e-7)
            self.assertLessEqual(r[label]['sample_lower_m'],r[label]['envelope_upper_m'])

    def test_budget_exhaustion_is_explicit(self):
        r=evaluate_candidate(problem(),(800,550),4,0.001,0,False)
        self.assertEqual(r['status'],'interval_budget_reached')
        self.assertGreater(r['worst_diameter']['gap_m'],0.001)

    def test_more_refinement_tightens_envelope(self):
        a=evaluate_candidate(problem(),(800,550),16,0.001,0,False)
        b=evaluate_candidate(problem(),(800,550),64,0.001,0,False)
        for key in ['worst_diameter','worst_mec_radius']:
            self.assertGreaterEqual(b[key]['sample_lower_m']+1e-8,a[key]['sample_lower_m'])
            self.assertLessEqual(b[key]['envelope_upper_m'],a[key]['envelope_upper_m']+1e-8)

    def test_mirror_symmetry(self):
        a=evaluate_candidate(problem(),(800,550),128,0.5,high_precision=False)
        b=evaluate_candidate(problem(),(800,-550),128,0.5,high_precision=False)
        for key in ['worst_diameter','worst_mec_radius']:
            self.assertAlmostEqual(a[key]['sample_lower_m'],b[key]['sample_lower_m'],places=7)

    def test_translate_rotate_wrap_invariance(self):
        p=Problem({'x':320,'y':-150,'svd_deg':359.5},domain_center=(320,-150))
        q=(800,550)
        local=p.to_local(p.to_world(q))
        self.assertAlmostEqual(local[0],q[0],places=9)
        self.assertAlmostEqual(local[1],q[1],places=9)
        a=evaluate_candidate(problem(),q,64,0.5,high_precision=False)
        b=evaluate_candidate(p,q,64,0.5,high_precision=False)
        self.assertAlmostEqual(a['worst_diameter']['sample_lower_m'],b['worst_diameter']['sample_lower_m'],places=7)

    def test_scaling_all_physical_lengths(self):
        scale=2.7;q=(800,550)
        p=problem(min_range=1000*scale,max_range=1500*scale,near_range=5*scale,domain_radius=1800*scale)
        a=evaluate_candidate(problem(),q,64,0.5,high_precision=False)
        b=evaluate_candidate(p,tuple(v*scale for v in q),64,0.5*scale,high_precision=False)
        for key in ['worst_diameter','worst_mec_radius']:
            self.assertAlmostEqual(a[key]['sample_lower_m']*scale,b[key]['sample_lower_m'],places=6)

    def test_worst_sample_has_valid_physical_witness(self):
        p=problem();q=(800,550);r=evaluate_candidate(p,q,128,1,high_precision=False)
        for k in ['worst_diameter','worst_mec_radius']:
            w=r[k]['physical_witness'];self.assertIsNotNone(w)
            g=p.to_local(w['target_world_m'])
            self.assertTrue(p.in_physical(g))
            self.assertLessEqual(abs(w['second_error_deg']),1+1e-9)
            self.assertLessEqual(math.dist(q,g),w['compatible_radius_m']+1e-7)

    def test_high_precision_disagreement_is_not_silent(self):
        with patch('b2_solver.precise_metrics',return_value={'status':'unbounded'}):
            r=evaluate_candidate(problem(),(800,550),16)
        self.assertEqual(r['status'],'numerically_uncertain')
        self.assertIsNone(r['worst_diameter'])

    def test_primary_and_auxiliary_fields_separate(self):
        r=evaluate_candidate(problem(),(800,550),64,high_precision=False)
        self.assertIn('worst_mec_radius',r)
        self.assertIn('sample_geometry_local',r['worst_mec_radius'])
        self.assertEqual(r['clearance_20m_assessment'],'counterexample_observation_found')


class IntegrationTests(unittest.TestCase):
    def test_twenty_meter_coverage_positive_case(self):
        p=Problem({'x':1500,'y':0,'svd_deg':0})
        r=evaluate_candidate(p,(225,-225),192,1)
        self.assertEqual(r['clearance_20m_assessment'],'supported_by_conservative_float_upper')
        self.assertLess(r['worst_mec_radius']['envelope_upper_m'],20)

    def test_deterministic_search_and_distinct_domains(self):
        opt={'candidate_points_local':[[500,600],[800,550],[775,-625]],'refinement_levels':0,'max_intervals':64,'final_intervals':128}
        a=solve_b2({'x':0,'y':0,'svd_deg':0},options=opt)
        b=solve_b2({'x':0,'y':0,'svd_deg':0},options=opt)
        self.assertEqual(a,b)
        self.assertEqual(a['recommended']['high_precision_check']['status'],'passed')
        self.assertEqual(len(a['domains']),5) # K plus four different placement domains
        self.assertTrue(a['preferred_candidates_local_m'])

    def test_default_search_escapes_coarse_grid_trap(self):
        """Regression: default search must not stay at the old coarse-grid point."""
        result=solve_b2({'x':0,'y':0,'svd_deg':0})
        best=result['recommended']
        self.assertIsNotNone(best)
        self.assertEqual(result['search']['refinement_levels'],7)
        self.assertEqual(result['search']['max_candidates'],1200)
        self.assertNotEqual(
            tuple(round(v,6) for v in best['point_local_m']),
            (775.0,-625.0)
        )
        self.assertEqual(best['high_precision_check']['status'],'passed')
        self.assertLess(best['worst_diameter']['envelope_upper_m'],137.0)
        self.assertEqual(
            result['selection_assessment']['unresolved_competitor_count'],
            len(result['unresolved_competitors_local_m'])
        )

    def test_candidate_budget_and_no_candidate(self):
        a=solve_b2({'x':0,'y':0,'svd_deg':0},options={'max_candidates':2,'refinement_levels':0})
        self.assertEqual(len(a['candidates']),2)
        self.assertTrue(a['search']['candidate_budget_reached'])
        b=solve_b2({'x':0,'y':0,'svd_deg':0},options={'candidate_points_local':[[500,0]],'refinement_levels':0})
        self.assertEqual(b['status'],'no_candidate_in_search')
        self.assertIsNone(b['recommended'])

    def test_json_and_svg_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            r=run_file(BASE/'examples/central_synthetic.json',directory)
            d=Path(directory)
            loaded=json.loads((d/'central_synthetic.json').read_text(encoding='utf-8'))
            self.assertEqual(loaded['status'],r['status'])
            ET.parse(d/'central_synthetic.svg')
            self.assertNotIn('NaN',(d/'central_synthetic.json').read_text(encoding='utf-8'))
        with self.assertRaises(ValueError): run_file(BASE/'examples/central_synthetic.json',BASE/'examples')

    def test_b1_dependency_hashes_unchanged(self):
        manifest=json.loads((BASE/'b1_dependency_manifest.json').read_text(encoding='utf-8'))
        text_suffixes={'.py','.md','.json','.txt','.csv','.yaml','.yml','.svg'}
        for entry in manifest['files']:
            path=ROOT/entry['path']
            self.assertTrue(path.is_file(),entry['path'])
            data=path.read_bytes()
            if path.suffix.lower() in text_suffixes:
                data=data.replace(b'\r\n',b'\n').replace(b'\r',b'\n')
            self.assertEqual(hashlib.sha256(data).hexdigest(),entry['sha256'])


if __name__=='__main__': unittest.main()
