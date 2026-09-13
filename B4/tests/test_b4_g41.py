import ast
from dataclasses import replace
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B4/code'));sys.path.insert(0,str(ROOT/'B3/code'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import jcr_cost as j
from solver_g35 import G40PassiveSafe, _strip_polygon, _local, chord_radial_upper, safe_rho
from solver_g41 import G41JCR
from solver_g0 import RobotPort
from offline_environment_b4 import make_case, OfflineTransportB4
from protocol_adapter import Client
from test_b4_source_blind import ReplayPort


OBS={'x':0.,'y':0.,'svd_deg':0.}


def pair():
    return j.resolver_pair(OBS,1,j.safe_site_info(OBS,(750.,400.)),2,j.safe_site_info(OBS,(750.,-400.)))


def actions(client):
    return [(e['path'],e['payload'].get('position'),e['payload'].get('channel'),e['response'],e['before'],e['after'])
            for e in client.events if e.get('kind')=='action']


def run(cls,seed,pattern='random_mixed',**kwargs):
    env=OfflineTransportB4(make_case(seed,count=10,pattern=pattern),error_mode='endpoints')
    client=Client(env,'offline-test');client.enter();solver=cls(RobotPort(client),**kwargs);solver.run_all();client.exit()
    return solver,client,env


class G41MathTests(unittest.TestCase):
    def test_equivalent_units_and_channel_state(self):
        self.assertEqual(j.action_eq('measure',3,3),25)
        self.assertEqual(j.action_eq('measure',3,4),30)
        self.assertEqual(j.action_eq('clear_fail'),15);self.assertEqual(j.action_eq('clear_success'),25)
        a=j.safe_site_info(OBS,(750.,400.),already_committed_measure=True)
        self.assertEqual(a.extra_measure_eq_m,0)

    def test_return_to_original_channel_is_also_budgeted(self):
        a=j.safe_site_info(OBS,(750.,400.),channel=3,current_channel=4)
        self.assertEqual(a.extra_measure_eq_m+a.resume_switch_eq_m,35.)
        same=j.safe_site_info(OBS,(750.,400.),channel=3,current_channel=3)
        self.assertEqual(same.extra_measure_eq_m+same.resume_switch_eq_m,25.)
        b=j.safe_site_info(OBS,(750.,-400.),channel=3,current_channel=4)
        p=j.resolver_pair(OBS,1,a,2,b)
        self.assertEqual(p.branch_costs_eq_m[0],35.+a.dir_completion_eq_m)
        self.assertEqual(p.branch_costs_eq_m[1],70.+b.dir_completion_eq_m)

    def test_unsafe_site_is_never_backside_certificate(self):
        info=j.safe_site_info(OBS,(750.,900.))
        self.assertFalse(info.safe);self.assertEqual(info.side,0);self.assertIsNone(info.predicted_dir_clear_count)
        self.assertIsNone(j.resolver_pair(OBS,1,info,2,j.safe_site_info(OBS,(750.,-400.))))

    def test_dense_adversarial_chord_bound(self):
        rng=random.Random(64241990)
        for _ in range(80):
            up=(rng.uniform(650,850),rng.uniform(220,400));lo=(rng.uniform(650,850),-rng.uniform(220,400))
            self.assertLessEqual(safe_rho(OBS,up),1000);self.assertLessEqual(safe_rho(OBS,lo),1000)
            bound=chord_radial_upper(OBS,up,lo)
            for k in range(101):
                a=math.radians(-1+2*k/100);u=(math.cos(a),math.sin(a));dx,dy=lo[0]-up[0],lo[1]-up[1]
                r=(up[0]*lo[1]-up[1]*lo[0])/(u[0]*dy-u[1]*dx)
                self.assertLessEqual(r,bound+1e-8)
                # Beyond chord, source cannot see O while rejecting both endpoints:
                # O-G is a positive combination of endpoint-G (solve explicitly).
                R=r+1.;g=(R*u[0],R*u[1]);a0=(up[0]-g[0],up[1]-g[1]);b0=(lo[0]-g[0],lo[1]-g[1])
                det=a0[0]*b0[1]-a0[1]*b0[0]
                ca=(-g[0]*b0[1]+g[1]*b0[0])/det;cb=(-a0[0]*g[1]+a0[1]*g[0])/det
                self.assertGreaterEqual(ca,-1e-8);self.assertGreaterEqual(cb,-1e-8)

    def test_all_explicit_branches_below_worst(self):
        p=pair()
        self.assertIsNotNone(p)
        for cost in p.branch_costs_eq_m:self.assertLessEqual(cost,p.worst_case_eq_m)
        self.assertEqual(p.worst_case_eq_m,max(p.branch_costs_eq_m))

    def test_direction_predictions_and_dense_outer_cover(self):
        for q in ((600.,300.),(750.,400.),(900.,250.),(900.,-250.)):
            info=j.safe_site_info(OBS,q);self.assertIsNotNone(info.predicted_dir_clear_count)
            for r in (0.,20.,500.,1000.,1500.):
                for e in (-1.,0.,1.):
                    src=(r*math.cos(math.radians(e)),r*math.sin(math.radians(e)))
                    for err in (-1.,1.):
                        bearing=(math.degrees(math.atan2(src[1]-q[1],src[0]-q[0]))+err)%360
                        body={'svd_deg':bearing};z=j.direction_plan(OBS,info,body,q)
                        self.assertIsNotNone(z);self.assertLessEqual(z['count'],info.predicted_dir_clear_count)
                        self.assertLessEqual(z['worst_eq_m'],info.dir_completion_eq_m+1e-6)
                        second={'x':q[0],'y':q[1],'svd_deg':bearing}
                        poly=_strip_polygon(OBS,1500.,second,info.rho_m+3e-6)
                        for i in range(9):
                            u=i/8
                            for k in range(9):
                                v=k/8
                                p=tuple((1-u)*(1-v)*poly[0][d]+u*(1-v)*poly[1][d]+u*v*poly[2][d]+(1-u)*v*poly[3][d] for d in (0,1))
                                self.assertLess(min(math.dist(p,c) for c in z['centers']),20.)

    def test_dense_radial_cover(self):
        p=pair();z=j.radial_centers(OBS,p.chord_radius_upper_m,p.second.point)
        for i in range(121):
            for k in range(21):
                r=p.chord_radius_upper_m*i/120;a=math.radians(-1+k/10)
                q=(r*math.cos(a),r*math.sin(a))
                self.assertLess(min(math.dist(q,c) for c in z['centers']),20.)

    def test_only_strict_pair_improvement_can_defer(self):
        p=pair();w=p.worst_case_eq_m
        self.assertTrue(j.should_defer('fan',10,p,w+1,[1,2]))
        self.assertFalse(j.should_defer('fan',10,p,w,[1,2]))
        self.assertFalse(j.should_defer('guaranteed',10,p,w+1,[1,2]))
        self.assertFalse(j.should_defer('fan',10,p,w+1,[1]))
        self.assertFalse(j.should_defer('fan',16,p,w+1,[1,2]))

    def test_active_completion_bound_is_finite(self):
        t=SimpleNamespace(near=None,mec=((750.,0.),751.),dirs=[OBS],fan_steps=0)
        a=j.active_resolver_worst_case_eq_m(t,(0.,0.));self.assertTrue(math.isfinite(a))
        t.fan_steps=8;self.assertLess(j.active_resolver_worst_case_eq_m(t,(0.,0.)),a)


class G41RouteTests(unittest.TestCase):
    def test_no_pair_ranking_is_length_ranking(self):
        pts={1:(10.,0.),2:(40.,1.),3:(5.,8.)};a={3:8000.}
        for order in ([1,2,3],[3,2,1],[2,1,3]):
            score,sel=j.objective((0.,0.),order,pts,a,{})
            self.assertEqual(sel,{});self.assertAlmostEqual(score-j.route_length((0.,0.),order,pts),8000.)

    def test_equal_length_prefers_cheaper_ordered_pair(self):
        p=replace(pair(),worst_case_eq_m=100.);reverse=replace(p,site_i=2,site_j=1,worst_case_eq_m=200.)
        points={1:(10.,0.),2:(-10.,0.)}
        a=j.objective((0.,0.),[1,2],points,{7:500.},{7:[p,reverse]})[0]
        b=j.objective((0.,0.),[2,1],points,{7:500.},{7:[p,reverse]})[0]
        self.assertLess(a,b)

    def test_longer_route_can_reduce_total_and_search_is_deterministic(self):
        p=replace(pair(),site_i=2,site_j=1,worst_case_eq_m=100.)
        points={1:(1.,0.),2:(20.,0.)}
        a=j.local_search((0.,0.),[1,2],points,{7:500.},{7:[p]})
        b=j.local_search((0.,0.),[1,2],points,{7:500.},{7:[p]})
        self.assertEqual(a,b);self.assertEqual(a['order'],[2,1])

    def test_16_removes_free_certificate_pairs(self):
        p=pair();value,sel=j.objective((0.,0.),[1,2],{1:(1.,0.),2:(2.,0.)},{7:9999.},{7:[p]},16)
        self.assertEqual(sel,{});self.assertEqual(value,10001.)


class G41ExecutionTests(unittest.TestCase):
    def test_disabled_equivalence_new_cases(self):
        for i,pattern in enumerate(('random_mixed','all_directional','boundary','cluster','origin','coverage_edge')):
            with self.subTest(pattern=pattern):
                _,a,_=run(G40PassiveSafe,64241900+i,pattern)
                _,b,_=run(G41JCR,64241900+i,pattern,jcr_enabled=False)
                self.assertEqual(actions(a),actions(b));self.assertEqual(a.stats,b.stats)

    def test_replay_source_blind_and_no_missing_certificate(self):
        s,client,env=run(G41JCR,64241910,'all_directional')
        self.assertEqual(len(env.cleared),10);self.assertEqual(s.stats['certificate_sites_visited'],21)
        for c in range(1,21):
            if c not in s.tracks:self.assertEqual(s.checked[c],set(range(21)))
        replay=ReplayPort(client.events);r=G41JCR(replay);r.run_all()
        self.assertEqual(replay.index,len(replay.events));self.assertEqual(s.cleared,r.cleared)
        for name in ('jcr_cost.py','solver_g41.py'):
            tree=ast.parse((ROOT/'B4/code'/name).read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                if isinstance(node,ast.ImportFrom):self.assertNotIn('offline_environment',node.module or '')
                if isinstance(node,ast.Attribute):self.assertNotIn(node.attr,('_sources','sources','transport','case','seed','direction_deg'))

    def test_exhausted_certificate_logs_failure_and_falls_back(self):
        port=SimpleNamespace(record=lambda x:events.append(x));events=[]
        s=object.__new__(G41JCR);s.port=port;s.pending={};s.planned={}
        s.stats={'jcr_invariant_failures':0,'jcr_active_fallbacks':0,'jcr_certified_clear_attempts':0}
        s.clear=lambda *a,**k:False;s.fallback=lambda c:events.append({'fallback':c})
        s._execute_cover(7,{'centers':[(0.,0.)],'count':1,'worst_eq_m':25.},25.)
        self.assertEqual(s.stats['jcr_invariant_failures'],1)
        self.assertTrue(any(e.get('kind')=='G41_invariant_failure' for e in events));self.assertEqual(events[-1],{'fallback':7})

    def test_early_stop_cancels_pending(self):
        s=object.__new__(G41JCR);s.tracks={i:None for i in range(16)};s.cleared=set()
        s.pending={1:{'pair':pair()}};s.planned={2:pair()};s.port=SimpleNamespace(record=lambda e:None)
        s.stats={'jcr_pair_abandoned_after_early_stop':0};s.cancel_certificate_pairs()
        self.assertEqual(s.pending,{});self.assertEqual(s.planned,{})
        self.assertEqual(s.stats['jcr_pair_abandoned_after_early_stop'],2)


if __name__=='__main__':unittest.main()
