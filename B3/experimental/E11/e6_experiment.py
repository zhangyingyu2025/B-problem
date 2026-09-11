from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time
spec=importlib.util.spec_from_file_location('e5',str(HERE/'e5_experiment.py'));e5=importlib.util.module_from_spec(spec);spec.loader.exec_module(e5)
e1=e5.e1;d7=e1.d7;d=e1.d
from offline_environment import make_case

class E6Solver(e5.E5Solver):
    def __init__(self,case,phase_index=3,radius_limit=500,max_joint_supp=2,transit=True):
        super().__init__(case,phase_index=phase_index,radius_limit=radius_limit,transit=transit)
        self.max_joint_supp=max_joint_supp;self.joint_supp_count={};self.joint_supp=0
    def broad_tracks(self):
        out=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            mc=self.mec_info(t)
            if mc and mc[1]<=self.radius_limit:continue
            if self.joint_supp_count.get(c,0)>=self.max_joint_supp:continue
            out.append(t)
        return out
    def supp_point(self,t,anchors):
        bank=self.safe_bank(t)
        if not bank:return None
        # Pick a guaranteed-safe point that fits the current global route rather than merely nearest current.
        # Approximate insertion cost between current and any existing anchor.
        if anchors:
            return min(bank,key=lambda p:min(math.dist(self.env.pos,p)+math.dist(p,a)-math.dist(self.env.pos,a) for a in anchors))
        return min(bank,key=lambda p:math.dist(self.env.pos,p))
    def joint_nodes(self,remaining):
        nodes=[('cover',idx,p) for idx,p in remaining]
        clears=self.eligible_targets();nodes+=clears
        anchors=[p for _,_,p in nodes]
        for t in self.broad_tracks():
            p=self.supp_point(t,anchors)
            if p is not None:nodes.append(('supp',t['channel'],p));anchors.append(p)
        return nodes
    def joint_phase(self,remaining_cover):
        remaining=list(remaining_cover);guard=0
        while True:
            guard+=1
            if guard>120:raise RuntimeError('joint route guard')
            nodes=self.joint_nodes(remaining)
            if not nodes:break
            route=e5.fast_open_route(self.env.pos,nodes);kind,key,p=route[0];start=self.env.pos
            self.transit_measure(start,p,exclude=key if kind in ('clear','supp') else None)
            if kind=='cover':
                self.scan_site(key,p);remaining=[x for x in remaining if x[0]!=key]
                if self.discovery_complete() and len(self.tracks)>=16:remaining=[]
            elif kind=='clear':
                if key not in self.env.cleared and self.local_clear_from_mec(self.tracks[key],self.radius_limit):self.joint_clears+=1
            else:
                if key not in self.env.cleared:
                    self.measure(p,key,None,'joint_safe_supplement');self.stats['supplemental_points']+=1;self.joint_supp+=1
                    self.joint_supp_count[key]=self.joint_supp_count.get(key,0)+1;self.maybe_clear_near(self.tracks[key])
            self.joint_steps+=1
            # stop only after discovery completed and no clearable or allowed broad targets remain
            if not remaining and self.discovery_complete() and not self.eligible_targets() and not self.broad_tracks():break
    def run_all(self):self.run_search();self.finish();return self

def run_case(case,**kw):
 s=E6Solver(case,**kw);t=time.perf_counter();s.run_all();return {'seed':case['seed'],'n':len(case['sources']),'cleared':len(s.env.cleared),'vt':s.env.vt,'movement':s.env.stats['movement_m'],'measurements':s.env.stats['measurements'],'switches':s.env.stats['switches'],'clear_failures':s.env.stats['clear_failures'],'joint_clears':s.joint_clears,'joint_supp':s.joint_supp,'joint_steps':s.joint_steps,'wall':time.perf_counter()-t}
def summary(rs):return {'weighted':sum(r['vt'] for r in rs)/sum(r['n'] for r in rs),'all':all(r['cleared']==r['n'] for r in rs),'meanmove':statistics.mean(r['movement'] for r in rs),'meanmeas':statistics.mean(r['measurements'] for r in rs),'meanclear':statistics.mean(r['joint_clears'] for r in rs),'meansupp':statistics.mean(r['joint_supp'] for r in rs),'maxper':max(r['vt']/r['n'] for r in rs)}
