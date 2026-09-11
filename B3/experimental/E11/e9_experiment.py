from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time
spec=importlib.util.spec_from_file_location('e6',str(HERE/'e6_experiment.py'));e6=importlib.util.module_from_spec(spec);spec.loader.exec_module(e6)
e5=e6.e5
from offline_environment import make_case

class E9Solver(e6.E6Solver):
    def route_fit_supp(self,t,structural):
        bank=self.safe_bank(t)
        if not bank:return None
        if not structural:return min(bank,key=lambda p:math.dist(self.env.pos,p))
        route=e5.fast_open_route(self.env.pos,structural)
        pts=[self.env.pos]+[x[2] for x in route]
        def insertion(p):
            best=math.dist(pts[-1],p) # append at free end
            for a,b in zip(pts,pts[1:]):
                best=min(best,math.dist(a,p)+math.dist(p,b)-math.dist(a,b))
            return best
        return min(bank,key=insertion)
    def joint_nodes(self,remaining):
        structural=[('cover',idx,p) for idx,p in remaining]+self.eligible_targets()
        nodes=list(structural)
        for t in self.broad_tracks():
            p=self.route_fit_supp(t,structural)
            if p is not None:nodes.append(('supp',t['channel'],p))
        return nodes

def run_case(case,**kw):
 s=E9Solver(case,**kw);t=time.perf_counter();s.run_all();return {'seed':case['seed'],'n':len(case['sources']),'cleared':len(s.env.cleared),'vt':s.env.vt,'movement':s.env.stats['movement_m'],'measurements':s.env.stats['measurements'],'clear_failures':s.env.stats['clear_failures'],'joint_supp':s.joint_supp,'wall':time.perf_counter()-t}
def summary(rs):return {'weighted':sum(r['vt'] for r in rs)/sum(r['n'] for r in rs),'all':all(r['cleared']==r['n'] for r in rs),'meanmove':statistics.mean(r['movement'] for r in rs),'meanmeas':statistics.mean(r['measurements'] for r in rs),'meansupp':statistics.mean(r['joint_supp'] for r in rs),'maxper':max(r['vt']/r['n'] for r in rs)}
