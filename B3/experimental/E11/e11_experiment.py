from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time
spec=importlib.util.spec_from_file_location('e9',str(HERE/'e9_experiment.py'));e9=importlib.util.module_from_spec(spec);spec.loader.exec_module(e9)
e5=e9.e5
from offline_environment import make_case

class E11Solver(e9.E9Solver):
    def __init__(self,case,proxy_r=1000,rotation_step=5,rho=1130,**kw):
        super().__init__(case,phase_index=0,**kw);self.proxy_r=proxy_r;self.rotation_step=rotation_step;self.rho=rho;self.rotation=0
    def choose_rotation(self):
        proxies=[]
        for c,t in self.tracks.items():
            if not t['dirs']:continue
            o=t['dirs'][0];a=math.radians(o['svd_deg']);proxies.append((o['x']+self.proxy_r*math.cos(a),o['y']+self.proxy_r*math.sin(a)))
        best=None
        for deg in range(0,60,self.rotation_step):
            outer=[(self.rho*math.cos(math.radians(deg+60*k)),self.rho*math.sin(math.radians(deg+60*k))) for k in range(6)]
            nodes=[('cover',k,p) for k,p in enumerate(outer)]+[('proxy',i,p) for i,p in enumerate(proxies)]
            cost=e5.route_cost((0.,0.),e5.fast_open_route((0.,0.),nodes))
            if best is None or cost<best[0]:best=(cost,deg,outer)
        self.rotation=best[1];return best[2]
    def run_search(self):
        self.scan_site(0,(0.,0.))
        if self.discovery_complete():return
        outer=self.choose_rotation();self.sites=[(0.,0.)]+outer
        self.joint_phase(list(enumerate(outer,start=1)))

def run_case(case,**kw):
 s=E11Solver(case,**kw);t=time.perf_counter();s.run_all();return {'seed':case['seed'],'n':len(case['sources']),'cleared':len(s.env.cleared),'vt':s.env.vt,'movement':s.env.stats['movement_m'],'measurements':s.env.stats['measurements'],'rotation':s.rotation,'joint_supp':s.joint_supp,'wall':time.perf_counter()-t}
def summary(rs):return {'weighted':sum(r['vt'] for r in rs)/sum(r['n'] for r in rs),'all':all(r['cleared']==r['n'] for r in rs),'meanmove':statistics.mean(r['movement'] for r in rs),'meanmeas':statistics.mean(r['measurements'] for r in rs),'maxper':max(r['vt']/r['n'] for r in rs),'rots':[r['rotation'] for r in rs]}
