from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time,json
spec=importlib.util.spec_from_file_location('d3',str(HERE/'d3_experiment.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=m.d

class D4Solver(m.D3Solver):
    def __init__(self,case): super().__init__(case,particle_step=2.5,ct=18,max_transit_obs=3);self.estimate_clear_fail=0;self.recovery_measures=0
    def box(self,t): return self.phys(t)
    def estimate_items(self,maxr=65):
        out=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            b=self.box(t)
            if b and b[1]<=maxr: out.append((c,b[0]))
        return out
    def clear_estimates(self,maxr=65,recover=True):
        # global open TSP through estimated source centers
        for c,p in d.open_tsp_order(self.env.pos,self.estimate_items(maxr)):
            if c in self.env.cleared:continue
            if self.env.clear(p,c):continue
            self.estimate_clear_fail+=1
            if not recover:continue
            # Measurement at failed estimate costs no movement and should be very informative if estimate is reasonably tight
            res=self.measure(p,c,None,'failed_estimate_remeasure');self.recovery_measures+=1
            if res['measure_result']=='near':
                self.env.clear(p,c);continue
            b=self.box(self.tracks[c])
            if b and b[1]<=35:
                if self.env.clear(b[0],c):continue
            # if still uncertain, don't thrash here
    def owner_supp_round_for_far(self,threshold=65):
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            b=self.box(t)
            if b and b[1]<=threshold:continue
            opts=self.owner_candidates(t)
            if not opts:continue
            # choose sign based on shortest from current only; global TSP fixes order, not sign
            p=min(opts,key=lambda q:math.dist(self.env.pos,q));items.append((c,p))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'far_owner_supp');self.stats['supplemental_points']+=1
    def local_recover(self,t):
        c=t['channel'];b=self.box(t)
        if not b:return False
        center,r=b
        # try current estimate first (if not already close to current failed point)
        if self.env.clear(center,c):return True
        self.estimate_clear_fail+=1
        res=self.measure(center,c,None,'local_recovery_measure');self.recovery_measures+=1
        if res['measure_result']=='near': return self.env.clear(center,c)
        b2=self.box(t)
        if b2 and self.env.clear(b2[0],c):return True
        # short ray-cover using fresh direction and current feasible radius estimate; cap at 120m before full fallback
        if res['measure_result']=='direction':
            th=math.radians(res['svd_deg']);u=(math.cos(th),math.sin(th))
            # clear failure at center means >20. cover forward sector with centers 35,65,95m; 1deg lateral error tiny
            for rr in (35,65,95):
                p=(center[0]+rr*u[0],center[1]+rr*u[1])
                if self.env.clear(p,c):return True
        return False
    def full_grid(self,t):
        c=t['channel'];o=t['dirs'][0];th=math.radians(o['svd_deg']);e=(math.cos(th),math.sin(th));n=(-e[1],e[0]);h=1500*math.sin(math.radians(1));nx=54
        xs=[(i+.5)*1500/nx for i in range(nx)];ys=[-h/2,h/2];route=[]
        for j,y in enumerate(ys):
            cols=xs if j%2==0 else list(reversed(xs));route += [(o['x']+x*e[0]+y*n[0],o['y']+x*e[1]+y*n[1]) for x in cols]
        if math.dist(self.env.pos,route[-1])<math.dist(self.env.pos,route[0]):route.reverse()
        for p in route:
            if self.env.clear(p,c):return True
        return False
    def finish(self):
        # 1) cheap attempt for moderately localized tracks, before any dedicated detour
        self.clear_estimates(65,True)
        # 2) one globally routed supplement for only truly broad tracks
        self.owner_supp_round_for_far(65)
        # 3) now most tracks should have tight centers; global TSP clear them
        self.clear_estimates(90,True)
        # 4) remaining: one extra owner supplement, globally routed
        leftovers=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared]
        items=[]
        for t in leftovers:
            opts=self.owner_candidates(t)
            if opts:items.append((t['channel'],min(opts,key=lambda q:math.dist(self.env.pos,q))))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'last_owner_supp');self.stats['supplemental_points']+=1
        # 5) TSP clear centers, then local recovery, then guaranteed grid
        self.clear_estimates(200,True)
        for t in self.tracks.values():
            if t['channel'] in self.env.cleared:continue
            if self.local_recover(t):continue
            self.full_grid(t)

def run():
 rows=[]
 for i,c in enumerate(d.cases12()):
  s=D4Solver(c);t=time.perf_counter();s.run_search();s.finish();r={'seed':c['seed'],'pattern':c['pattern'],'cleared':len(s.env.cleared),'total':len(s.env.sources),'vt':s.env.vt,'movement':s.env.stats['movement_m'],**s.env.stats,**s.stats,'transit_attempts':s.transit_attempts,'transit_dirs':s.transit_dirs,'estimate_clear_fail':s.estimate_clear_fail,'recovery_measures':s.recovery_measures,'wall':time.perf_counter()-t};rows.append(r);print(i+1,r)
 print('all',all(r['cleared']==r['total'] for r in rows));print('mean total',statistics.mean(r['vt'] for r in rows));print('mean per',statistics.mean(r['vt']/r['total'] for r in rows));print('weighted',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows));print('move',statistics.mean(r['movement'] for r in rows));print('meas',statistics.mean(r['measurements'] for r in rows));print('fails',sum(r['clear_failures'] for r in rows))
 json.dump(rows,open(str(HERE/'results'/'d4_results.json'),'w'),indent=2)
if __name__=='__main__':run()
