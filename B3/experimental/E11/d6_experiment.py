from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time,json
spec=importlib.util.spec_from_file_location('d5',str(HERE/'d5_experiment.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=m.d

class D6Solver(m.D5Solver):
    def __init__(self,case): super().__init__(case); self.local_radial_attempts=0;self.center_failures=0
    def safe_bank(self,t):
        if not t['dirs']: return []
        o=t['dirs'][0];th=math.radians(o['svd_deg']);e=(math.cos(th),math.sin(th));n=(-e[1],e[0]);alpha=math.radians(1);out=[]
        used=[(x['x'],x['y']) for x in t['dirs']]
        for a in (450,550,650,750,850,950):
            for babs in (300,400,500,600,700):
                for sign in (-1,1):
                    b=sign*babs;q=a*a+b*b
                    if q>=1000**2:continue
                    if q>=2000*(a*math.cos(alpha)-abs(b)*math.sin(alpha)):continue
                    threshold=1500*math.tan(alpha)+(1500-a)*math.tan(math.radians(3.1))
                    if abs(b)<=threshold:continue
                    if abs(b)*math.cos(alpha)-a*math.sin(alpha)<=5:continue
                    p=(o['x']+a*e[0]+b*n[0],o['y']+a*e[1]+b*n[1])
                    if any(math.dist(p,u)<1e-6 for u in used):continue
                    out.append(p)
        return out
    def mec_info(self,t):
        if not t.get('mec') and t['dirs']: d.track_geometry(t)
        return t.get('mec')
    def bounded_enough(self,t,Rmax=120):
        mc=self.mec_info(t); return mc is not None and mc[1]<=Rmax
    def local_clear_from_mec(self,t,Rmax=120):
        c=t['channel'];mc=self.mec_info(t)
        if mc is None or mc[1]>Rmax:return False
        center,R=mc
        if self.env.clear(center,c):return True
        self.center_failures+=1
        # Source is within R of center but farther than 20. Measure at center -> bearing with ±1°.
        res=self.measure(center,c,None,'mec_center_recovery')
        if res['measure_result']=='near':return self.env.clear(center,c)
        if res['measure_result']!='direction':return False
        th=math.radians(res['svd_deg']);u=(math.cos(th),math.sin(th))
        # q=35,70,... covers radial intervals with overlap >5m; ±1° lateral displacement is <~2.1m for R<=120.
        q=35.0
        while q<=R+20:
            p=(center[0]+q*u[0],center[1]+q*u[1]);self.local_radial_attempts+=1
            if self.env.clear(p,c):return True
            q+=35.0
        return False
    def clear_bounded_global(self,Rmax=120):
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared:continue
            mc=self.mec_info(t)
            if mc and mc[1]<=Rmax:items.append((c,mc[0]))
        # TSP order to centers, but local_clear will move along short ray; still good enough
        for c,_ in d.open_tsp_order(self.env.pos,items):
            if c not in self.env.cleared:self.local_clear_from_mec(self.tracks[c],Rmax)
    def finish(self):
        # Clear everything already bounded to a modest neighborhood, globally ordered.
        self.clear_bounded_global(120)
        # Any remaining tracks are broad (usually one bearing). One guaranteed supplemental measurement each.
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            bank=self.safe_bank(t)
            if bank:
                # choose closest to current as representative; TSP optimizes order
                p=min(bank,key=lambda q:math.dist(self.env.pos,q));items.append((c,p))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'single_safe_supplement');self.stats['supplemental_points']+=1;self.maybe_clear_near(self.tracks[c])
        # Second stage: almost all should now have bounded intersection. Allow up to 180m local neighborhood.
        self.clear_bounded_global(180)
        # rare broad leftovers get one second safe supplement then bounded clear
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            bank=self.safe_bank(t)
            if bank:items.append((c,min(bank,key=lambda q:math.dist(self.env.pos,q))))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'second_safe_supplement');self.stats['supplemental_points']+=1
        self.clear_bounded_global(300)
        # absolute finite fallback only if something pathological remains
        for c,t in self.tracks.items():
            if c in self.env.cleared and True:continue
            if not t['dirs']:continue
            # use inherited local recovery if mec <=500; otherwise finite grid
            mc=self.mec_info(t)
            if mc and mc[1]<=500 and self.local_clear_from_mec(t,500):continue
            o=t['dirs'][0];th=math.radians(o['svd_deg']);e=(math.cos(th),math.sin(th));n=(-e[1],e[0]);h=1500*math.sin(math.radians(1));nx=54
            xs=[(i+.5)*1500/nx for i in range(nx)];ys=[-h/2,h/2];route=[]
            for j,y in enumerate(ys):
                cols=xs if j%2==0 else list(reversed(xs));route += [(o['x']+x*e[0]+y*n[0],o['y']+x*e[1]+y*n[1]) for x in cols]
            if math.dist(self.env.pos,route[-1])<math.dist(self.env.pos,route[0]):route.reverse()
            for p in route:
                if self.env.clear(p,c):break

def run():
 rows=[]
 for i,c in enumerate(d.cases12()):
  s=D6Solver(c);t=time.perf_counter();s.run_search();s.finish();r={'seed':c['seed'],'pattern':c['pattern'],'cleared':len(s.env.cleared),'total':len(s.env.sources),'vt':s.env.vt,'movement':s.env.stats['movement_m'],**s.env.stats,**s.stats,'transit_attempts':s.transit_attempts,'transit_dirs':s.transit_dirs,'local_radial_attempts':s.local_radial_attempts,'center_failures':s.center_failures,'wall':time.perf_counter()-t};rows.append(r);print(i+1,r)
 print('all',all(r['cleared']==r['total'] for r in rows));print('mean total',statistics.mean(r['vt'] for r in rows));print('mean per',statistics.mean(r['vt']/r['total'] for r in rows));print('weighted',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows));print('move',statistics.mean(r['movement'] for r in rows));print('meas',statistics.mean(r['measurements'] for r in rows));print('fails',sum(r['clear_failures'] for r in rows))
 json.dump(rows,open(str(HERE/'results'/'d6_results.json'),'w'),indent=2)
if __name__=='__main__':run()
