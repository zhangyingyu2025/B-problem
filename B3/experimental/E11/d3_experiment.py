from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time,json
spec=importlib.util.spec_from_file_location('d',str(HERE/'d_experiment.py'));d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)

class D3Solver(d.DSolver):
    def __init__(self,case,particle_step=3,ct=18,max_transit_obs=3):
        super().__init__(case,aggressive=False,shared_rounds=0);self.pstep=particle_step;self.ct=ct;self.max_transit_obs=max_transit_obs;self.transit_attempts=0;self.transit_dirs=0
    def phys(self,t): return d.particle_bbox(t,self.pstep)[0]
    def localized(self,t):
        if d.clearable(t): return True
        box=self.phys(t); return box is not None and box[1]<=self.ct
    def effective_from_first(self,t,p):
        if not t['dirs']: return False
        o=t['dirs'][0];th=math.radians(o['svd_deg']);e=(math.cos(th),math.sin(th));n=(-e[1],e[0]);q=(p[0]-o['x'],p[1]-o['y']);a=q[0]*e[0]+q[1]*e[1];b=q[0]*n[0]+q[1]*n[1]
        alpha=math.radians(1);q2=a*a+b*b
        if q2>=1000**2-1e-6:return False
        if q2>=2000*(a*math.cos(alpha)-abs(b)*math.sin(alpha))-1e-6:return False
        threshold=1500*math.tan(alpha)+(1500-a)*math.tan(math.radians(3.1))
        if abs(b)<=threshold+1e-6:return False
        if abs(b)*math.cos(alpha)-a*math.sin(alpha)<=5+1e-6:return False
        return True
    def best_on_segment(self,t,a,b):
        if not t['dirs'] or len(t['dirs'])>=self.max_transit_obs+1:return None
        candidates=[]
        first=t['dirs'][0]
        for k in range(1,40):
            u=k/40
            p=(a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]))
            if not self.effective_from_first(t,p):continue
            # approximate geometry separation relative source proxy midpoint
            th=math.radians(first['svd_deg']);proxy=(first['x']+750*math.cos(th),first['y']+750*math.sin(th))
            bearing=math.degrees(math.atan2(proxy[1]-p[1],proxy[0]-p[0]))%360;sep=d.angle_diff(bearing,first['svd_deg'])
            # want large sep, moderate t so sorted schedule possible
            candidates.append((sep,u,p))
        return max(candidates,key=lambda z:z[0]) if candidates else None
    def run_search(self):
        for idx,site in enumerate(self.sites):
            # we always need current site unless discovery already complete; if 16 found can stop entire search
            if self.discovery_complete(): break
            unknown=[c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<7]
            unknown=sorted(unknown,reverse=bool(idx%2))
            if self.env.channel in unknown:unknown.remove(self.env.channel);unknown.insert(0,self.env.channel)
            for c in unknown:
                if self.discovery_complete():break
                self.measure(site,c,idx,'discovery');
                if c in self.tracks:self.maybe_clear_near(self.tracks[c])
            if self.discovery_complete(): break
            if idx+1>=len(self.sites):continue
            nxt=self.sites[idx+1]
            planned=[]
            for c,t in self.tracks.items():
                if c in self.env.cleared or self.localized(t):continue
                best=self.best_on_segment(t,site,nxt)
                if best: planned.append((best[1],c,best[2],best[0]))
            # sort along segment -> no extra movement beyond straight segment
            planned.sort()
            seen=set()
            for u,c,p,sep in planned:
                if c in seen:continue
                self.transit_attempts+=1;res=self.measure(p,c,None,'zero_detour_transit');seen.add(c)
                if res['measure_result']=='direction':self.transit_dirs+=1
                self.maybe_clear_near(self.tracks[c])
            # don't explicitly move to nxt; first discovery measurement there next loop accounts for remainder
    def owner_candidates(self,t):
        return [p for p in (self.supplemental_candidate(t,-1),self.supplemental_candidate(t,1)) if p]
    def finish(self):
        # One round owner supplements only for remaining unresolved, route globally; owner measurement only.
        unresolved=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared and not self.localized(t)]
        items=[]
        for t in unresolved:
            opts=self.owner_candidates(t)
            if not opts:continue
            # choose option closer to current plus estimated proxy
            first=t['dirs'][0];th=math.radians(first['svd_deg']);proxy=(first['x']+750*math.cos(th),first['y']+750*math.sin(th))
            p=min(opts,key=lambda q:math.dist(self.env.pos,q)+.3*math.dist(q,proxy));items.append((t['channel'],p))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'owner_supplement');self.stats['supplemental_points']+=1;self.maybe_clear_near(self.tracks[c])
        # recompute localization; a second supplement only for true leftovers
        unresolved=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared and not self.localized(t)]
        items=[]
        for t in unresolved:
            opts=self.owner_candidates(t)
            if opts:items.append((t['channel'],max(opts,key=lambda q:math.dist(q,(t['dirs'][0]['x'],t['dirs'][0]['y'])))))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'owner_supplement2');self.stats['supplemental_points']+=1;self.maybe_clear_near(self.tracks[c])
        # global TSP clear at physical-set center if tight, otherwise exact angle MEC
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared:continue
            box=self.phys(t)
            if box and box[1]<=self.ct:items.append((c,box[0]))
            elif d.clearable(t):items.append((c,t['mec'][0]))
        for c,p in d.open_tsp_order(self.env.pos,items):self.env.clear(p,c)
        # fallback only leftovers
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
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
  s=D3Solver(c);t=time.perf_counter();s.run_search();s.finish();r={'seed':c['seed'],'pattern':c['pattern'],'cleared':len(s.env.cleared),'total':len(s.env.sources),'vt':s.env.vt,'movement':s.env.stats['movement_m'],**s.env.stats,**s.stats,'transit_attempts':s.transit_attempts,'transit_dirs':s.transit_dirs,'wall':time.perf_counter()-t};rows.append(r);print(i+1,r)
 print('all',all(r['cleared']==r['total'] for r in rows));print('mean total',statistics.mean(r['vt'] for r in rows));print('mean per',statistics.mean(r['vt']/r['total'] for r in rows));print('weighted',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows));print('move',statistics.mean(r['movement'] for r in rows));print('meas',statistics.mean(r['measurements'] for r in rows));print('fails',sum(r['clear_failures'] for r in rows))
 json.dump(rows,open(str(HERE/'results'/'d3_results.json'),'w'),indent=2)
if __name__=='__main__':run()
