from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time,json
spec=importlib.util.spec_from_file_location('d6',str(HERE/'d6_experiment.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=m.d

class D7Solver(m.D6Solver):
    def finish(self):
        # LOCALIZE FIRST. Do not leave the search route to clear already-localized targets yet.
        # Broad tracks (typically only one bearing) receive one guaranteed safe supplement each,
        # globally ordered. This avoids two separate tours through the target field.
        for round_no,threshold in [(1,120),(2,180)]:
            items=[]
            for c,t in self.tracks.items():
                if c in self.env.cleared or not t['dirs']: continue
                mc=self.mec_info(t)
                if mc is not None and mc[1] <= threshold: continue
                bank=self.safe_bank(t)
                if not bank: continue
                # choose candidate minimizing distance from current plus mild preference toward target-domain center
                p=min(bank,key=lambda q:math.dist(self.env.pos,q)+0.1*math.hypot(*q))
                items.append((c,p))
            for c,p in d.open_tsp_order(self.env.pos,items):
                self.measure(p,c,None,f'safe_supp_round_{round_no}');self.stats['supplemental_points']+=1;self.maybe_clear_near(self.tracks[c])
        # ONE GLOBAL CLEAR TOUR through all bounded MEC centers.
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared: continue
            mc=self.mec_info(t)
            if mc is not None and mc[1] <= 300: items.append((c,mc[0]))
        for c,_ in d.open_tsp_order(self.env.pos,items):
            if c not in self.env.cleared: self.local_clear_from_mec(self.tracks[c],300)
        # Rare broad leftovers: one final supplement then local clear; finite grid only last resort.
        leftovers=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared]
        items=[]
        for t in leftovers:
            bank=self.safe_bank(t)
            if bank: items.append((t['channel'],min(bank,key=lambda q:math.dist(self.env.pos,q))))
        for c,p in d.open_tsp_order(self.env.pos,items):
            self.measure(p,c,None,'final_safe_supp');self.stats['supplemental_points']+=1
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared: continue
            mc=self.mec_info(t)
            if mc: items.append((c,mc[0]))
        for c,_ in d.open_tsp_order(self.env.pos,items):
            if c not in self.env.cleared:
                mc=self.mec_info(self.tracks[c])
                if mc and mc[1] <= 500 and self.local_clear_from_mec(self.tracks[c],500): continue
        # absolute finite fallback
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']: continue
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
  s=D7Solver(c);t=time.perf_counter();s.run_search();s.finish();r={'seed':c['seed'],'pattern':c['pattern'],'cleared':len(s.env.cleared),'total':len(s.env.sources),'vt':s.env.vt,'movement':s.env.stats['movement_m'],**s.env.stats,**s.stats,'transit_attempts':s.transit_attempts,'transit_dirs':s.transit_dirs,'local_radial_attempts':s.local_radial_attempts,'center_failures':s.center_failures,'wall':time.perf_counter()-t};rows.append(r);print(i+1,r)
 print('all',all(r['cleared']==r['total'] for r in rows));print('mean total',statistics.mean(r['vt'] for r in rows));print('mean per',statistics.mean(r['vt']/r['total'] for r in rows));print('weighted',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows));print('move',statistics.mean(r['movement'] for r in rows));print('meas',statistics.mean(r['measurements'] for r in rows));print('fails',sum(r['clear_failures'] for r in rows))
 json.dump(rows,open(str(HERE/'results'/'d7_results.json'),'w'),indent=2)
if __name__=='__main__':run()
