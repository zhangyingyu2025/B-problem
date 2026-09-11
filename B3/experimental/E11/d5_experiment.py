from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time,json
spec=importlib.util.spec_from_file_location('d4',str(HERE/'d4_experiment.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=m.d

class D5Solver(m.D4Solver):
    def segment_points(self,t,a,b,maxpts=2):
        if not t['dirs'] or self.localized(t): return []
        vals=[];first=t['dirs'][0]
        for k in range(1,60):
            u=k/60;p=(a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]))
            if self.effective_from_first(t,p):
                th=math.radians(first['svd_deg']);proxy=(first['x']+750*math.cos(th),first['y']+750*math.sin(th))
                br=math.degrees(math.atan2(proxy[1]-p[1],proxy[0]-p[0]))%360;sep=d.angle_diff(br,first['svd_deg'])
                vals.append((u,p,sep))
        if not vals:return []
        # choose points maximizing angular diversity / segment spacing. First max sep, second farthest u from first among strong candidates.
        firstv=max(vals,key=lambda z:z[2]);out=[firstv]
        if maxpts>1:
            cand=[z for z in vals if abs(z[0]-firstv[0])>=0.12]
            if cand:out.append(max(cand,key=lambda z:(abs(z[0]-firstv[0]),z[2])))
        return sorted(out)
    def run_search(self):
        for idx,site in enumerate(self.sites):
            if self.discovery_complete():break
            unknown=[c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<7]
            unknown=sorted(unknown,reverse=bool(idx%2))
            if self.env.channel in unknown:unknown.remove(self.env.channel);unknown.insert(0,self.env.channel)
            for c in unknown:
                if self.discovery_complete():break
                self.measure(site,c,idx,'discovery')
                if c in self.tracks:self.maybe_clear_near(self.tracks[c])
            if self.discovery_complete() or idx+1>=len(self.sites):continue
            nxt=self.sites[idx+1];planned=[]
            for c,t in self.tracks.items():
                if c in self.env.cleared or self.localized(t):continue
                for u,p,sep in self.segment_points(t,site,nxt,2): planned.append((u,c,p,sep))
            planned.sort(key=lambda z:z[0])
            used=set()
            for u,c,p,sep in planned:
                key=(c,round(u,4))
                if key in used:continue
                self.transit_attempts+=1;res=self.measure(p,c,None,'zero_detour_multi_transit');used.add(key)
                if res['measure_result']=='direction':self.transit_dirs+=1
                self.maybe_clear_near(self.tracks[c])

def run():
 rows=[]
 for i,c in enumerate(d.cases12()):
  s=D5Solver(c);t=time.perf_counter();s.run_search();s.finish();r={'seed':c['seed'],'pattern':c['pattern'],'cleared':len(s.env.cleared),'total':len(s.env.sources),'vt':s.env.vt,'movement':s.env.stats['movement_m'],**s.env.stats,**s.stats,'transit_attempts':s.transit_attempts,'transit_dirs':s.transit_dirs,'estimate_clear_fail':s.estimate_clear_fail,'recovery_measures':s.recovery_measures,'wall':time.perf_counter()-t};rows.append(r);print(i+1,r)
 print('all',all(r['cleared']==r['total'] for r in rows));print('mean total',statistics.mean(r['vt'] for r in rows));print('mean per',statistics.mean(r['vt']/r['total'] for r in rows));print('weighted',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows));print('move',statistics.mean(r['movement'] for r in rows));print('meas',statistics.mean(r['measurements'] for r in rows));print('fails',sum(r['clear_failures'] for r in rows))
 json.dump(rows,open(str(HERE/'results'/'d5_results.json'),'w'),indent=2)
if __name__=='__main__':run()
