from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import importlib.util,math,statistics,time
spec=importlib.util.spec_from_file_location('e1',str(HERE/'e1_experiment.py'));e1=importlib.util.module_from_spec(spec);spec.loader.exec_module(e1)
d7=e1.d7;d=e1.d
from offline_environment import make_case

def route_cost(start,nodes):
    p=start;z=0
    for _,_,q in nodes:z+=math.dist(p,q);p=q
    return z

def fast_open_route(start,nodes):
    nodes=list(nodes)
    if len(nodes)<=1:return nodes
    # multi-start nearest-neighbor then open 2-opt
    best=None
    starts=sorted(range(len(nodes)),key=lambda i:math.dist(start,nodes[i][2]))[:min(5,len(nodes))]
    for first in starts:
        rem=set(range(len(nodes)));rem.remove(first);order=[first];p=nodes[first][2]
        while rem:
            j=min(rem,key=lambda i:math.dist(p,nodes[i][2]));rem.remove(j);order.append(j);p=nodes[j][2]
        improved=True;passes=0
        while improved and passes<6:
            improved=False;passes+=1
            # reverse a subsequence; open path has fixed start but free end
            for i in range(len(order)-1):
                a=start if i==0 else nodes[order[i-1]][2];b=nodes[order[i]][2]
                for j in range(i+1,len(order)):
                    c=nodes[order[j]][2];dpt=nodes[order[j+1]][2] if j+1<len(order) else None
                    old=math.dist(a,b)+(math.dist(c,dpt) if dpt else 0)
                    new=math.dist(a,c)+(math.dist(b,dpt) if dpt else 0)
                    if new+1e-9<old:
                        order[i:j+1]=reversed(order[i:j+1]);improved=True
            
        seq=[nodes[i] for i in order];c=route_cost(start,seq)
        if best is None or c<best[0]:best=(c,seq)
    return best[1]

class E5Solver(d7.D7Solver):
    def __init__(self,case,phase_index=4,radius_limit=500,transit=True):
        super().__init__(case);self.env=e1.OfficialEnv(case);self.phase_index=phase_index;self.radius_limit=radius_limit;self.transit=transit;self.joint_clears=0;self.joint_steps=0;self.joint_transit=0
    def scan_site(self,idx,site):
        unknown=sorted([c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<7],reverse=bool(idx%2))
        if self.env.channel in unknown:unknown.remove(self.env.channel);unknown.insert(0,self.env.channel)
        for c in unknown:
            if self.discovery_complete():break
            self.measure(site,c,idx,'discovery')
            if c in self.tracks:self.maybe_clear_near(self.tracks[c])
    def transit_measure(self,a,b,exclude=None):
        if not self.transit or math.dist(a,b)<1e-7:return
        planned=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or c==exclude or self.localized(t):continue
            for u,p,sep in self.segment_points(t,a,b,2):planned.append((u,c,p))
        planned.sort()
        seen=set()
        for u,c,p in planned:
            key=(c,round(p[0],6),round(p[1],6))
            if key in seen:continue
            seen.add(key);self.transit_attempts+=1;self.joint_transit+=1
            res=self.measure(p,c,None,'joint_route_transit')
            if res['measure_result']=='direction':self.transit_dirs+=1
            self.maybe_clear_near(self.tracks[c])
    def eligible_targets(self):
        out=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            mc=self.mec_info(t)
            if mc and mc[1]<=self.radius_limit:out.append(('clear',c,mc[0]))
        return out
    def joint_phase(self,remaining_cover):
        remaining=list(remaining_cover)
        guard=0
        while remaining or self.eligible_targets():
            guard+=1
            if guard>100:raise RuntimeError('joint route guard')
            nodes=[('cover',idx,p) for idx,p in remaining]+self.eligible_targets()
            if not nodes:break
            route=fast_open_route(self.env.pos,nodes);kind,key,p=route[0];start=self.env.pos
            self.transit_measure(start,p,exclude=key if kind=='clear' else None)
            if kind=='cover':
                # transit actions may end before p; discovery measurement moves the rest
                self.scan_site(key,p);remaining=[x for x in remaining if x[0]!=key]
            else:
                if key not in self.env.cleared and self.local_clear_from_mec(self.tracks[key],self.radius_limit):self.joint_clears+=1
            self.joint_steps+=1
            # If 16 found, no remaining coverage sites are needed.
            if self.discovery_complete() and len(self.tracks)>=16:remaining=[]
            # Once all mandatory sites are done, only clear currently eligible; finish handles broad leftovers.
            if not remaining and self.discovery_complete() and not self.eligible_targets():break
    def run_search(self):
        # conventional guaranteed route until phase_index inclusive, with D5 zero-detour measurements.
        for idx,site in enumerate(self.sites):
            if self.discovery_complete():break
            self.scan_site(idx,site)
            if self.discovery_complete():break
            if idx>=self.phase_index:
                self.joint_phase(list(enumerate(self.sites[idx+1:],start=idx+1)));return
            if idx+1<len(self.sites):
                nxt=self.sites[idx+1];planned=[]
                for c,t in self.tracks.items():
                    if c in self.env.cleared or self.localized(t):continue
                    for u,p,sep in self.segment_points(t,site,nxt,2):planned.append((u,c,p))
                planned.sort();seen=set()
                for u,c,p in planned:
                    key=(c,round(p[0],6),round(p[1],6))
                    if key in seen:continue
                    seen.add(key);self.transit_attempts+=1;res=self.measure(p,c,None,'pre_joint_transit')
                    if res['measure_result']=='direction':self.transit_dirs+=1
                    self.maybe_clear_near(self.tracks[c])
    def run_all(self):self.run_search();self.finish();return self

def run_case(case,**kw):
 s=E5Solver(case,**kw);t=time.perf_counter();s.run_all();return {'seed':case['seed'],'n':len(case['sources']),'cleared':len(s.env.cleared),'vt':s.env.vt,'movement':s.env.stats['movement_m'],'measurements':s.env.stats['measurements'],'switches':s.env.stats['switches'],'clear_failures':s.env.stats['clear_failures'],'joint_clears':s.joint_clears,'joint_steps':s.joint_steps,'wall':time.perf_counter()-t}
def summary(rs):return {'weighted':sum(r['vt'] for r in rs)/sum(r['n'] for r in rs),'all':all(r['cleared']==r['n'] for r in rs),'meanmove':statistics.mean(r['movement'] for r in rs),'meanmeas':statistics.mean(r['measurements'] for r in rs),'meanclear':statistics.mean(r['joint_clears'] for r in rs),'maxper':max(r['vt']/r['n'] for r in rs)}
