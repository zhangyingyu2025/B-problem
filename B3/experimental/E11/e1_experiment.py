from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import sys, math, statistics, time, importlib.util, pathlib, json
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'B3/code'));sys.path.insert(0,str(ROOT/'B1/code'))
from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client
import b1_geometry as b1g

# Load D7 prototype inheritance tree.
spec=importlib.util.spec_from_file_location('d7',str(HERE/'d7_experiment.py'))
d7=importlib.util.module_from_spec(spec);spec.loader.exec_module(d7)
d=d7.d

# Replace prototype polygon geometry with the repository's exact B1 geometry.
def exact_track_geometry(track):
    if not track['dirs']:
        track['poly']=[];track['mec']=None;return None
    res=b1g.solve_b1(track['dirs'])
    if res['status'] in ('bounded','point','segment') and res.get('minimum_enclosing_circle'):
        z=res['minimum_enclosing_circle'];track['poly']=[tuple(v) for v in res['vertices']];track['mec']=(tuple(z['center']),z['radius_m'])
    else:
        track['poly']=[];track['mec']=None
    return track['mec']
d.track_geometry=exact_track_geometry
d.clearable=lambda t:t.get('mec') is not None and t['mec'][1]<=19.999

class OfficialEnv:
    """Adapter using the repository's OfflineTransport + protocol Client."""
    def __init__(self,case,error_mode=None):
        if error_mode is None:error_mode='endpoints' if case.get('pattern')=='boundary' else 'fixed_field'
        self.t=OfflineTransport(case,error_mode=error_mode)
        self.c=Client(self.t,'offline-test',session_id='e1-'+str(case['seed']))
        self.c.enter();self.cleared=set();self.sources={s['channel']:dict(s) for s in case['sources']};self.seed=case['seed']
    @property
    def pos(self):return self.c.position
    @property
    def channel(self):return self.c.channel
    @property
    def vt(self):return self.c.virtual_time_s
    @property
    def stats(self):return self.c.stats
    def measure(self,p,ch):
        b=self.c.measure(p,ch,'e1')
        return {'measure_result':b['measure_result'],**({'svd_deg':b['svd_deg']} if b['measure_result']=='direction' else {})}
    def clear(self,p,ch):
        b=self.c.clear(p,ch,'e1');ok=b['clear_result']=='success'
        if ok:self.cleared.add(ch)
        return ok

class E1Solver(d7.D7Solver):
    """Joint discovery/localization/clearance prototype.

    The seven-site discovery certificate remains unchanged. Between required
    coverage sites, already-bounded sources may be cleared when their insertion
    detour is cheap. Every movement leg is also reused for guaranteed-safe
    bearing measurements of other unresolved tracks.
    """
    def __init__(self,case,insert_limit_m=340.0,radius_limit_m=180.0,max_insert_per_edge=4,
                 detour_radius_weight=0.55,measure_on_clear_legs=True):
        super().__init__(case)
        self.env=OfficialEnv(case)
        self.insert_limit_m=float(insert_limit_m);self.radius_limit_m=float(radius_limit_m)
        self.max_insert_per_edge=int(max_insert_per_edge);self.detour_radius_weight=float(detour_radius_weight)
        self.measure_on_clear_legs=measure_on_clear_legs
        self.joint_insertions=0;self.joint_detour_nominal_m=0.0;self.joint_transit_attempts=0;self.joint_transit_dirs=0

    def transit_measure(self,a,b,exclude=None,reason='joint_transit'):
        if math.dist(a,b)<1e-8:return
        planned=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or c==exclude or self.localized(t):continue
            for u,p,sep in self.segment_points(t,a,b,2):planned.append((u,c,p,sep))
        planned.sort(key=lambda z:z[0])
        # one or two points per source may appear; protect exact duplicate actions
        seen=set()
        for u,c,p,sep in planned:
            key=(c,round(p[0],6),round(p[1],6))
            if key in seen:continue
            seen.add(key);self.transit_attempts+=1;self.joint_transit_attempts+=1
            res=self.measure(p,c,None,reason)
            if res['measure_result']=='direction':self.transit_dirs+=1;self.joint_transit_dirs+=1
            self.maybe_clear_near(self.tracks[c])

    def insertion_candidate(self,nxt):
        cur=self.env.pos;cands=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t['dirs']:continue
            mc=self.mec_info(t)
            if mc is None:continue
            center,R=mc
            if R>self.radius_limit_m:continue
            delta=math.dist(cur,center)+math.dist(center,nxt)-math.dist(cur,nxt)
            effective=delta+self.detour_radius_weight*R
            if effective<=self.insert_limit_m:
                cands.append((effective,delta,R,c,center))
        return min(cands) if cands else None

    def insert_along_edge(self,nxt):
        count=0
        while count<self.max_insert_per_edge:
            z=self.insertion_candidate(nxt)
            if z is None:break
            _,delta,R,c,center=z
            start=self.env.pos
            if self.measure_on_clear_legs:
                self.transit_measure(start,center,exclude=c,reason='joint_to_clear')
                # transit measurements may change other tracks but not owner's geometry
            # owner geometry/center remains valid; local recovery guaranteed for R<=500
            before=self.env.pos
            ok=self.local_clear_from_mec(self.tracks[c],self.radius_limit_m)
            if not ok:break
            self.joint_insertions+=1;self.joint_detour_nominal_m+=delta;count+=1
        # Reuse the final leg toward the next mandatory coverage site.
        if self.measure_on_clear_legs:self.transit_measure(self.env.pos,nxt,reason='joint_to_cover')

    def run_search(self):
        for idx,site in enumerate(self.sites):
            if self.discovery_complete():break
            # First action at site pays any residual movement from prior leg.
            unknown=[c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<7]
            unknown=sorted(unknown,reverse=bool(idx%2))
            if self.env.channel in unknown:unknown.remove(self.env.channel);unknown.insert(0,self.env.channel)
            for c in unknown:
                if self.discovery_complete():break
                self.measure(site,c,idx,'discovery')
                if c in self.tracks:self.maybe_clear_near(self.tracks[c])
            if self.discovery_complete() or idx+1>=len(self.sites):continue
            self.insert_along_edge(self.sites[idx+1])

    def run_all(self):
        self.run_search();self.finish();return self


def run_case(case,**kw):
    s=E1Solver(case,**kw);t=time.perf_counter();s.run_all()
    return {'seed':case['seed'],'pattern':case.get('pattern','random'),'n':len(case['sources']),
            'cleared':len(s.env.cleared),'vt':s.env.vt,'movement':s.env.stats['movement_m'],
            'measurements':s.env.stats['measurements'],'switches':s.env.stats['switches'],
            'clear_attempts':s.env.stats['clear_attempts'],'clear_failures':s.env.stats['clear_failures'],
            'joint_insertions':s.joint_insertions,'joint_detour_nominal_m':s.joint_detour_nominal_m,
            'joint_transit_attempts':s.joint_transit_attempts,'joint_transit_dirs':s.joint_transit_dirs,
            'wall':time.perf_counter()-t}

def summary(rows):
    return {'cases':len(rows),'sources':sum(r['n'] for r in rows),'all_clear':all(r['cleared']==r['n'] for r in rows),
            'weighted_per_source':sum(r['vt'] for r in rows)/sum(r['n'] for r in rows),
            'mean_per_source':statistics.mean(r['vt']/r['n'] for r in rows),
            'mean_total':statistics.mean(r['vt'] for r in rows),'mean_move':statistics.mean(r['movement'] for r in rows),
            'mean_measure':statistics.mean(r['measurements'] for r in rows),'mean_insertions':statistics.mean(r['joint_insertions'] for r in rows),
            'max_per_source':max(r['vt']/r['n'] for r in rows)}

if __name__=='__main__':
    rows=[]
    for seed in range(20262100,20262120):
        r=run_case(make_case(seed));rows.append(r);print(r,flush=True)
    print(summary(rows))
