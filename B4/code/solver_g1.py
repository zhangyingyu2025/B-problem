"""G1: replan the finite certificate/source task set after each observation."""
import math
from pathlib import Path
import sys

from solver_g0 import G0Solver, InvariantFailure
from b4_geometry import lower_distance
from paired_fan import paired_fan

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'B3/code'))
from e12_coverage_hook import E11


class G1Solver(G0Solver):
    def source_task(self,c):
        t=self.tracks[c]
        if t.near is not None: return ('near',c,t.near)
        if t.mec and t.mec[1]<=20-1e-6: return ('guaranteed',c,t.mec[0])
        if t.fan_steps>=self.fan_limit: return ('fallback',c,t.mec[0])
        anchor=t.last_signal_point
        lb=lower_distance(anchor,t.geometry['polygon'])
        if lb<=20 and anchor not in t.clear_fail_points: return ('anchor',c,anchor)
        pair=paired_fan(anchor,t.last_signal_bearing,max(lb,20.))
        return ('fan',c,min(pair,key=lambda p:math.dist(self.port.position,p)))

    def scan_certificate(self,site):
        p=self.mesh['vertices'][site]
        unknown=[c for c in range(1,21) if c not in self.tracks and c not in self.absent]
        unknown.sort(key=lambda c:(c!=self.port.channel,c))
        self.stats['certificate_sites_visited']+=1
        for c in unknown:
            if len(self.tracks)==16: break
            self.observe(p,c,'B4_certificate',site=site)

    def execute_source(self,task):
        kind,c,p=task; t=self.tracks[c]
        if kind in ('near','guaranteed','anchor'):
            reason={'near':'B4_near_clear','guaranteed':'B4_guaranteed_clear','anchor':'B4_anchor_clear'}[kind]
            self.clear(p,c,kind!='anchor',reason); return
        if kind=='fallback': self.fallback(c); return
        if kind!='fan': raise InvariantFailure('unknown source task')
        anchor=t.last_signal_point
        lb=max(20.,lower_distance(anchor,t.geometry['polygon']))
        pair=sorted(paired_fan(anchor,t.last_signal_bearing,lb),key=lambda q:math.dist(self.port.position,q))
        if tuple(p)!=tuple(pair[0]): raise InvariantFailure('stale fan task')
        t.fan_steps+=1; self.stats['fan_steps']+=1
        self.port.record({'kind':'B4_fan','channel':c,'anchor':anchor,'lower_bound_m':lb,'points':pair})
        first=self.observe(pair[0],c,'B4_fan_first',backside=True)
        if first['measure_result']=='no_signal':
            self.stats['fan_second_required']+=1
            second=self.observe(pair[1],c,'B4_fan_second',backside=True)
            if second['measure_result']=='no_signal':
                self.stats['fan_invariant_failures']+=1
                raise InvariantFailure('both SPF45 branches returned no_signal')
        else: self.stats['fan_first_success']+=1

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        # At most M scans + 16*(8 fans + 9 anchor clears + terminal clear).
        # A fallback is internally finite. This guard is deliberately looser.
        for _ in range(1000):
            nodes=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            nodes += [self.source_task(c) for c in self.tracks if c not in self.cleared]
            if not nodes: break
            task=self.select_task(nodes)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes)})
            if task[0]=='cover':
                self.scan_certificate(task[1]); remaining.remove(task[1])
            else: self.execute_source(task)
        else: raise InvariantFailure('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise InvariantFailure('incomplete discovery or clearance')

    def select_task(self,nodes):
        return E11.e5.fast_open_route(self.port.position,nodes)[0]


class G1DeferredSolver(G1Solver):
    """Fixed certificate sweep, source tasks inserted with <=200m detour."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        route=E11.e5.fast_open_route(self.port.position,[('cover',i,p) for i,p in enumerate(self.mesh['vertices'])])
        self.cover_rank={t[1]:i for i,t in enumerate(route)}

    def select_task(self,nodes):
        covers=[t for t in nodes if t[0]=='cover']
        if not covers: return super().select_task(nodes)
        cover=min(covers,key=lambda t:self.cover_rank[t[1]])
        start=self.port.position; destination=cover[2]
        eligible=[t for t in nodes if t[0]!='cover' and
                  math.dist(start,t[2])+math.dist(t[2],destination)-math.dist(start,destination)<=200.]
        if eligible: return min(eligible,key=lambda t:math.dist(start,t[2]))
        return cover
