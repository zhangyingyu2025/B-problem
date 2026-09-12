"""G4 sandbox: take informative bearing measurements *on already-paid travel segments*.

All transit samples lie on the straight segment to the already chosen task, in monotone
order, so they add zero movement distance. They only add measure/switch time. Ambiguous
no_signal remains non-clipping and G0 SPF45/fallback correctness is unchanged.
"""
import math
from solver_g2 import G2OpportunisticSolver


class G4TransitSolver(G2OpportunisticSolver):
    transit_per_leg = 4
    transit_fracs = (0.20,0.40,0.60,0.80)

    def __init__(self,*args,**kwargs):
        super().__init__(*args, opportunity_per_site=6, max_proxy_distance=1000., min_line_angle=14., **kwargs)
        self.stats.update({'transit_opportunity_measurements':0,'transit_opportunity_directions':0,
                           'transit_opportunity_no_signal':0,'transit_opportunity_near':0})

    def transit_opportunities(self, destination, exclude_channel=None):
        start=self.port.position
        length=math.dist(start,destination)
        if length<1e-7 or self.transit_per_leg<=0:
            return
        options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:
                continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):
                    best=(score,u,p,c)
            if best is not None:
                options.append(best)
        # one sample per source per leg; strongest expected geometry first, then execute monotonically.
        chosen=sorted(options,reverse=True)[:self.transit_per_leg]
        chosen.sort(key=lambda x:x[1])
        for _,u,p,c in chosen:
            # A prior sample on this same leg can have made this source clearable; avoid redundant measure.
            if c in self.cleared or self.tracks[c].near is not None or self._already_sampled(self.tracks[c],p):
                continue
            body=self.observe(p,c,'B4_transit_opportunity')
            self.stats['transit_opportunity_measurements']+=1
            kind=body['measure_result']
            if kind=='direction':self.stats['transit_opportunity_directions']+=1
            elif kind=='near':self.stats['transit_opportunity_near']+=1
            else:self.stats['transit_opportunity_no_signal']+=1

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1600):
            nodes=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared: continue
                task=self.source_task(c)
                if task[0]=='fan' and remaining and not self.discovery_complete() and self.has_future_opportunity(c,remaining):
                    self.stats['fan_deferred_cycles']+=1
                    continue
                nodes.append(task)
            if not nodes: break
            task=self.select_task(nodes)
            dest=task[2]
            exclude=None if task[0]=='cover' else task[1]
            self.transit_opportunities(dest,exclude_channel=exclude)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G4_transit'})
            if task[0]=='cover':
                self.scan_certificate(task[1]);remaining.remove(task[1])
            else:
                # Other-source transit samples cannot stale this task's own geometry.
                self.execute_source(task)
        else: raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')

class G4Transit2(G4TransitSolver): transit_per_leg=2
class G4Transit6(G4TransitSolver): transit_per_leg=6
class G4Transit8(G4TransitSolver): transit_per_leg=8
