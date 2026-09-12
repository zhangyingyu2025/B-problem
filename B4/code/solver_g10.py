"""G10 DEV: defer guaranteed clears until the mandatory certificate route gets closer.

A clear whose MEC radius <=20 m stays mathematically guaranteed forever (additional
measurements can only shrink the source set). During discovery we can therefore wait
for a future mandatory certificate site closer to its center, avoiding an early detour.
Correctness fallback is unchanged.
"""
import math
from solver_g4 import G4TransitSolver

class _G10ClearDefer(G4TransitSolver):
    min_future_saving_m=200.0
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.stats.update({'guaranteed_clear_deferred_cycles':0})
    def opportunity_score(self,c,p):
        t=self.tracks[c]
        # Once clear is guaranteed, more bearing samples have zero mission value.
        if t.mec and t.mec[1] <= 20-1e-6:
            return None
        return super().opportunity_score(c,p)
    def _defer_clear(self,task,remaining):
        if task[0] != 'guaranteed' or not remaining or self.discovery_complete():
            return False
        q=task[2]; now=math.dist(self.port.position,q)
        future=min(math.dist(self.mesh['vertices'][i],q) for i in remaining)
        return future + self.min_future_saving_m < now
    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1700):
            nodes=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:continue
                task=self.source_task(c)
                if task[0]=='fan' and remaining and not self.discovery_complete() and self.has_future_opportunity(c,remaining):
                    self.stats['fan_deferred_cycles']+=1;continue
                if self._defer_clear(task,remaining):
                    self.stats['guaranteed_clear_deferred_cycles']+=1;continue
                nodes.append(task)
            if not nodes:break
            task=self.select_task(nodes);dest=task[2];exclude=None if task[0]=='cover' else task[1]
            self.transit_opportunities(dest,exclude_channel=exclude)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G10_clear_defer'})
            if task[0]=='cover':self.scan_certificate(task[1]);remaining.remove(task[1])
            else:self.execute_source(task)
        else:raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):raise RuntimeError('incomplete discovery or clearance')
class G10D0(_G10ClearDefer):min_future_saving_m=0.
class G10D100(_G10ClearDefer):min_future_saving_m=100.
class G10D200(_G10ClearDefer):min_future_saving_m=200.
class G10D300(_G10ClearDefer):min_future_saving_m=300.
class G10D500(_G10ClearDefer):min_future_saving_m=500.
class G10NoDefer(_G10ClearDefer):
    def _defer_clear(self,task,remaining): return False
