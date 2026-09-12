"""G3 sandbox: stable certificate-route spine with source-task insertion.

Inspired by informative/orienteering path planning: treat mandatory discovery sites as
an immutable information spine, and insert localization/clear tasks only when their
incremental travel cost is low on the current spine. G0 fallbacks are unchanged.
"""
import math
from solver_g2 import G2OpportunisticSolver, better_open_route


def _best_insertion(start, route, point):
    """Return (extra_distance, segment_index) for inserting point before route[index].

    segment_index 0 is start->route[0]. For k>0 it is route[k-1]->route[k].
    Appending after the last mandatory cover is intentionally not considered during
    discovery; it can be handled after the certificate is complete.
    """
    if not route:
        return (0.0, 0)
    best = None
    prev = start
    for k, node in enumerate(route):
        nxt = node[2]
        extra = math.dist(prev, point) + math.dist(point, nxt) - math.dist(prev, nxt)
        cand = (extra, k)
        if best is None or cand < best:
            best = cand
        prev = nxt
    return best


class G3SpineSolver(G2OpportunisticSolver):
    # DEV-only knobs.
    clear_detour_limit_m = 650.0
    fan_detour_limit_m = 300.0

    def __init__(self,*args,**kwargs):
        # Carry forward the best G2 DEV settings.
        super().__init__(*args, opportunity_per_site=6, max_proxy_distance=1000., min_line_angle=14., **kwargs)
        self.stats.update({'spine_clears':0,'spine_fans':0,'spine_cover_steps':0,
                           'spine_clear_deferred':0,'spine_fan_deferred':0})

    def _spine_choice(self, remaining):
        covers=[('cover',i,self.mesh['vertices'][i]) for i in remaining]
        route=better_open_route(self.port.position,covers)
        if not route:
            return None
        immediate=[]
        for c in self.tracks:
            if c in self.cleared:
                continue
            task=self.source_task(c)
            kind=task[0]
            if kind=='fallback':
                continue
            if kind=='fan' and self.has_future_opportunity(c,remaining):
                self.stats['fan_deferred_cycles']+=1
                continue
            extra,seg=_best_insertion(self.port.position,route,task[2])
            if seg!=0:
                if kind=='fan': self.stats['spine_fan_deferred']+=1
                else: self.stats['spine_clear_deferred']+=1
                continue
            limit=self.fan_detour_limit_m if kind=='fan' else self.clear_detour_limit_m
            if extra<=limit:
                # Include action overhead only as a tiny tie-break; travel dominates.
                immediate.append((extra, 1 if kind=='fan' else 0, math.dist(self.port.position,task[2]), task))
        if immediate:
            return min(immediate)[-1]
        return route[0]

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1600):
            if not self.discovery_complete() and remaining:
                task=self._spine_choice(remaining)
            else:
                nodes=[self.source_task(c) for c in self.tracks if c not in self.cleared]
                if not nodes: break
                task=self.select_task(nodes)
            if task is None: break
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(remaining),
                              'policy':'G3_spine'})
            if task[0]=='cover':
                self.stats['spine_cover_steps']+=1
                self.scan_certificate(task[1]); remaining.remove(task[1])
            else:
                if task[0]=='fan': self.stats['spine_fans']+=1
                else: self.stats['spine_clears']+=1
                self.execute_source(task)
        else:
            raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')


class G3Spine300(G3SpineSolver): clear_detour_limit_m=300.; fan_detour_limit_m=200.
class G3Spine450(G3SpineSolver): clear_detour_limit_m=450.; fan_detour_limit_m=250.
class G3Spine800(G3SpineSolver): clear_detour_limit_m=800.; fan_detour_limit_m=350.
class G3Spine1200(G3SpineSolver): clear_detour_limit_m=1200.; fan_detour_limit_m=500.
