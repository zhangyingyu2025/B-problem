"""G2 sandbox prototype: exploit mandatory certificate sites as free localization measurements.

Correctness fallback is unchanged: ambiguous no_signal never clips position; if the
mandatory route does not resolve a source, the inherited SPF45/fallback chain runs.
"""
import math
from solver_g1 import G1Solver


def line_angle_deg(a,b):
    d=abs((a-b+180.)%360.-180.)
    return min(d,180.-d)


class G2OpportunisticSolver(G1Solver):
    def __init__(self,*args,opportunity_per_site=4,max_proxy_distance=1250.,min_line_angle=14.,**kwargs):
        super().__init__(*args,**kwargs)
        self.opportunity_per_site=opportunity_per_site
        self.max_proxy_distance=max_proxy_distance
        self.min_line_angle=min_line_angle
        self.stats.update({'opportunity_measurements':0,'opportunity_directions':0,
                           'opportunity_no_signal':0,'opportunity_near':0,
                           'fan_deferred_cycles':0})

    def _proxy(self,t):
        if t.mec: return t.mec[0]
        if t.geometry and t.geometry.get('centers'): return t.geometry['centers'][0]
        return None

    def _already_sampled(self,t,p):
        # A repeated location has fixed error in the simulator, so remeasurement adds no information.
        for q in t.signals+t.ambiguous_no_signal:
            if math.dist(q,p)<1e-7:return True
        return False

    def opportunity_score(self,c,p):
        t=self.tracks[c]
        if c in self.cleared or t.near is not None or not t.dirs or self._already_sampled(t,p): return None
        proxy=self._proxy(t)
        if proxy is None:return None
        dist=math.dist(p,proxy)
        if dist>self.max_proxy_distance:return None
        cand=math.degrees(math.atan2(proxy[1]-p[1],proxy[0]-p[0]))
        separations=[]
        for o in t.dirs:
            old=math.degrees(math.atan2(proxy[1]-o['y'],proxy[0]-o['x']))
            separations.append(line_angle_deg(cand,old))
        sep=max(separations) if separations else 0.
        if sep<self.min_line_angle:return None
        # Larger crossing angle and sources still requiring large MEC get priority.
        radius=t.mec[1] if t.mec else 1000.
        # Favor nearer candidates modestly because reception is more likely with R>=1000.
        range_bonus=max(0.,(self.max_proxy_distance-dist)/self.max_proxy_distance)
        return sep + 0.018*min(radius,1000.) + 8.*range_bonus

    def has_future_opportunity(self,c,remaining):
        # Deferring a fan costs nothing in correctness: if all opportunities fail, SPF45 remains.
        for site in remaining:
            if self.opportunity_score(c,self.mesh['vertices'][site]) is not None:
                return True
        return False

    def scan_certificate(self,site):
        # Preserve the exact G1 discovery scan first.
        super().scan_certificate(site)
        p=self.mesh['vertices'][site]
        candidates=[]
        for c,t in self.tracks.items():
            if c in self.cleared:continue
            score=self.opportunity_score(c,p)
            if score is not None:candidates.append((score,c))
        candidates.sort(reverse=True)
        for _,c in candidates[:self.opportunity_per_site]:
            body=self.observe(p,c,'B4_certificate_opportunity')
            self.stats['opportunity_measurements']+=1
            kind=body['measure_result']
            if kind=='direction':self.stats['opportunity_directions']+=1
            elif kind=='near':self.stats['opportunity_near']+=1
            else:self.stats['opportunity_no_signal']+=1

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1400):
            nodes=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:continue
                task=self.source_task(c)
                # Clear/fallback tasks stay schedulable. Only active fan travel can be replaced by a free future sample.
                if task[0]=='fan' and remaining and not self.discovery_complete() and self.has_future_opportunity(c,remaining):
                    self.stats['fan_deferred_cycles']+=1
                    continue
                nodes.append(task)
            if not nodes:break
            task=self.select_task(nodes)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G2_opportunity'})
            if task[0]=='cover':
                self.scan_certificate(task[1]);remaining.remove(task[1])
            else:self.execute_source(task)
        else:raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')

class G2RouteFirstSolver(G2OpportunisticSolver):
    """Finish the mandatory discovery sweep with free co-measurements before active source travel."""
    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        # Phase 1: movement is only between mandatory certificate points.
        for _ in range(40):
            if self.discovery_complete() or not remaining:break
            nodes=[('cover',i,self.mesh['vertices'][i]) for i in remaining]
            task=self.select_task(nodes)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G2_route_first'})
            self.scan_certificate(task[1]); remaining.remove(task[1])
            # near is literally at the current certificate point; clear it with zero added movement.
            for c,t in list(self.tracks.items()):
                if c not in self.cleared and t.near is not None and math.dist(self.port.position,t.near)<1e-7:
                    self.clear(t.near,c,True,'B4_near_clear')
        # Phase 2: all remaining source tasks, using G1 joint route.
        for _ in range(1000):
            nodes=[self.source_task(c) for c in self.tracks if c not in self.cleared]
            if not nodes:break
            task=self.select_task(nodes)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G2_route_first_finish'})
            self.execute_source(task)
        else:raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):raise RuntimeError('incomplete discovery or clearance')

def _path_cost(start,nodes,order):
    if not order:return 0.
    z=math.dist(start,nodes[order[0]][2])
    for a,b in zip(order,order[1:]):z+=math.dist(nodes[a][2],nodes[b][2])
    return z

def better_open_route(start,nodes):
    nodes=list(nodes); n=len(nodes)
    if n<=1:return nodes
    candidates=[]
    # Multi-start nearest neighbor.
    starts=sorted(range(n),key=lambda i:math.dist(start,nodes[i][2]))[:min(10,n)]
    for first in starts:
        rem=set(range(n));rem.remove(first);order=[first];p=nodes[first][2]
        while rem:
            j=min(rem,key=lambda i:math.dist(p,nodes[i][2]));rem.remove(j);order.append(j);p=nodes[j][2]
        candidates.append(order)
    # Cheapest insertion path from the closest point, free end.
    first=min(range(n),key=lambda i:math.dist(start,nodes[i][2])); order=[first]; rem=set(range(n));rem.remove(first)
    while rem:
        best=None
        for j in rem:
            # insert before position k or append at end
            for k in range(len(order)+1):
                prev=start if k==0 else nodes[order[k-1]][2]
                if k==len(order): delta=math.dist(prev,nodes[j][2])
                else:
                    nxt=nodes[order[k]][2]
                    delta=math.dist(prev,nodes[j][2])+math.dist(nodes[j][2],nxt)-math.dist(prev,nxt)
                if best is None or delta<best[0]:best=(delta,j,k)
        _,j,k=best;order.insert(k,j);rem.remove(j)
    candidates.append(order)
    best_order=None;best_cost=None
    for order in candidates:
        # 2-opt + single-node relocation; fixed start, free end.
        for _ in range(30):
            improved=False
            base=_path_cost(start,nodes,order)
            # best 2-opt move
            move=None; mc=base
            for i in range(len(order)-1):
                for j in range(i+1,len(order)):
                    cand=order[:i]+list(reversed(order[i:j+1]))+order[j+1:]
                    c=_path_cost(start,nodes,cand)
                    if c+1e-8<mc:mc=c;move=cand
            if move is not None:order=move;improved=True;base=mc
            # best relocate move
            move=None;mc=base
            for i in range(len(order)):
                x=order[i]; rest=order[:i]+order[i+1:]
                for k in range(len(rest)+1):
                    cand=rest[:k]+[x]+rest[k:]
                    c=_path_cost(start,nodes,cand)
                    if c+1e-8<mc:mc=c;move=cand
            if move is not None:order=move;improved=True
            if not improved:break
        c=_path_cost(start,nodes,order)
        if best_cost is None or c<best_cost:best_cost=c;best_order=order
    return [nodes[i] for i in best_order]

class G2BetterRouteSolver(G2OpportunisticSolver):
    def select_task(self,nodes):
        return better_open_route(self.port.position,nodes)[0]

class _G2DetourBase(G2OpportunisticSolver):
    detour_limit=400.
    def select_task(self,nodes):
        covers=[t for t in nodes if t[0]=='cover']
        if not covers:return super().select_task(nodes)
        # Re-optimize the remaining certificate path instead of using a frozen sweep.
        cover=better_open_route(self.port.position,covers)[0]
        a=self.port.position; b=cover[2]; direct=math.dist(a,b)
        eligible=[]
        for t in nodes:
            if t[0]=='cover':continue
            detour=math.dist(a,t[2])+math.dist(t[2],b)-direct
            if detour<=self.detour_limit:eligible.append((detour,math.dist(a,t[2]),t))
        if eligible:return min(eligible,key=lambda x:(x[0],x[1]))[2]
        return cover
class G2Detour250Solver(_G2DetourBase):detour_limit=250.
class G2Detour500Solver(_G2DetourBase):detour_limit=500.
class G2Detour800Solver(_G2DetourBase):detour_limit=800.

class G2WideSolver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1500.,min_line_angle=10.,**kw)
class G2Range1000Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=4,max_proxy_distance=1000.,min_line_angle=14.,**kw)
class G2Angle20Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=4,max_proxy_distance=1250.,min_line_angle=20.,**kw)
class G2SixSolver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1250.,min_line_angle=14.,**kw)
class G2TwoSolver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=2,max_proxy_distance=1250.,min_line_angle=14.,**kw)

class G2Six1000Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1000.,min_line_angle=14.,**kw)
class G2Six1100Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1100.,min_line_angle=14.,**kw)
class G2Six1400Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1400.,min_line_angle=14.,**kw)
class G2SixAngle10Solver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=6,max_proxy_distance=1250.,min_line_angle=10.,**kw)
class G2EightSolver(G2OpportunisticSolver):
    def __init__(self,*a,**kw):super().__init__(*a,opportunity_per_site=8,max_proxy_distance=1250.,min_line_angle=14.,**kw)
