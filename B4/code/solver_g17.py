"""G17 DEV: local TSP-with-neighborhoods optimization for guaranteed clear tasks.

G13 established that a source with outer polygon vertices v_i can be cleared from
any q in Q=intersection_i B(v_i,20).  G15 still presents Q to the route planner as
one point chosen nearest to the *current* robot.  That is myopic when the clear lies
between two later tasks.

Here an initial open route is built as before.  For every interior guaranteed-clear
node, the clear point is moved to the certified point in Q nearest the segment joining
its predecessor and successor; terminal nodes use the point nearest their predecessor.
The discrete route is then rebuilt and the process repeated a few times.  Every
returned clear point is rechecked against all outer-set vertices, so correctness is
identical to G13/G15; only the representative of the clear neighborhood changes.
"""
from __future__ import annotations
import math

from solver_g15 import G15TightMesh
from solver_g13 import nearest_clear_point
R_CLEAR=19.99998
from solver_g1 import E11


def _project_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    d2=dx*dx+dy*dy
    if d2<1e-14:return a
    t=((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d2
    t=max(0.,min(1.,t))
    return (a[0]+t*dx,a[1]+t*dy)


class G17TSPN(G15TightMesh):
    tspn_rounds=2
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.stats.update({'tspn_representative_updates':0,'tspn_estimated_route_gain_m':0.0})

    def _optimize_route(self,nodes):
        work=list(nodes)
        if len(work)<=1:return work
        before=None
        for _ in range(self.tspn_rounds):
            route=E11.e5.fast_open_route(self.port.position,work)
            old=E11.e5.route_cost(self.port.position,route)
            changed=[]
            for i,t in enumerate(route):
                if t[0]!='guaranteed':
                    changed.append(t);continue
                c=t[1];track=self.tracks.get(c);verts=track.geometry.get('polygon',[]) if track and track.geometry else []
                if not verts:
                    changed.append(t);continue
                prev=self.port.position if i==0 else route[i-1][2]
                if i+1<len(route):
                    nxt=route[i+1][2]
                    # Use the MEC/old representative only to choose a projection
                    # parameter; the final q is recomputed inside the exact full
                    # guaranteed-clear region.
                    seed=_project_segment(t[2],prev,nxt)
                    q=nearest_clear_point(seed,verts,R=R_CLEAR)
                else:
                    q=nearest_clear_point(prev,verts,R=R_CLEAR)
                if q is None or not all(math.dist(q,v)<=R_CLEAR+1e-7 for v in verts):
                    changed.append(t);continue
                changed.append((t[0],t[1],q))
                if math.dist(q,t[2])>1e-8:self.stats['tspn_representative_updates']+=1
            work=changed
            newroute=E11.e5.fast_open_route(self.port.position,work)
            new=E11.e5.route_cost(self.port.position,newroute)
            if before is None:before=old
            if new>=old-1e-7:
                # Coordinates can still help a later discrete reorder, but if this
                # round made no progress there is no reason to iterate further.
                work=changed;break
        final=E11.e5.fast_open_route(self.port.position,work)
        if before is not None:
            self.stats['tspn_estimated_route_gain_m']+=max(0.,before-E11.e5.route_cost(self.port.position,final))
        return final

    def select_task(self,nodes):
        return self._optimize_route(nodes)[0]

class G17OneRound(G17TSPN):tspn_rounds=1
class G17ThreeRounds(G17TSPN):tspn_rounds=3
