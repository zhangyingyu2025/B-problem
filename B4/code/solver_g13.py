"""G13 DEV: nearest point in the full guaranteed-clear region.

For convex source polygon P with vertices v_i, a robot point q guarantees clear iff
||q-v_i|| <= 20 for every vertex. This is an intersection of equal disks. The nearest
point to the robot is either already feasible, on one active disk along the robot ray,
or at an intersection of two active disk boundaries. Enumerate these candidates with a
small inward radius margin; fall back to G11's MEC-inscribed safe disk if needed.
"""
import math
from solver_g11 import G11SafeClear


def _circle_intersections(a,b,R):
    dx,dy=b[0]-a[0],b[1]-a[1]; d=math.hypot(dx,dy)
    if d<1e-12 or d>2*R:return []
    x=d/2.; h2=R*R-x*x
    if h2<-1e-10:return []
    h=math.sqrt(max(0.,h2)); ux,uy=dx/d,dy/d; mx,my=a[0]+x*ux,a[1]+x*uy
    px,py=-uy,ux
    if h<1e-12:return [(mx,my)]
    return [(mx+h*px,my+h*py),(mx-h*px,my-h*py)]


def nearest_clear_point(here, vertices, R=19.99998):
    if not vertices:return None
    def feasible(q):return all(math.dist(q,v)<=R+1e-8 for v in vertices)
    if feasible(here):return here
    candidates=[]
    for v in vertices:
        d=math.dist(v,here)
        if d>1e-12:
            q=(v[0]+R*(here[0]-v[0])/d,v[1]+R*(here[1]-v[1])/d)
            if feasible(q):candidates.append(q)
    # Only vertices separated by <=2R can define an active intersection.
    for i,a in enumerate(vertices):
        for b in vertices[i+1:]:
            if math.dist(a,b)<=2*R+1e-8:
                for q in _circle_intersections(a,b,R):
                    if feasible(q):candidates.append(q)
    return min(candidates,key=lambda q:math.dist(here,q)) if candidates else None

class G13FullClearRegion(G11SafeClear):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.stats.update({'full_clear_region_used':0,'full_clear_region_gain_m':0.})
    def source_task(self,c):
        # Get G10/G11 classification, but recompute full-region point for guaranteed task.
        task=super().source_task(c)
        if task[0]!='guaranteed':return task
        t=self.tracks[c]; vertices=t.geometry.get('polygon',[]) if t.geometry else []
        q=nearest_clear_point(self.port.position,vertices)
        if q is None:return task
        old=math.dist(self.port.position,task[2]);new=math.dist(self.port.position,q)
        self.stats['full_clear_region_used']+=1;self.stats['full_clear_region_gain_m']+=max(0.,old-new)
        return ('guaranteed',c,q)
