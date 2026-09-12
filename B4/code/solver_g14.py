"""G14 DEV: clear sources for free when a paid travel segment crosses the exact
vertex-disk guaranteed-clear region from G13.
"""
import math
from solver_g13 import G13FullClearRegion

R_CLEAR=19.99998

def segment_clear_interval(a,b,vertices,R=R_CLEAR):
    dx,dy=b[0]-a[0],b[1]-a[1]; A=dx*dx+dy*dy
    if A<1e-14:
        return (0.,1.) if all(math.dist(a,v)<=R for v in vertices) else None
    lo,hi=0.,1.
    for vx,vy in vertices:
        ox,oy=a[0]-vx,a[1]-vy
        B=2*(ox*dx+oy*dy); C=ox*ox+oy*oy-R*R
        disc=B*B-4*A*C
        if disc<0:
            # Entire line outside if quadratic minimum positive.
            if C>0:return None
            continue
        root=math.sqrt(max(0.,disc)); l=(-B-root)/(2*A); h=(-B+root)/(2*A)
        lo=max(lo,l);hi=min(hi,h)
        if lo>hi+1e-12:return None
    if hi<0 or lo>1:return None
    return max(0.,lo),min(1.,hi)

class G14TransitFullClear(G13FullClearRegion):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.stats.update({'transit_full_region_clears':0})
    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position;length=math.dist(start,destination)
        if length<1e-7:return
        plan=[]
        # Normal G4 opportunities, using G10/G13 scoring that skips already-clearable sources.
        options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c)
            if best is not None:options.append(best)
        for score,u,p,c in sorted(options,reverse=True)[:self.transit_per_leg]:plan.append((u,1,'measure',c,p))
        # Exact safe-region intersection along this already-paid segment.
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or not t.mec or t.mec[1]>20-1e-6 or not t.geometry:continue
            verts=t.geometry.get('polygon',[])
            interval=segment_clear_interval(start,destination,verts)
            if interval is None:continue
            u=interval[0];p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
            plan.append((u,0,'clear',c,p))
        plan.sort(key=lambda x:(x[0],x[1]))
        for u,_,kind,c,p in plan:
            if c in self.cleared:continue
            if kind=='clear':
                t=self.tracks[c]
                if not t.geometry or not all(math.dist(p,v)<=R_CLEAR+1e-7 for v in t.geometry.get('polygon',[])):continue
                self.clear(p,c,True,'B4_transit_full_region_clear');self.stats['transit_full_region_clears']+=1
            else:
                t=self.tracks[c]
                if t.near is not None or self._already_sampled(t,p):continue
                body=self.observe(p,c,'B4_transit_opportunity');self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result']
                if k=='direction':self.stats['transit_opportunity_directions']+=1
                elif k=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
