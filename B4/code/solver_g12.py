"""G12 DEV: zero-detour guaranteed clears on already-paid travel segments.

For a source outer set contained in B(c,r), r<20, B(c,20-r-eps) is a certified
robot-position disk from which clear must succeed. If an already-selected travel
segment intersects that disk, execute clear at the first intersection without adding
travel distance. Transit bearing samples and clears are jointly ordered along the segment.
"""
import math
from solver_g11 import G11SafeClear


def segment_disk_entry(a,b,c,r):
    dx,dy=b[0]-a[0],b[1]-a[1]; L2=dx*dx+dy*dy
    if L2<=1e-14:
        return 0.0 if math.dist(a,c)<=r else None
    # projection parameter and squared perpendicular distance
    t0=((c[0]-a[0])*dx+(c[1]-a[1])*dy)/L2
    tc=max(0.,min(1.,t0)); q=(a[0]+tc*dx,a[1]+tc*dy)
    if math.dist(q,c)>r+1e-9:return None
    L=math.sqrt(L2)
    # line entry; clamp for segment endpoints
    perp2=max(0.,(a[0]+t0*dx-c[0])**2+(a[1]+t0*dy-c[1])**2)
    dt=math.sqrt(max(0.,r*r-perp2))/L
    return max(0.,min(1.,t0-dt))

class G12TransitClear(G11SafeClear):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.stats.update({'transit_guaranteed_clears':0})

    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position; length=math.dist(start,destination)
        if length<1e-7:return
        plan=[]
        # Existing G4 known-source opportunities.
        options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c)
            if best is not None:options.append(best)
        for score,u,p,c in sorted(options,reverse=True)[:self.transit_per_leg]:
            plan.append((u,1,'measure',c,p))
        # Certified clear disks intersected by this paid segment.
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or not t.mec:continue
            center,r=t.mec
            if r>20-1e-6:continue
            safe=max(0.,20.-r-self.clear_margin)
            if safe<=0:continue
            u=segment_disk_entry(start,destination,center,safe)
            if u is None:continue
            p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
            plan.append((u,0,'clear',c,p))
        plan.sort(key=lambda x:(x[0],x[1]))
        for u,_,kind,c,p in plan:
            if c in self.cleared:continue
            if kind=='clear':
                # Recheck current certificate in case prior actions changed state only favorably.
                t=self.tracks[c]
                if not t.mec or t.mec[1]>20-1e-6:continue
                center,r=t.mec; safe=max(0.,20.-r-self.clear_margin)
                if math.dist(p,center)>safe+1e-7:continue
                self.clear(p,c,True,'B4_transit_guaranteed_clear')
                self.stats['transit_guaranteed_clears']+=1
            else:
                t=self.tracks[c]
                if t.near is not None or self._already_sampled(t,p):continue
                body=self.observe(p,c,'B4_transit_opportunity')
                self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result']
                if k=='direction':self.stats['transit_opportunity_directions']+=1
                elif k=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
