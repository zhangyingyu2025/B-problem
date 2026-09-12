"""G35 DEV: zero-detour guaranteed-in-range probes on the 21-site backbone.

Mathematical invariant after the first direction at O with measured center theta:
K0 is contained in the radius-1500, +/-1 degree sector.  Let A+/- be the two
far endpoints.  A probe q satisfying
    rho(q)=max(|q-O|, |q-A+|, |q-A-|) <= 1000
is within the guaranteed minimum reception radius of every feasible source.
Therefore no_signal at q is a proven directional backside event, never a range
ambiguity.

Two such backside probes on opposite sides of the whole first-bearing wedge give
an exact radial upper bound: their chord intersects every feasible bearing ray;
a source beyond that intersection cannot see O while rejecting both probes.
The resulting radius bound is applied conservatively as an additional outer-disk
constraint.  All G34 adaptive fan / center-probe / finite fallback logic remains
available when free backbone probes do not resolve the source.
"""
from __future__ import annotations

import math

from solver_g34 import G34R80
from solver_g1 import G1Solver
from solver_g14 import segment_clear_interval, R_CLEAR

ALPHA_DEG = 1.0
ALPHA = math.radians(ALPHA_DEG)
SAFE_R = 1000.0
MAX_SOURCE_RANGE = 1500.0
EPS = 2e-7


def _unit(deg):
    a=math.radians(deg)
    return math.cos(a),math.sin(a)


def _cross(a,b):
    return a[0]*b[1]-a[1]*b[0]


def _local(obs,p):
    """Coordinates relative to first measured bearing centerline."""
    u=_unit(obs['svd_deg']); n=(-u[1],u[0])
    dx,dy=p[0]-obs['x'],p[1]-obs['y']
    return dx*u[0]+dy*u[1], dx*n[0]+dy*n[1]


def sector_extremes(obs):
    o=(obs['x'],obs['y'])
    pts=[]
    for s in (-1.,1.):
        u=_unit(obs['svd_deg']+s*ALPHA_DEG)
        pts.append((o[0]+MAX_SOURCE_RANGE*u[0],o[1]+MAX_SOURCE_RANGE*u[1]))
    return o,pts[0],pts[1]


def safe_rho(obs,p):
    return max(math.dist(p,c) for c in sector_extremes(obs))


def _disk_interval(a,b,c,r=SAFE_R):
    """t interval in [0,1] for a+t(b-a) inside a closed disk."""
    dx,dy=b[0]-a[0],b[1]-a[1]
    ox,oy=a[0]-c[0],a[1]-c[1]
    A=dx*dx+dy*dy
    if A<1e-18:
        return (0.,1.) if ox*ox+oy*oy <= r*r+1e-9 else None
    B=2*(ox*dx+oy*dy); C=ox*ox+oy*oy-r*r
    disc=B*B-4*A*C
    if disc < -1e-8:return None
    root=math.sqrt(max(0.,disc))
    lo=(-B-root)/(2*A); hi=(-B+root)/(2*A)
    lo=max(0.,lo);hi=min(1.,hi)
    return (lo,hi) if lo<=hi+1e-12 else None


def _linear_interval(a,b,fa,fb,positive=True):
    """t interval where a linear signed quantity f(t) is >=0 (or <=0)."""
    if not positive: fa,fb=-fa,-fb
    if fa>=-1e-12 and fb>=-1e-12:return (0.,1.)
    if fa<-1e-12 and fb<-1e-12:return None
    den=fb-fa
    if abs(den)<1e-18:return None
    t=-fa/den
    if fa>=-1e-12:return (0.,min(1.,max(0.,t)))
    return (max(0.,min(1.,t)),1.)


def safe_segment_interval(obs,a,b,side=0):
    """Exact line-segment intersection with the three-disk safe region.

    side=+1 additionally requires the point to lie above the +alpha boundary ray;
    side=-1 below the -alpha ray.  Those side constraints guarantee that one upper
    and one lower point bracket every feasible first-bearing ray.
    """
    lo,hi=0.,1.
    for c in sector_extremes(obs):
        z=_disk_interval(a,b,c)
        if z is None:return None
        lo=max(lo,z[0]);hi=min(hi,z[1])
        if lo>hi+1e-12:return None
    if side:
        # In first-bearing local coordinates, upper: y-x*tan(alpha)>=0;
        # lower: y+x*tan(alpha)<=0, written as a signed linear condition >=0.
        xa,ya=_local(obs,a);xb,yb=_local(obs,b)
        if side>0:
            fa=ya-xa*math.tan(ALPHA);fb=yb-xb*math.tan(ALPHA)
        else:
            fa=-(ya+xa*math.tan(ALPHA));fb=-(yb+xb*math.tan(ALPHA))
        z=_linear_interval(a,b,fa,fb,True)
        if z is None:return None
        lo=max(lo,z[0]);hi=min(hi,z[1])
        if lo>hi+1e-12:return None
    return max(0.,lo),min(1.,hi)


def safe_point_on_segment(obs,a,b,side=0):
    """Pick a strong zero-detour probe from the exact feasible interval.

    Endpoints and midpoint suffice for policy scoring; safety itself is exact because
    every candidate lies inside the certified interval.  Prefer smaller rho, then a
    larger angular displacement from the first bearing centerline.
    """
    iv=safe_segment_interval(obs,a,b,side)
    if iv is None:return None
    lo,hi=iv
    ts={lo,hi,(lo+hi)/2}
    cand=[]
    for t in ts:
        p=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
        rho=safe_rho(obs,p)
        if rho>SAFE_R+2e-6:continue
        x,y=_local(obs,p)
        angle=abs(math.degrees(math.atan2(y,x)))
        cand.append((rho,-angle,t,p))
    return min(cand)[-1] if cand else None


def _bracket_side(obs,p):
    x,y=_local(obs,p);ta=math.tan(ALPHA)
    up=y-x*ta; low=-(y+x*ta)
    if up>1e-8:return 1
    if low>1e-8:return -1
    return 0


def chord_radial_upper(obs,p_upper,p_lower):
    """Worst intersection radius of chord p_upper--p_lower over +/-alpha rays.

    Requires the two probes to bracket the full first-bearing wedge. Returns None if
    the geometric preconditions are not numerically satisfied.
    """
    if _bracket_side(obs,p_upper)!=1 or _bracket_side(obs,p_lower)!=-1:return None
    qu=_local(obs,p_upper); ql=_local(obs,p_lower)
    d=(ql[0]-qu[0],ql[1]-qu[1]); num=_cross(qu,ql)
    # r(e)=cross(qu,ql)/cross(u(e),d).  Check endpoints and all stationary
    # points of the denominator that can fall in the tiny [-alpha,+alpha] interval.
    A=d[1];B=-d[0]  # denominator=A*cos(e)+B*sin(e)
    base=math.atan2(B,A)
    es=[-ALPHA,ALPHA]
    for k in range(-2,3):
        e=base+k*math.pi
        if -ALPHA-1e-15<=e<=ALPHA+1e-15:es.append(e)
    vals=[]
    for e in es:
        den=A*math.cos(e)+B*math.sin(e)
        if abs(den)<1e-12:continue
        r=num/den
        if r>0 and math.isfinite(r):vals.append(r)
    if not vals:return None
    R=max(vals)
    return R if R<=MAX_SOURCE_RANGE+1e-5 else R


def _clip_halfplane(poly,a,b,c):
    if not poly:return []
    out=[];prev=poly[-1];old=a*prev[0]+b*prev[1]-c
    for p in poly:
        val=a*p[0]+b*p[1]-c
        pin=val<=1e-10;oin=old<=1e-10
        if pin!=oin:
            den=old-val
            if abs(den)>1e-18:
                t=old/den;out.append((prev[0]+t*(p[0]-prev[0]),prev[1]+t*(p[1]-prev[1])))
        if pin:out.append(tuple(p))
        prev,old=p,val
    return out


def _circle2(a,b):
    c=((a[0]+b[0])/2,(a[1]+b[1])/2);return c,math.dist(a,b)/2


def _circle3(a,b,c):
    ax,ay=a;bx,by=b;cx,cy=c
    d=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(d)<1e-12:return None
    aa=ax*ax+ay*ay;bb=bx*bx+by*by;cc=cx*cx+cy*cy
    ux=(aa*(by-cy)+bb*(cy-ay)+cc*(ay-by))/d
    uy=(aa*(cx-bx)+bb*(ax-cx)+cc*(bx-ax))/d
    q=(ux,uy);return q,math.dist(q,a)


def _min_circle(points):
    pts=list(points)
    if not pts:return None
    c=pts[0];r=0.
    for i,p in enumerate(pts):
        if math.dist(c,p)>r+1e-8:
            c=p;r=0.
            for j,q in enumerate(pts[:i]):
                if math.dist(c,q)>r+1e-8:
                    c,r=_circle2(p,q)
                    for z in pts[:j]:
                        if math.dist(c,z)>r+1e-8:
                            w=_circle3(p,q,z)
                            if w is not None:c,r=w
    r=max(math.dist(c,p) for p in pts)+2e-7
    return c,r


def clip_radial_outer(geometry,center,radius,count=96):
    """Intersect an existing outer polygon with a circumscribed radius bound."""
    if not geometry or geometry.get('status')!='bounded' or not geometry.get('polygon'):
        return geometry
    poly=[tuple(p) for p in geometry['polygon']]
    # Tangent halfplanes n.(x-center)<=radius define a circumscribed regular polygon,
    # hence never remove a point from the exact closed disk.
    rr=radius+2e-6
    for k in range(count):
        a=math.cos(2*math.pi*k/count);b=math.sin(2*math.pi*k/count)
        poly=_clip_halfplane(poly,a,b,a*center[0]+b*center[1]+rr)
        if not poly:return geometry  # numerical anomaly: retain the previous safe outer set
    mec=_min_circle(poly)
    if mec is None:return geometry
    g=dict(geometry);g['polygon']=poly;g['mec']=mec;g['centers']=[mec[0]]
    return g


class G35BackboneResolver(G34R80):
    """Conservative first implementation of the analytical backbone resolver.

    It accepts only zero-detour probes on already-paid certificate/source travel legs.
    Safe no_signal observations are paired when possible to add a certified radial
    upper bound.  No active movement is introduced by the new mechanism; G34 remains
    the correctness fallback.
    """
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.safe_negative={}
        self.radial_upper={}
        self.stats.update({
            'safe_probe_measurements':0,'safe_probe_directions':0,'safe_probe_no_signal':0,
            'safe_probe_near':0,'safe_pairs':0,'safe_pair_radial_updates':0,
            'safe_pair_best_radius_m':None,'safe_site_probes':0,'safe_transit_probes':0,
            'fan_deferred_by_safe':0,
        })

    def observe(self,p,c,reason,site=None,backside=False):
        body=super().observe(p,c,reason,site=site,backside=backside)
        # A fresh direction recomputes base geometry; reapply any certified chord radius.
        if c in self.radial_upper and c in self.tracks and self.tracks[c].geometry:
            center,r=self.radial_upper[c]
            g=clip_radial_outer(self.tracks[c].geometry,center,r)
            self.tracks[c].geometry=g;self.tracks[c].mec=g['mec']
        return body

    def _first_obs(self,c):
        t=self.tracks.get(c)
        return t.dirs[0] if t and t.dirs else None

    def _safe_site(self,c,p,side=0):
        obs=self._first_obs(c)
        if obs is None or self._already_sampled(self.tracks[c],p):return False
        if safe_rho(obs,p)>SAFE_R+1e-6:return False
        return side==0 or _bracket_side(obs,p)==side

    def _pair_update(self,c,p):
        obs=self._first_obs(c)
        if obs is None:return
        side=_bracket_side(obs,p)
        if side==0:return
        rec=self.safe_negative.setdefault(c,[])
        if all(math.dist(p,q)>1e-7 for _,q in rec):rec.append((side,tuple(p)))
        upp=[q for s,q in rec if s==1];low=[q for s,q in rec if s==-1]
        best=None
        for u in upp:
            for l in low:
                R=chord_radial_upper(obs,u,l)
                if R is not None and (best is None or R<best[0]):best=(R,u,l)
        if best is None:return
        self.stats['safe_pairs']+=1
        R,u,l=best
        if R>=MAX_SOURCE_RANGE-1e-6:return
        old=self.radial_upper.get(c)
        if old is not None and R>=old[1]-1e-6:return
        center=(obs['x'],obs['y']);self.radial_upper[c]=(center,R)
        self.stats['safe_pair_radial_updates']+=1
        b=self.stats['safe_pair_best_radius_m']
        self.stats['safe_pair_best_radius_m']=R if b is None else min(b,R)
        t=self.tracks[c]
        if t.geometry:
            g=clip_radial_outer(t.geometry,center,R)
            t.geometry=g;t.mec=g['mec']
        self.port.record({'kind':'B4_safe_pair_radius','channel':c,'upper_m':R,'upper_probe':u,'lower_probe':l})

    def _record_safe_result(self,c,p,body,where):
        self.stats['safe_probe_measurements']+=1
        if where=='site':self.stats['safe_site_probes']+=1
        else:self.stats['safe_transit_probes']+=1
        k=body['measure_result']
        if k=='direction':self.stats['safe_probe_directions']+=1
        elif k=='near':self.stats['safe_probe_near']+=1
        else:
            self.stats['safe_probe_no_signal']+=1
            self._pair_update(c,p)

    def _safe_priority(self,c,p):
        """Higher is better; all returned points are already safe."""
        obs=self._first_obs(c);side=_bracket_side(obs,p);x,y=_local(obs,p)
        angle=abs(math.degrees(math.atan2(y,x)));rho=safe_rho(obs,p)
        rec=self.safe_negative.get(c,[])
        opposite=any(s==-side for s,_ in rec) if side else False
        # Completing a backside pair is most valuable; then larger angle and tighter rho.
        return (10000. if opposite else 5000.) + 10.*angle + (SAFE_R-rho)/5.

    def has_future_opportunity(self,c,remaining):
        # Preserve ordinary G34/G2 opportunities, but additionally defer active fan travel
        # whenever a mandatory future certificate site itself is a guaranteed safe probe.
        if super().has_future_opportunity(c,remaining):return True
        for i in remaining:
            p=self.mesh['vertices'][i]
            if self._safe_site(c,p):
                self.stats['fan_deferred_by_safe']+=1
                return True
        return False

    def scan_certificate(self,site):
        # Run only the mandatory unknown-channel scan from G1, then perform one known-source
        # opportunity pass so safe points can be labelled proven backside correctly.
        G1Solver.scan_certificate(self,site)
        p=self.mesh['vertices'][site];candidates=[]
        for c,t in self.tracks.items():
            if c in self.cleared or t.near is not None or self._already_sampled(t,p):continue
            if self._safe_site(c,p):
                candidates.append((self._safe_priority(c,p),1,c))
            else:
                score=self.opportunity_score(c,p)
                if score is not None:candidates.append((score,0,c))
        candidates.sort(reverse=True)
        for _,safe,c in candidates[:self.opportunity_per_site]:
            if c in self.cleared or self._already_sampled(self.tracks[c],p):continue
            body=self.observe(p,c,'B4_certificate_safe_opportunity' if safe else 'B4_certificate_opportunity',backside=bool(safe))
            self.stats['opportunity_measurements']+=1
            k=body['measure_result']
            if k=='direction':self.stats['opportunity_directions']+=1
            elif k=='near':self.stats['opportunity_near']+=1
            else:self.stats['opportunity_no_signal']+=1
            if safe:self._record_safe_result(c,p,body,'site')

    def _best_safe_on_leg(self,c,start,destination):
        t=self.tracks[c];obs=self._first_obs(c)
        if obs is None:return None
        # If one safe backside side is already known, first try to complete the opposite side.
        sides={s for s,_ in self.safe_negative.get(c,[])}
        wanted=[]
        if 1 in sides and -1 not in sides:wanted=[-1,0]
        elif -1 in sides and 1 not in sides:wanted=[1,0]
        else:wanted=[1,-1,0]
        best=None
        for side in wanted:
            p=safe_point_on_segment(obs,start,destination,side)
            if p is None or self._already_sampled(t,p):continue
            length=math.dist(start,destination)
            u=0. if length<1e-12 else math.dist(start,p)/length
            # Collinearity plus interval membership makes u monotone; protect rounding by projection.
            dx,dy=destination[0]-start[0],destination[1]-start[1];L2=dx*dx+dy*dy
            if L2>0:u=max(0.,min(1.,((p[0]-start[0])*dx+(p[1]-start[1])*dy)/L2))
            score=self._safe_priority(c,p)
            item=(score,u,p,c)
            if best is None or item[0]>best[0]:best=item
            if side and best is not None:break
        return best

    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position;length=math.dist(start,destination)
        if length<1e-7:return
        plan=[];options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            safe=self._best_safe_on_leg(c,start,destination)
            if safe is not None:
                score,u,p,c=safe;options.append((score+20000.,u,p,c,True));continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c,False)
            if best is not None:options.append(best)
        # one sample per source per leg, strongest first, then execute monotonically
        for score,u,p,c,safe in sorted(options,reverse=True)[:self.transit_per_leg]:
            plan.append((u,1,'measure',c,p,safe))
        # Preserve G14 exact free guaranteed-clear intersections.
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or not t.mec or t.mec[1]>20-1e-6 or not t.geometry:continue
            verts=t.geometry.get('polygon',[]);interval=segment_clear_interval(start,destination,verts)
            if interval is None:continue
            u=interval[0];p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
            plan.append((u,0,'clear',c,p,False))
        plan.sort(key=lambda x:(x[0],x[1]))
        for u,_,kind,c,p,safe in plan:
            if c in self.cleared:continue
            if kind=='clear':
                t=self.tracks[c]
                if not t.geometry or not all(math.dist(p,v)<=R_CLEAR+1e-7 for v in t.geometry.get('polygon',[])):continue
                self.clear(p,c,True,'B4_transit_full_region_clear');self.stats['transit_full_region_clears']+=1
            else:
                t=self.tracks[c]
                if t.near is not None or self._already_sampled(t,p):continue
                body=self.observe(p,c,'B4_transit_safe_probe' if safe else 'B4_transit_opportunity',backside=bool(safe))
                self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result']
                if k=='direction':self.stats['transit_opportunity_directions']+=1
                elif k=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
                if safe:self._record_safe_result(c,p,body,'transit')


class G35SafeBackbone(G35BackboneResolver):
    pass

class G36SafeDistanceDisks(G35BackboneResolver):
    """G35 plus the full distance consequence of every safe probe.

    If q is chosen with rho(q)=max_{g in K0}|q-g|<=1000, then before observing
    anything we already know the true source belongs to B(q,rho(q)).  Intersecting
    this certified disk with the bearing outer set is valid for direction *and*
    no_signal outcomes and is typically much stronger than the generic R<=1500 disk.
    """
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.certified_disks={}
        self.stats.update({'safe_distance_bounds':0,'safe_distance_best_rho_m':None})

    def _apply_certified_disks(self,c):
        if c not in self.tracks:return
        t=self.tracks[c]
        if not t.geometry:return
        g=t.geometry
        for center,r in self.certified_disks.get(c,[]):
            g=clip_radial_outer(g,center,r)
        # G35's chord upper bound is another independently certified disk.
        if c in self.radial_upper:
            center,r=self.radial_upper[c];g=clip_radial_outer(g,center,r)
        t.geometry=g;t.mec=g['mec']

    def observe(self,p,c,reason,site=None,backside=False):
        body=super().observe(p,c,reason,site=site,backside=backside)
        # super may have rebuilt geometry after a fresh direction.
        self._apply_certified_disks(c)
        return body

    def _record_safe_result(self,c,p,body,where):
        obs=self._first_obs(c)
        if obs is not None:
            rho=safe_rho(obs,p)
            if rho<=SAFE_R+3e-6:
                lst=self.certified_disks.setdefault(c,[])
                # Keep distinct disks; a later rebase reapplies every certificate.
                if all(math.dist(p,q)>1e-7 for q,_ in lst):
                    lst.append((tuple(p),rho+3e-6))
                    self.stats['safe_distance_bounds']+=1
                    b=self.stats['safe_distance_best_rho_m']
                    self.stats['safe_distance_best_rho_m']=rho if b is None else min(b,rho)
        super()._record_safe_result(c,p,body,where)
        self._apply_certified_disks(c)

class G36SafeBackboneDisks(G36SafeDistanceDisks):
    pass

STRIP_CLEAR_R=19.9995

def _strip_polygon(obs0,D0,obs1,D1):
    """Conservative parallelogram from two bounded-error bearing strips."""
    u0=_unit(obs0['svd_deg']);u1=_unit(obs1['svd_deg'])
    n0=(-u0[1],u0[0]);n1=(-u1[1],u1[0])
    det=_cross(n0,n1)
    if abs(det)<math.sin(math.radians(3.0)):return None
    w0=D0*math.sin(ALPHA)+2e-5;w1=D1*math.sin(ALPHA)+2e-5
    b0=n0[0]*obs0['x']+n0[1]*obs0['y']
    b1=n1[0]*obs1['x']+n1[1]*obs1['y']
    verts=[]
    for s0 in (-1.,1.):
        for s1 in (-1.,1.):
            c0=b0+s0*w0;c1=b1+s1*w1
            x=(c0*n1[1]-n0[1]*c1)/det
            y=(n0[0]*c1-c0*n1[0])/det
            verts.append((x,y))
    # Convex cyclic order around centroid, useful for diagnostics only.
    cx=sum(x for x,y in verts)/4;cy=sum(y for x,y in verts)/4
    verts.sort(key=lambda p:math.atan2(p[1]-cy,p[0]-cx))
    return verts


def _segment_polygon_clear_plan(a,b,poly,R=STRIP_CLEAR_R,max_clears=24):
    """Clear a whole convex polygon using centers constrained to segment a--b.

    If every polygon vertex has normal distance <=D<R and its along-segment
    projection lies in [smin,smax] within the segment, centers spaced so that every
    along-coordinate is within h=sqrt(R^2-D^2) cover the entire polygon.  This is a
    rectangle outer-cover argument, hence conservative for the polygon interior.
    """
    L=math.dist(a,b)
    if L<1e-9:return None
    ux,uy=(b[0]-a[0])/L,(b[1]-a[1])/L;nx,ny=-uy,ux
    ss=[];ds=[]
    for p in poly:
        dx,dy=p[0]-a[0],p[1]-a[1]
        ss.append(dx*ux+dy*uy);ds.append(dx*nx+dy*ny)
    smin,smax=min(ss),max(ss);D=max(abs(x) for x in ds)
    if D>=R-1e-8 or smin<-1e-7 or smax>L+1e-7:return None
    h=math.sqrt(max(0.,R*R-D*D))
    span=max(0.,smax-smin)
    n=max(1,math.ceil(span/(2*h))) if h>1e-12 else max_clears+1
    if n>max_clears:return None
    centers=[]
    if span<1e-10:
        ss2=[(smin+smax)/2]
    else:
        step=span/n;ss2=[smin+(i+.5)*step for i in range(n)]
    for s in ss2:
        centers.append((a[0]+s*ux,a[1]+s*uy,s/L))
    return {'centers':centers,'count':n,'normal_bound_m':D,'span_m':span}


class G37RouteStripClear(G36SafeDistanceDisks):
    """Use safe directions to clear sources for zero extra movement on later paid legs."""
    route_strip_max_clears=20
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.safe_direction_records={}
        self.stats.update({'strip_route_plans':0,'strip_route_clear_attempts':0,
                           'strip_route_clear_successes':0,'strip_route_invariant_failures':0,
                           'safe_direction_records':0})

    def _record_safe_result(self,c,p,body,where):
        obs0=self._first_obs(c)
        rho=safe_rho(obs0,p) if obs0 is not None else None
        super()._record_safe_result(c,p,body,where)
        if body['measure_result']=='direction' and rho is not None and rho<=SAFE_R+3e-6:
            rec={'x':p[0],'y':p[1],'svd_deg':body['svd_deg'],'D':rho+3e-6}
            lst=self.safe_direction_records.setdefault(c,[])
            if all(math.dist(p,(z['x'],z['y']))>1e-7 for z in lst):
                lst.append(rec);self.stats['safe_direction_records']+=1

    def _bearing_records(self,c):
        t=self.tracks[c]
        if not t.dirs:return []
        safe={(round(z['x'],7),round(z['y'],7)):z['D'] for z in self.safe_direction_records.get(c,[])}
        out=[]
        for o in t.dirs:
            D=safe.get((round(o['x'],7),round(o['y'],7)),MAX_SOURCE_RANGE)
            out.append((o,D))
        return out

    def _route_strip_plan(self,c,start,destination):
        rec=self._bearing_records(c)
        if len(rec)<2:return None
        best=None
        for i,(a,Da) in enumerate(rec):
            for b,Db in rec[i+1:]:
                # At least one narrow certified strip is required for realistic <=20m tube coverage.
                if min(Da,Db)>SAFE_R+1e-4:continue
                poly=_strip_polygon(a,Da,b,Db)
                if poly is None:continue
                plan=_segment_polygon_clear_plan(start,destination,poly,max_clears=self.route_strip_max_clears)
                if plan is None:continue
                key=(plan['count'],plan['span_m'],plan['normal_bound_m'])
                if best is None or key<best[0]:best=(key,plan)
        return None if best is None else best[1]

    def transit_opportunities(self,destination,exclude_channel=None):
        # Start from the G35/G36 safe-measure planning logic, but reproduce it here so
        # certified strip-clear centers can be merged into the same monotone paid leg.
        start=self.port.position;length=math.dist(start,destination)
        if length<1e-7:return
        plan=[];options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            safe=self._best_safe_on_leg(c,start,destination)
            if safe is not None:
                score,u,p,c=safe;options.append((score+20000.,u,p,c,True));continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c,False)
            if best is not None:options.append(best)
        for score,u,p,c,safe in sorted(options,reverse=True)[:self.transit_per_leg]:
            plan.append((u,2,'measure',c,p,safe,None))
        # Existing exact guaranteed-clear region intersections.
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or not t.mec or t.mec[1]>20-1e-6 or not t.geometry:continue
            verts=t.geometry.get('polygon',[]);interval=segment_clear_interval(start,destination,verts)
            if interval is None:continue
            u=interval[0];p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
            plan.append((u,0,'clear',c,p,False,None))
        # New zero-detour strip-cover clear groups.
        gid=0
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None:continue
            z=self._route_strip_plan(c,start,destination)
            if z is None:continue
            gid+=1;self.stats['strip_route_plans']+=1
            centers=z['centers']
            for j,(p0,p1,u) in enumerate((p[0],p[1],p[2]) for p in centers):
                plan.append((u,1,'strip_clear',c,(p0,p1),False,(gid,j==len(centers)-1)))
        plan.sort(key=lambda x:(x[0],x[1]))
        strip_done=set()
        for u,_,kind,c,p,safe,extra in plan:
            if c in self.cleared:continue
            if kind=='clear':
                t=self.tracks[c]
                if not t.geometry or not all(math.dist(p,v)<=R_CLEAR+1e-7 for v in t.geometry.get('polygon',[])):continue
                self.clear(p,c,True,'B4_transit_full_region_clear');self.stats['transit_full_region_clears']+=1
            elif kind=='strip_clear':
                gid,last=extra
                if gid in strip_done:continue
                self.stats['strip_route_clear_attempts']+=1
                if self.clear(p,c,False,'B4_strip_route_clear'):
                    strip_done.add(gid);self.stats['strip_route_clear_successes']+=1
                elif last:
                    self.stats['strip_route_invariant_failures']+=1
                    raise RuntimeError('certified strip route cover exhausted without clearing')
            else:
                t=self.tracks[c]
                if t.near is not None or self._already_sampled(t,p):continue
                body=self.observe(p,c,'B4_transit_safe_probe' if safe else 'B4_transit_opportunity',backside=bool(safe))
                self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result']
                if k=='direction':self.stats['transit_opportunity_directions']+=1
                elif k=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
                if safe:self._record_safe_result(c,p,body,'transit')

class G37SafeRouteClear(G37RouteStripClear):
    pass

def _line_intersection_obs(a,b):
    u=_unit(a['svd_deg']);v=_unit(b['svd_deg']);det=_cross(u,v)
    if abs(det)<1e-10:return None
    d=(b['x']-a['x'],b['y']-a['y']);t=_cross(d,v)/det
    return (a['x']+t*u[0],a['y']+t*u[1])


def _acute_line_angle_deg(a,b):
    d=abs((a-b+180.)%360.-180.)
    return min(d,180.-d)


def _active_strip_centers(a,Da,b,Db,here,R=STRIP_CLEAR_R,max_clears=30):
    """Certified 1-D disk cover of the two-strip parallelogram.

    Use the narrower strip as the sweep axis. The intersection of the two centerlines
    is the parallelogram center.  A rectangle of half-width w_narrow and half-length S
    contains the whole parallelogram, so equally spaced clear centers on that axis cover
    it whenever their worst-corner distance is <20 m.
    """
    if Db<Da:a,Da,b,Db=b,Db,a,Da
    gamma=math.radians(_acute_line_angle_deg(a['svd_deg'],b['svd_deg']))
    if gamma<math.radians(3.):return None
    wA=Da*math.sin(ALPHA)+2e-5;wB=Db*math.sin(ALPHA)+2e-5
    if wA>=R:return None
    center=_line_intersection_obs(a,b)
    if center is None:return None
    s=abs(math.sin(gamma));co=abs(math.cos(gamma))
    S=(wB+wA*co)/s
    h=math.sqrt(max(0.,R*R-wA*wA))
    n=max(1,math.ceil(S/h)) if h>1e-12 else max_clears+1
    if n>max_clears:return None
    u=_unit(a['svd_deg']);step=2*S/n
    pts=[(center[0]+(-S+(i+.5)*step)*u[0],center[1]+(-S+(i+.5)*step)*u[1]) for i in range(n)]
    if math.dist(here,pts[-1])<math.dist(here,pts[0]):pts.reverse()
    movement=math.dist(here,pts[0])+sum(math.dist(x,y) for x,y in zip(pts,pts[1:]))
    worst_seconds=movement/5.+3.*(n-1)+5.
    return {'centers':pts,'count':n,'movement_m':movement,'worst_seconds':worst_seconds,
            'gamma_deg':math.degrees(gamma),'narrow_D_m':Da}


class _G38ActiveStrip(G37RouteStripClear):
    strip_seconds_limit=100.
    active_strip_max_clears=20
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.stats.update({'active_strip_tasks':0,'active_strip_attempts':0,
                           'active_strip_successes':0,'active_strip_invariant_failures':0})

    def _best_active_strip(self,c):
        rec=self._bearing_records(c)
        if len(rec)<2:return None
        best=None
        for i,(a,Da) in enumerate(rec):
            for b,Db in rec[i+1:]:
                if min(Da,Db)>SAFE_R+1e-4:continue
                z=_active_strip_centers(a,Da,b,Db,self.port.position,max_clears=self.active_strip_max_clears)
                if z is None or z['worst_seconds']>self.strip_seconds_limit:continue
                key=(z['worst_seconds'],z['count'],z['movement_m'])
                if best is None or key<best[0]:best=(key,z)
        return None if best is None else best[1]

    def source_task(self,c):
        base=super().source_task(c)
        # Never replace immediate near/guaranteed/anchor/finite fallback or the cheap
        # G34 center probe.  Only replace an active fan trip with a certified local sweep.
        if base[0]!='fan':return base
        z=self._best_active_strip(c)
        if z is None:return base
        return ('strip',c,z['centers'][0])

    def execute_source(self,task):
        if task[0]!='strip':return super().execute_source(task)
        _,c,_=task
        z=self._best_active_strip(c)
        if z is None:
            return super().execute_source(super().source_task(c))
        self.stats['active_strip_tasks']+=1
        self.port.record({'kind':'B4_active_strip','channel':c,'count':z['count'],
                          'worst_seconds':z['worst_seconds'],'movement_m':z['movement_m'],
                          'gamma_deg':z['gamma_deg'],'narrow_D_m':z['narrow_D_m']})
        for j,p in enumerate(z['centers']):
            self.stats['active_strip_attempts']+=1
            if self.clear(p,c,False,'B4_active_strip_clear'):
                self.stats['active_strip_successes']+=1;return
        self.stats['active_strip_invariant_failures']+=1
        raise RuntimeError('certified active strip cover exhausted without clearing')

class G38S75(_G38ActiveStrip):strip_seconds_limit=75.
class G38S100(_G38ActiveStrip):strip_seconds_limit=100.
class G38S125(_G38ActiveStrip):strip_seconds_limit=125.

def predicted_second_clear_count(obs,p,R=STRIP_CLEAR_R):
    """Pre-measurement worst-case clear count if q returns direction.

    Uses only the first +/-alpha sector and q's certified rho, so it is valid before
    seeing the second bearing.  The same-side far endpoint gives the smallest possible
    second measured-line angle; the two-strip rectangle bound then yields a clear count.
    """
    x,y=_local(obs,p);rho=safe_rho(obs,p)
    ay=abs(y);sx=MAX_SOURCE_RANGE*math.cos(ALPHA);sy=MAX_SOURCE_RANGE*math.sin(ALPHA)
    delta=math.atan2(max(0.,ay-sy),sx-x)
    gamma=max(0.,delta-ALPHA)
    if gamma<=1e-10:return 10**9
    w0=MAX_SOURCE_RANGE*math.sin(ALPHA)+2e-5;w1=rho*math.sin(ALPHA)+2e-5
    if w1>=R:return 10**9
    S=(w0+w1*abs(math.cos(gamma)))/math.sin(gamma)
    h=math.sqrt(max(0.,R*R-w1*w1))
    return max(1,math.ceil(S/h)) if h>1e-12 else 10**9

class _G39SelectiveSafe(_G38ActiveStrip):
    predicted_clear_limit=10
    strip_seconds_limit=100.
    def _safe_site(self,c,p,side=0):
        if not super()._safe_site(c,p,side):return False
        return predicted_second_clear_count(self._first_obs(c),p)<=self.predicted_clear_limit
    def _best_safe_on_leg(self,c,start,destination):
        z=super()._best_safe_on_leg(c,start,destination)
        if z is None:return None
        _,_,p,_=z
        return z if predicted_second_clear_count(self._first_obs(c),p)<=self.predicted_clear_limit else None

class G39P8(_G39SelectiveSafe):predicted_clear_limit=8
class G39P10(_G39SelectiveSafe):predicted_clear_limit=10
class G39P12(_G39SelectiveSafe):predicted_clear_limit=12
class G39P15(_G39SelectiveSafe):predicted_clear_limit=15

class G40PassiveSafe(_G38ActiveStrip):
    """Use safe mathematics only on measurements G34 would already take.

    No extra probe is created: certificate opportunities use the original G2 score;
    transit opportunities use the original fixed fractions.  If such a paid measurement
    happens to satisfy rho<=1000, it is upgraded to a proven-backside-capable safe probe
    and can feed the active strip resolver.
    """
    strip_seconds_limit=100.
    def has_future_opportunity(self,c,remaining):
        # Exactly the inherited ordinary G2/G34 deferral criterion; do not defer a fan
        # for a safe point that this passive variant would not necessarily measure.
        return G34R80.has_future_opportunity(self,c,remaining)

    def scan_certificate(self,site):
        G1Solver.scan_certificate(self,site)
        p=self.mesh['vertices'][site];candidates=[]
        for c,t in self.tracks.items():
            if c in self.cleared:continue
            score=self.opportunity_score(c,p)
            if score is not None:candidates.append((score,c))
        candidates.sort(reverse=True)
        for _,c in candidates[:self.opportunity_per_site]:
            if c in self.cleared or self._already_sampled(self.tracks[c],p):continue
            safe=self._first_obs(c) is not None and safe_rho(self._first_obs(c),p)<=SAFE_R+1e-6
            body=self.observe(p,c,'B4_certificate_opportunity',backside=safe)
            self.stats['opportunity_measurements']+=1
            k=body['measure_result']
            if k=='direction':self.stats['opportunity_directions']+=1
            elif k=='near':self.stats['opportunity_near']+=1
            else:self.stats['opportunity_no_signal']+=1
            if safe:self._record_safe_result(c,p,body,'site')

    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position;length=math.dist(start,destination)
        if length<1e-7:return
        plan=[];options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c)
            if best is not None:options.append(best)
        for score,u,p,c in sorted(options,reverse=True)[:self.transit_per_leg]:plan.append((u,1,'measure',c,p))
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or not t.mec or t.mec[1]>20-1e-6 or not t.geometry:continue
            verts=t.geometry.get('polygon',[]);interval=segment_clear_interval(start,destination,verts)
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
                safe=self._first_obs(c) is not None and safe_rho(self._first_obs(c),p)<=SAFE_R+1e-6
                body=self.observe(p,c,'B4_transit_opportunity',backside=safe)
                self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result']
                if k=='direction':self.stats['transit_opportunity_directions']+=1
                elif k=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
                if safe:self._record_safe_result(c,p,body,'transit')
