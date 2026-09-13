"""Exact certificate for directional-source discovery using local convex hulls.

A directional source at g with guaranteed reception radius at least L=1000 m is
seen from a test site s whenever ||s-g||<=L and s lies in the source's emitting
closed half-plane through g.  Let N(g) be the tested sites within L of g.  Every
possible emitting half-plane contains a member of N(g) iff g lies in conv(N(g)).
(The reverse direction is the separating-hyperplane theorem.)

``verify_local_convex_certificate`` proves this condition for every g in the target
disk.  It recursively partitions the bounding square into rational axis-aligned
boxes.  For a box B, let U be the sites whose L-disks contain all four corners of B.
Then their disks contain the whole box (squared distance is convex), and if all four
corners of B lie in conv(U), convexity puts the whole box in conv(U).  Such a box is
therefore certified at once.  Boxes strictly outside the target disk are discarded;
all others are subdivided until certified.  All predicates use Fraction arithmetic
on the supplied binary floating-point coordinates, matching the project's B1 policy.
"""
from __future__ import annotations
from fractions import Fraction as F
import math

_VERIFY_CACHE={}


def _cross(o,a,b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def _hull(points):
    pts=sorted(set(points))
    if len(pts)<=1:return pts
    lo=[]
    for p in pts:
        while len(lo)>=2 and _cross(lo[-2],lo[-1],p)<=0:lo.pop()
        lo.append(p)
    up=[]
    for p in reversed(pts):
        while len(up)>=2 and _cross(up[-2],up[-1],p)<=0:up.pop()
        up.append(p)
    return lo[:-1]+up[:-1]


def _inside_convex(poly,p):
    return len(poly)>=3 and all(_cross(poly[i],poly[(i+1)%len(poly)],p)>=0 for i in range(len(poly)))


def _corners(box):
    x0,x1,y0,y1=box
    return ((x0,y0),(x0,y1),(x1,y0),(x1,y1))


def _outside_target(box,R2):
    x0,x1,y0,y1=box
    x=F(0) if x0<=0<=x1 else min(abs(x0),abs(x1))
    y=F(0) if y0<=0<=y1 else min(abs(y0),abs(y1))
    return x*x+y*y>R2


def _box_covered(box,sites,L2):
    cs=_corners(box); universal=[]
    for s in sites:
        if max((p[0]-s[0])**2+(p[1]-s[1])**2 for p in cs)<=L2:
            universal.append(s)
    if len(universal)<3:return False
    h=_hull(universal)
    return all(_inside_convex(h,p) for p in cs)


def verify_local_convex_certificate(mesh, initial_grid=64, max_boxes=250000):
    """Return a proof summary or raise ValueError if the supplied sites are not certified.

    No tolerance is used.  The recursive proof normally terminates at boxes several
    metres wide for the G18 21-site design; ``max_boxes`` is only a computation guard,
    not a geometric acceptance threshold.
    """
    raw=mesh.get('vertices',[])
    if len(raw)<3 or any(len(p)!=2 for p in raw):raise ValueError('invalid certificate vertices')
    cache_key=(tuple((float(p[0]),float(p[1])) for p in raw), float(mesh.get('target_radius',1800.0)), float(mesh.get('range',1000.0)), initial_grid)
    if cache_key in _VERIFY_CACHE:
        return dict(_VERIFY_CACHE[cache_key])
    sites=[tuple(F(float(x)) for x in p) for p in raw]
    if len(set(sites))!=len(sites):raise ValueError('duplicate certificate vertices')
    R=F(mesh.get('target_radius',1800.0));L=F(mesh.get('range',1000.0));R2=R*R;L2=L*L
    if initial_grid<=0 or initial_grid&(initial_grid-1):raise ValueError('initial_grid must be positive power of two')
    # Exact grid over [-R,R]^2. For R=1800 and 64 this starts at 56.25 m boxes.
    step=2*R/initial_grid;stack=[]
    for i in range(initial_grid):
        x0=-R+i*step;x1=x0+step
        for j in range(initial_grid):
            y0=-R+j*step;y1=y0+step;b=(x0,x1,y0,y1)
            if not _outside_target(b,R2):stack.append(b)
    processed=split=0;smallest=None;largest_certified=F(0)
    while stack:
        b=stack.pop();processed+=1
        if processed>max_boxes:raise ValueError('local-convex proof box budget exhausted')
        if _outside_target(b,R2):continue
        if _box_covered(b,sites,L2):
            w=max(b[1]-b[0],b[3]-b[2]);largest_certified=max(largest_certified,w)
            continue
        x0,x1,y0,y1=b;w=x1-x0;h=y1-y0
        smallest=max(w,h) if smallest is None else min(smallest,max(w,h))
        # There is deliberately no epsilon acceptance.  If the geometry has no open
        # certificate margin, recursion will hit the computation guard and fail.
        if w>=h:
            m=(x0+x1)/2;stack.append((x0,m,y0,y1));stack.append((m,x1,y0,y1))
        else:
            m=(y0+y1)/2;stack.append((x0,x1,y0,m));stack.append((x0,x1,m,y1))
        split+=1
    result={'valid':True,'kind':'local_convex_directional_discovery',
            'vertices':len(sites),'target_radius_m':float(R),'guaranteed_range_m':float(L),
            'boxes_processed':processed,'boxes_split':split,
            'smallest_split_box_m':None if smallest is None else float(smallest),
            'largest_certified_box_m':float(largest_certified),
            'proof':'every target point lies in the convex hull of test sites guaranteed within reception range',
            'numeric_domain':'exact rational arithmetic on stored binary vertex coordinates and dyadic boxes'}
    _VERIFY_CACHE[cache_key]=dict(result)
    return result


# Final-certificate coordinates are frozen as binary64 literals.  The previous
# implementation generated these regular rings with platform libm sin/cos.
# The geometry is symmetric, so ulp-scale Windows/Linux trig differences could
# change a near-tied route orientation and cascade into different opportunity
# measurements.  Freezing the final 1868.0/997.5 design removes that platform
# dependency without changing a single final-coordinate bit on the reference run.
_FINAL_1868_9975 = (
    (0.0, 0.0),
    (1868.0, 0.0),
    (1617.7354542693315, 933.9999999999999),
    (934.0000000000002, 1617.7354542693313),
    (1.1438201104036278e-13, 1868.0),
    (-933.9999999999995, 1617.7354542693315),
    (-1617.7354542693315, 933.9999999999999),
    (-1868.0, 2.2876402208072556e-13),
    (-1617.7354542693313, -934.0000000000002),
    (-934.0000000000008, -1617.7354542693308),
    (-3.4314603312108834e-13, -1868.0),
    (934.0000000000002, -1617.7354542693313),
    (1617.7354542693308, -934.0000000000008),
    (997.5, 0.0),
    (705.3390142335812, 705.3390142335811),
    (6.107925910747424e-14, 997.5),
    (-705.3390142335811, 705.3390142335812),
    (-997.5, 1.2215851821494847e-13),
    (-705.3390142335813, -705.3390142335811),
    (-1.832377773224227e-13, -997.5),
    (705.339014233581, -705.3390142335813),
)

def mesh21(outer_radius=1868.7,inner_radius=999.9):
    """21-site candidate; the final design uses frozen cross-platform coordinates."""
    if float(outer_radius) == 1868.0 and float(inner_radius) == 997.5:
        vertices=list(_FINAL_1868_9975)
    else:
        polar=lambda r,a:(r*math.cos(math.radians(a)),r*math.sin(math.radians(a)))
        vertices=[(0.,0.)]
        vertices += [polar(outer_radius,30*k) for k in range(12)]
        vertices += [polar(inner_radius,45*k) for k in range(8)]
    return {'name':'G18-local-convex-21','vertices':vertices,
            'target_radius':1800.0,'range':1000.0,
            'certificate_kind':'local_convex'}
