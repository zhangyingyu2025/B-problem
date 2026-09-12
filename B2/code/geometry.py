"""Small float geometry kernel, with independent Decimal/Fraction rechecks.

Fast evaluations are restricted to the analytically bounded B2 search domain.
B1 is imported read-only for independent feasibility, recession and exact
support enumeration in high-precision checks. No interval-certification claim.
"""
from decimal import Decimal, localcontext
from fractions import Fraction
from itertools import combinations
from pathlib import Path
import math
import sys

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_B1_CODE = next((q for q in (_PROJECT_ROOT / "B1" / "code", _PROJECT_ROOT / "问题一" / "code") if q.is_dir()), None)
if _B1_CODE is None:
    raise ImportError("B1 code directory not found; expected sibling B1/code")
sys.path.insert(0, str(_B1_CODE))
import b1_geometry as b1

PI = Decimal("3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148")


class GeometryError(ArithmeticError):
    pass


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


def dot(a, b):
    return a[0]*b[0]+a[1]*b[1]


def sub(a, b):
    return (a[0]-b[0], a[1]-b[1])


def unit(deg):
    return b1._unit(deg)


def hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points
    chains = []
    for seq in (points, reversed(points)):
        chain = []
        for p in seq:
            while len(chain)>1 and cross(sub(chain[-1],chain[-2]),sub(p,chain[-1])) <= 0:
                chain.pop()
            chain.append(p)
        chains.append(chain[:-1])
    return chains[0]+chains[1]


def minimum_circle(vertices):
    best = None
    scale = max(1.0, *(math.hypot(*p) for p in vertices))
    tol = 2e-10*scale

    def consider(center, support):
        nonlocal best
        # Use maximum distance, not a tolerance-shrunken radius.
        r = max(math.dist(center,p) for p in vertices)
        support_r = max(math.dist(center,vertices[i]) for i in support)
        if r > support_r+tol:
            return
        if best is None or r < best[1]:
            best = (center, r, list(support))

    for i,p in enumerate(vertices):
        consider(p,[i])
    for i,j in combinations(range(len(vertices)),2):
        a,b = vertices[i],vertices[j]
        consider(((a[0]+b[0])/2,(a[1]+b[1])/2),[i,j])
    for i,j,k in combinations(range(len(vertices)),3):
        a=vertices[i];u=sub(vertices[j],a);v=sub(vertices[k],a)
        det=2*cross(u,v)
        if abs(det) <= 1e-13*max(1,math.hypot(*u)*math.hypot(*v)):
            continue
        uu,vv=dot(u,u),dot(v,v)
        c=(a[0]+(uu*v[1]-vv*u[1])/det,a[1]+(u[0]*vv-v[0]*uu)/det)
        consider(c,[i,j,k])
    if best is None:
        raise GeometryError("minimum circle support recovery failed")
    return best


def metrics(p, theta, alpha=1.0, second_alpha=None):
    second_alpha=alpha if second_alpha is None else second_alpha
    hs=b1.observations_to_halfplanes([{"x":0,"y":0,"svd_deg":0}],alpha)[0]
    hs+=b1.observations_to_halfplanes([{"x":p[0],"y":p[1],"svd_deg":theta%360}],second_alpha)[0]
    points=[]
    scale=max(1,math.hypot(*p))
    for h,k in combinations(hs,2):
        det=h.ax*k.ay-h.ay*k.ax
        if abs(det)<1e-12:
            continue
        q=((h.b*k.ay-h.ay*k.b)/det,(h.ax*k.b-h.b*k.ax)/det)
        if all(a*q[0]+b*q[1] <= c+1e-10*max(scale,math.hypot(*q)) for a,b,c in hs):
            points.append(q)
    vertices=hull(points)
    if not vertices:
        raise GeometryError("no vertices in analytically nonempty bounded intersection")
    pair=max(combinations(range(len(vertices)),2),key=lambda ij:math.dist(vertices[ij[0]],vertices[ij[1]]),default=(0,0))
    diameter=math.dist(vertices[pair[0]],vertices[pair[1]])
    center,radius,support=minimum_circle(vertices)
    if not math.isfinite(diameter+radius):
        raise GeometryError("nonfinite metric")
    return {"diameter_m":diameter,"mec_radius_m":radius,"vertices":vertices,
            "farthest_pair_indices":pair,"mec_center":center,"mec_support_indices":support}


def decimal_unit(angle, digits=65):
    with localcontext() as ctx:
        ctx.prec=digits+12
        deg=Decimal(str(angle)) % Decimal(360)
        if deg>180: deg-=360
        if deg< -180: deg+=360
        if deg==0: return Decimal(1),Decimal(0)
        x=deg*PI/180
        x2=x*x
        sin_t=sin_s=x
        cos_t=cos_s=Decimal(1)
        threshold=Decimal(10)**(-digits-5)
        for n in range(1,500):
            sin_t *= -x2/((2*n)*(2*n+1))
            cos_t *= -x2/((2*n-1)*(2*n))
            sin_s+=sin_t;cos_s+=cos_t
            if max(abs(sin_t),abs(cos_t))<threshold: break
        else: raise GeometryError("Decimal Taylor series did not converge")
        ctx.prec=digits
        return +cos_s,+sin_s


def precise_metrics(p, theta, alpha=1.0, second_alpha=None, digits=65):
    second_alpha=alpha if second_alpha is None else second_alpha
    hs=[]
    # Coordinate and angle values are checked as their decimal string values.
    for center,angle,half in [((0,0),0,alpha),(p,theta,second_alpha)]:
        x,y=(Fraction(Decimal(str(v))) for v in center)
        with localcontext() as ctx:
            ctx.prec=digits+12
            low=Decimal(str(angle))-Decimal(str(half))
            high=Decimal(str(angle))+Decimal(str(half))
        c0,s0=map(Fraction,decimal_unit(low,digits))
        c1,s1=map(Fraction,decimal_unit(high,digits))
        for a,b in [(s0,-c0),(-s1,c1)]:
            hs.append((a,b,a*x+b*y))
    feasible=b1._feasible(hs)
    if feasible is None: return {"status":"empty"}
    direction,_=b1._recession(hs)
    if direction is not None: return {"status":"unbounded"}
    vertices,ill=b1._vertices(hs)
    if ill or not vertices: return {"status":"numerically_uncertain"}
    d2,pair=b1._exact_diameter(vertices)
    center,r2,support=b1._minimum_circle(vertices)
    def sqrt(q):
        with localcontext() as ctx:
            ctx.prec=digits
            return (Decimal(q.numerator)/Decimal(q.denominator)).sqrt()
    return {"status":"bounded","diameter_m":float(sqrt(d2)),"mec_radius_m":float(sqrt(r2)),
            "diameter_decimal":str(sqrt(d2)),"mec_radius_decimal":str(sqrt(r2)),
            "digits":digits,"mec_support_indices":support,
            "vertices":[[float(v) for v in q] for q in vertices],
            "mec_center":[float(v) for v in center]}
