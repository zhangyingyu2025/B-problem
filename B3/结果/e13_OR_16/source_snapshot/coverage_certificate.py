"""Finite Voronoi/circle coverage certificate using rational enclosing intervals.

No grid sampling is used for the decision. Coordinates are exact binary-float
rationals; each algebraic square root is enclosed with integer square root.
"""
from fractions import Fraction as F
from itertools import combinations
import math


def sqrt_interval(value, scale=10**30):
    if value < 0:
        raise ValueError('negative square root')
    n = math.isqrt(value.numerator*scale*scale//value.denominator)
    lo = F(n, scale)
    return lo, lo if lo*lo == value else F(n+1, scale)


def times(interval, value):
    a, b = interval[0]*value, interval[1]*value
    return min(a, b), max(a, b)


def certificate(sites, arena_radius=1800, reception_radius=1000):
    points = list(dict.fromkeys((F(x), F(y)) for x, y in sites))
    if not points:
        return {'status': 'not_covered', 'radius_upper_m': None}
    R = F(arena_radius); candidates = []
    def point(x, y):
        candidates.append(((x, x), (y, y)))
    point(R, F(0)); point(-R, F(0)); point(F(0), R); point(F(0), -R)
    # Interior Voronoi vertices are circumcenters of at least three sites.
    for a, b, c in combinations(points, 3):
        nx, ny = 2*(b[0]-a[0]), 2*(b[1]-a[1])
        mx, my = 2*(c[0]-a[0]), 2*(c[1]-a[1])
        d = b[0]**2+b[1]**2-a[0]**2-a[1]**2
        e = c[0]**2+c[1]**2-a[0]**2-a[1]**2
        det = nx*my-ny*mx
        if not det: continue
        x, y = (d*my-ny*e)/det, (nx*e-d*mx)/det
        if x*x+y*y <= R*R: point(x, y)
    # Boundary nearest-site changes occur at circle/bisector intersections.
    for a, b in combinations(points, 2):
        nx, ny = 2*(b[0]-a[0]), 2*(b[1]-a[1]); norm2 = nx*nx+ny*ny
        d = b[0]**2+b[1]**2-a[0]**2-a[1]**2
        if not norm2: continue
        t2 = (R*R-d*d/norm2)/norm2
        if t2 < 0: continue
        t = sqrt_interval(t2); x0, y0 = nx*d/norm2, ny*d/norm2
        for sign in (-1, 1):
            dx, dy = times(t, -sign*ny), times(t, sign*nx)
            candidates.append(((x0+dx[0], x0+dx[1]), (y0+dy[0], y0+dy[1])))
    # On an arc with a fixed nearest site, distance extrema are radial.
    for x, y in points:
        norm2 = x*x+y*y
        if not norm2: continue
        t = sqrt_interval(R*R/norm2)
        for sign in (-1, 1):
            candidates.append((times(t, sign*x), times(t, sign*y)))
    best_hi = F(0); best_lo = F(0); witness = None
    for xi, yi in candidates:
        uppers, lowers = [], []
        for x, y in points:
            dx, dy = (xi[0]-x, xi[1]-x), (yi[0]-y, yi[1]-y)
            uppers.append(max(dx[0]**2, dx[1]**2)+max(dy[0]**2, dy[1]**2))
            lx = 0 if dx[0] <= 0 <= dx[1] else min(dx[0]**2, dx[1]**2)
            ly = 0 if dy[0] <= 0 <= dy[1] else min(dy[0]**2, dy[1]**2)
            lowers.append(lx+ly)
        upper, lower = min(uppers), min(lowers)
        if upper > best_hi:
            best_hi = upper; witness = [float(sum(xi)/2), float(sum(yi)/2)]
        best_lo = max(best_lo, lower)
    limit2 = F(reception_radius)**2
    status = 'covered' if best_hi <= limit2 else 'not_covered' if best_lo > limit2 else 'numerically_uncertain'
    return {'status': status, 'radius_upper_m': math.nextafter(math.sqrt(float(best_hi)), math.inf),
            'radius_lower_m': math.nextafter(math.sqrt(float(best_lo)), -math.inf),
            'witness': witness, 'candidate_count': len(candidates),
            'method': 'finite Voronoi candidates; rational sqrt enclosures; exact threshold comparison'}
