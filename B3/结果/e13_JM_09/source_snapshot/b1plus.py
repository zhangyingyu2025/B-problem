"""B3-only conservative intersection of bearings, arena, reception and silence.

Exact rational clipping preserves the supplied binary halfplane coefficients.
Circle support offsets round outward. This is not an inner disk polygon.
"""
from fractions import Fraction as F
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'B1/code'))
import b1_geometry as b1


def clip(poly, plane):
    a, b, c = (F(v) for v in plane)
    if not poly:
        return []
    out = []
    previous = poly[-1]; old = a*previous[0]+b*previous[1]-c
    for point in poly:
        value = a*point[0]+b*point[1]-c
        if (old <= 0) != (value <= 0):
            t = old/(old-value)
            out.append((previous[0]+t*(point[0]-previous[0]), previous[1]+t*(point[1]-previous[1])))
        if value <= 0:
            out.append(point)
        previous, old = point, value
    return out


def circle_planes(center, radius, count=64):
    for k in range(count):
        a = math.cos(2*math.pi*k/count); b = math.sin(2*math.pi*k/count)
        # u need not be exactly unit after floating-point trigonometry.
        rhs = a*center[0]+b*center[1]+radius*math.hypot(a, b)
        yield a, b, math.nextafter(rhs+1e-8*(1+abs(rhs)/10000), math.inf)


def intersection(directions, signals, silent, circle_count=64):
    bearings = b1.observations_to_halfplanes(directions)[0] if directions else []
    planes = bearings+list(circle_planes((0., 0.), 1800, circle_count))
    for s in signals:
        planes += list(circle_planes(s, 1500, circle_count))
        for q in silent:
            # < is relaxed to <=: no genuine feasible source is removed.
            qx, qy, sx, sy = map(F, (q[0], q[1], s[0], s[1]))
            planes.append((2*(qx-sx), 2*(qy-sy), qx*qx+qy*qy-sx*sx-sy*sy))
    poly = [(F(-2000), F(-2000)), (F(2000), F(-2000)), (F(2000), F(2000)), (F(-2000), F(2000))]
    # Bearings first keep the exact arithmetic polygon small during disk clipping.
    for plane in planes:
        poly = clip(poly, plane)
        if not poly:
            return {'status': 'numerically_uncertain', 'polygon': [], 'mec': None,
                    'diagnostic': 'empty B3 outer intersection; do not infer source absence'}
    vertices = [(float(p[0]), float(p[1])) for p in poly]
    # Exact B1 minimum-circle helper on this bounded convex polygon.
    center, radius, _ = b1._minimum_circle(poly)
    center = tuple(float(v) for v in center)
    radius = max(math.dist(center, p) for p in vertices)+1e-7
    pairs = [(math.dist(a, b), a, b) for i, a in enumerate(vertices) for b in vertices[i:]]
    diameter, first, last = max(pairs, key=lambda x: x[0])
    centroid = (sum(p[0] for p in vertices)/len(vertices), sum(p[1] for p in vertices)/len(vertices))
    if directions:
        theta = math.radians(directions[0]['svd_deg']); u = (math.cos(theta), math.sin(theta)); v = (-u[1], u[0])
        width = lambda axis: max(p[0]*axis[0]+p[1]*axis[1] for p in vertices)-min(p[0]*axis[0]+p[1]*axis[1] for p in vertices)
        longitudinal, lateral = width(u), width(v)
    else:
        longitudinal = lateral = diameter
    return {'status': 'bounded', 'polygon': vertices, 'mec': (center, radius), 'diameter': diameter,
            'centers': [center, centroid], 'axis_ends': [first, last], 'longitudinal': longitudinal,
            'lateral': lateral, 'signal_count': len(signals), 'silent_count': len(silent),
            'halfplane_count': len(planes), 'guaranteed_center_clear': radius <= 20}
