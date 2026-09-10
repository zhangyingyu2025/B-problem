"""B1 bearing-wedge geometry. Python standard library only.

Feasibility and recession classification use exact rational arithmetic on the
supplied binary floating-point coefficients, not an epsilon-expanded region.
This is not a claim of exact trigonometry: ill-conditioned active intersections
are explicitly marked numerically_uncertain. Distances use float arithmetic.
"""
from __future__ import annotations

from fractions import Fraction as F
from itertools import combinations
import math
from typing import NamedTuple

ABS_TOL_M = 1e-8
REL_TOL = 1e-12
PARALLEL_SIN_GUARD = 1e-10


class HalfPlane(NamedTuple):
    """Closed half-plane ax*x + ay*y <= b, in the original coordinates."""
    ax: float
    ay: float
    b: float


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _rational_planes(planes):
    result = []
    for i, h in enumerate(planes):
        if len(h) != 3:
            raise ValueError(f"halfplane {i} must have three coefficients")
        # Conversion preserves every bit of the represented coefficient.
        result.append(tuple(F(_number(v, f"halfplane {i}")) for v in h))
    return result


def _dot(a, p):
    return a[0] * p[0] + a[1] * p[1]


def _inside(p, hs):
    return all(_dot(h, p) <= h[2] for h in hs)


def _feasible(hs):
    """Incremental 2D LP feasibility via 1D boundary intervals, O(k^2).

No vertex enumerator is used. None interval endpoints mean infinity.
"""
    p = (F(0), F(0))
    for i, (a, b, c) in enumerate(hs):
        if a * p[0] + b * p[1] <= c:
            continue
        if a == b == 0:
            return None
        base = (c / a, F(0)) if abs(a) >= abs(b) else (F(0), c / b)
        direction = (-b, a)
        lo = hi = None
        for old in hs[:i]:
            coefficient = _dot(old, direction)
            rhs = old[2] - _dot(old, base)
            if coefficient == 0:
                if rhs < 0:
                    return None
                continue
            bound = rhs / coefficient
            if coefficient > 0:
                hi = bound if hi is None else min(hi, bound)
            else:
                lo = bound if lo is None else max(lo, bound)
            if lo is not None and hi is not None and lo > hi:
                return None
        t = F(0)
        if lo is not None and t < lo:
            t = lo
        if hi is not None and t > hi:
            t = hi
        p = (base[0] + t * direction[0], base[1] + t * direction[1])
    assert _inside(p, hs)
    return p


def _float_point(p):
    q = [float(p[0]), float(p[1])]
    if not all(math.isfinite(x) for x in q):
        raise OverflowError("point is outside floating-point output range")
    return q


def _exact_point(p):
    return [str(p[0]), str(p[1])]


def feasible_point(planes):
    """Public standalone feasibility check; None means the system is empty."""
    p = _feasible(_rational_planes(planes))
    return None if p is None else _float_point(p)


def _recession(hs):
    cone = [(a, b, F(0)) for a, b, _ in hs]
    checks = []
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
        a, b = (F(1), F(0)) if axis == 0 else (F(0), F(1))
        point = _feasible(cone + [(a, b, F(sign)), (-a, -b, F(-sign))])
        checks.append({"axis": "x" if axis == 0 else "y", "value": sign,
                       "feasible": point is not None})
        if point is not None:
            magnitude = max(abs(point[0]), abs(point[1]))
            direction = (point[0] / magnitude, point[1] / magnitude)
            assert direction != (0, 0) and _inside(direction, cone)
            return direction, checks
    return None, checks


def recession_direction(planes):
    """Nonzero direction of A*d<=0, or None. Does not test P's feasibility."""
    d, _ = _recession(_rational_planes(planes))
    return None if d is None else _float_point(d)


def _intersection(h, k):
    a, b, c = h
    d, e, f = k
    determinant = a * e - b * d
    if determinant == 0:
        return None
    return ((c * e - b * f) / determinant, (a * f - c * d) / determinant)


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _hull(points):
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts
    lower, upper = [], []
    for chain, sequence in ((lower, pts), (upper, reversed(pts))):
        for p in sequence:
            while len(chain) >= 2 and _cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    return lower[:-1] + upper[:-1]


def _vertices(hs):
    points, ill = [], []
    for i, j in combinations(range(len(hs)), 2):
        p = _intersection(hs[i], hs[j])
        if p is None or not _inside(p, hs):
            continue
        points.append(p)
        h, k = hs[i], hs[j]
        # Dimensionless squared sine; exact comparison avoids underflow.
        det = h[0] * k[1] - h[1] * k[0]
        sine2 = det * det / ((h[0]**2 + h[1]**2) * (k[0]**2 + k[1]**2))
        if sine2 < F(PARALLEL_SIN_GUARD)**2:
            ill.append([i, j])
    return _hull(points), ill


def _squared_distance(a, b):
    return (a[0] - b[0])**2 + (a[1] - b[1])**2


def _exact_diameter(vertices):
    if not vertices:
        raise ValueError("diameter needs a nonempty bounded region")
    best, pair = F(0), (0, 0)
    for i, j in combinations(range(len(vertices)), 2):
        distance = _squared_distance(vertices[i], vertices[j])
        if distance > best:
            best, pair = distance, (i, j)
    return best, pair


def _circle_for_three(a, b, c):
    # Work relative to a, avoiding subtraction of large squared coordinates.
    u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-a[0], c[1]-a[1])
    det = 2 * (u[0]*v[1] - u[1]*v[0])
    if det == 0:
        return None
    uu, vv = _dot(u, u), _dot(v, v)
    offset = ((uu*v[1] - u[1]*vv)/det, (u[0]*vv - uu*v[0])/det)
    center = (a[0]+offset[0], a[1]+offset[1])
    return center, _dot(offset, offset)


def _minimum_circle(vertices):
    """Independent support-set enumeration; exact containment, O(m^4)."""
    best = None

    def consider(center, radius2, support):
        nonlocal best
        if best is not None and radius2 >= best[1]:
            return
        if all(_squared_distance(p, center) <= radius2 for p in vertices):
            best = (center, radius2, list(support))

    for i, p in enumerate(vertices):
        consider(p, F(0), (i,))
    for i, j in combinations(range(len(vertices)), 2):
        a, b = vertices[i], vertices[j]
        center = ((a[0]+b[0])/2, (a[1]+b[1])/2)
        consider(center, _squared_distance(a, b)/4, (i, j))
    for i, j, k in combinations(range(len(vertices)), 3):
        candidate = _circle_for_three(vertices[i], vertices[j], vertices[k])
        if candidate is not None:
            consider(*candidate, (i, j, k))
    assert best is not None
    return best


def _circle_decision(vertices):
    diameter2, pair = _exact_diameter(vertices)
    a, b = (vertices[i] for i in pair)
    center = ((a[0]+b[0])/2, (a[1]+b[1])/2)
    radius2 = diameter2/4
    outside = [i for i, p in enumerate(vertices) if _squared_distance(p, center) > radius2]
    # The main verdict is complete BEFORE invoking the independent MEC routine.
    covers = not outside
    radius = math.sqrt(float(radius2))
    tolerance = ABS_TOL_M + REL_TOL * max(1.0, 2*radius)
    marginal = []
    for i, p in enumerate(vertices):
        delta2 = _squared_distance(p, center)-radius2
        # Exact equality is a genuine boundary case of the represented system.
        # A nonzero margin below output resolution must not be oversold.
        if delta2:
            denominator = math.sqrt(float(_squared_distance(p, center)))+radius
            margin = abs(float(delta2))/denominator if denominator else 0.0
            if margin <= tolerance:
                marginal.append(i)
    mc, mr2, support = _minimum_circle(vertices)
    independent = mr2 <= radius2
    return {
        "diameter_m": math.sqrt(float(diameter2)),
        "farthest_pair_indices": list(pair),
        "farthest_pair": [_float_point(a), _float_point(b)],
        "diameter_circle": {"center": _float_point(center), "radius_m": math.sqrt(float(radius2)),
                            "covers": covers, "uncovered_vertex_indices": outside,
                            "uncovered_vertices": [_float_point(vertices[i]) for i in outside]},
        "minimum_enclosing_circle": {"center": _float_point(mc), "radius_m": math.sqrt(float(mr2)),
                                     "diameter_m": 2*math.sqrt(float(mr2)),
                                     "support_indices": support},
        "circle_crosscheck_passed": covers == independent,
        "marginal_circle_vertex_indices": marginal,
    }


def _blank():
    return {"status": None, "algebraic_status": None, "feasible_point": None,
            "feasible_point_exact": None, "recession_direction": None,
            "recession_direction_exact": None, "recession_checks": [],
            "vertices": [], "diameter_m": None, "farthest_pair": None,
            "farthest_pair_indices": None, "diameter_circle": None,
            "minimum_enclosing_circle": None, "circle_crosscheck_passed": None,
            "diagnostics": [],
            "numeric_policy": {"coefficients": "exact rational arithmetic on input binary floats",
                               "absolute_output_tolerance_m": ABS_TOL_M,
                               "relative_output_tolerance": REL_TOL,
                               "active_parallel_sine_guard": PARALLEL_SIN_GUARD}}


def solve_halfplanes(planes):
    """General 2D half-plane solver; includes full-plane/strip/line cases."""
    hs = _rational_planes(planes)
    result = _blank()
    result["halfplanes"] = [[float(v) for v in h] for h in hs]
    redundant = [i for i, (a, b, c) in enumerate(hs) if a == b == 0 and c >= 0]
    if redundant:
        result["diagnostics"].append({"code": "zero_normal_tautologies", "indices": redundant})
    # Stage 1: independent feasibility. Never consult vertices here.
    point = _feasible(hs)
    if point is None:
        result["status"] = result["algebraic_status"] = "empty"
        return result
    result["feasible_point_exact"] = _exact_point(point)
    try:
        result["feasible_point"] = _float_point(point)
        # Stage 2: recession cone, using four independent feasibility queries.
        direction, checks = _recession(hs)
        result["recession_checks"] = checks
        if direction is not None:
            result["status"] = result["algebraic_status"] = "unbounded"
            result["recession_direction"] = _float_point(direction)
            result["recession_direction_exact"] = _exact_point(direction)
            return result
        # Stage 3: only now enumerate boundary intersections.
        result["algebraic_status"] = "bounded"
        vertices, ill = _vertices(hs)
        if not vertices:
            result["status"] = "numerically_uncertain"
            result["diagnostics"].append({"code": "bounded_but_no_vertices"})
            return result
        result["vertices"] = [_float_point(p) for p in vertices]
        kind = "point" if len(vertices) == 1 else "segment" if len(vertices) == 2 else "bounded"
        result["algebraic_status"] = kind
        if ill:
            result["status"] = "numerically_uncertain"
            result["diagnostics"].append({"code": "ill_conditioned_active_intersection", "pairs": ill})
            return result
        # Output precision is checked separately; physical constraints stay exact.
        for q, p in zip(result["vertices"], vertices):
            for h in hs:
                norm = math.hypot(float(h[0]), float(h[1]))
                if not norm:
                    continue
                distance = float((_dot(h, (F(q[0]), F(q[1]))) - h[2])) / norm
                tolerance = ABS_TOL_M + REL_TOL * max(1.0, abs(q[0]), abs(q[1]))
                if distance > tolerance:
                    raise ArithmeticError("rounded vertex violates output residual tolerance")
        result.update(_circle_decision(vertices))
        if not result["circle_crosscheck_passed"]:
            raise ArithmeticError("independent circle checks disagree")
        if result["marginal_circle_vertex_indices"]:
            raise ArithmeticError("nonzero circle containment margin below output tolerance")
        if not all(math.isfinite(result[k]) for k in ("diameter_m",)):
            raise OverflowError("diameter output overflow")
        result["status"] = kind
    except (OverflowError, ArithmeticError) as exc:
        result["status"] = "numerically_uncertain"
        for key in ("diameter_m", "farthest_pair", "farthest_pair_indices", "diameter_circle",
                    "minimum_enclosing_circle", "circle_crosscheck_passed"):
            result[key] = None
        result["diagnostics"].append({"code": "numeric_output_failure", "message": str(exc)})
    return result


def _unit(deg):
    angle = deg % 360.0
    # Only exact cardinal angles are snapped. No angular epsilon is introduced.
    cardinals = {0.0: (1.0, 0.0), 90.0: (0.0, 1.0),
                 180.0: (-1.0, 0.0), 270.0: (0.0, -1.0)}
    if angle in cardinals:
        return cardinals[angle]
    # Evaluate complementary/quadrant angles from the same base. In particular,
    # perpendicular rays stay exactly perpendicular in represented arithmetic.
    quadrant = int(angle // 90)
    local = angle - 90*quadrant
    complement = local > 45
    base = 90-local if complement else local
    c, s = math.cos(math.radians(base)), math.sin(math.radians(base))
    if complement:
        c, s = s, c
    return ((c,s),(-s,c),(-c,-s),(s,-c))[quadrant]


def observations_to_halfplanes(observations, error_deg=1.0):
    alpha = _number(error_deg, "error_deg")
    if not 0 < alpha < 90:
        raise ValueError("error_deg must lie strictly between 0 and 90 degrees")
    if not isinstance(observations, (list, tuple)) or not observations:
        raise ValueError("observations must be a nonempty list for one source")
    planes, unique, seen, duplicates = [], [], set(), []
    for i, record in enumerate(observations):
        if not isinstance(record, dict) or set(record) != {"x", "y", "svd_deg"}:
            raise ValueError(f"observation {i}: require exactly x, y, svd_deg (direction readings only)")
        x, y, theta = (_number(record[k], f"observation {i}.{k}") for k in ("x", "y", "svd_deg"))
        if not 0 <= theta < 360:
            raise ValueError(f"observation {i}: svd_deg must be in [0,360)")
        key = (x, y, theta)
        if key in seen:
            duplicates.append(i)
            continue
        seen.add(key)
        unique.append({"x": x, "y": y, "svd_deg": theta})
        lower, upper = _unit(theta-alpha), _unit(theta+alpha)
        for a, b in ((lower[1], -lower[0]), (-upper[1], upper[0])):
            c = a*x+b*y
            if not math.isfinite(c):
                raise ValueError("coordinate/normal product overflow")
            planes.append(HalfPlane(a, b, c))
    return planes, unique, duplicates


def solve_b1(observations, error_deg=1.0):
    """Solve one source's B1 problem; input bearings are degrees CCW from east."""
    planes, unique, duplicates = observations_to_halfplanes(observations, error_deg)
    result = solve_halfplanes(planes)
    result.update({"schema_version": 1, "region_definition": "intersection_of_closed_bearing_wedges",
                   "units": {"coordinates": "m", "bearings": "degree_ccw_from_east"},
                   "error_deg": float(error_deg), "observations": unique,
                   "input_count": len(observations), "unique_count": len(unique),
                   "duplicate_indices": duplicates})
    return result
