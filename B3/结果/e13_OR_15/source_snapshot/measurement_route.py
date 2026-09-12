"""Choose source-local measurements by route detour plus forecast recovery cost.

Posterior sampling is a scoring heuristic only. Actual clear guarantees always
come from the conservative B1+ polygon after a real observation.
"""
import math


def choose(geometry, current, structural):
    from e12_coverage_hook import E11
    d = E11.e5.e1.d
    center, radius = geometry['mec']
    if radius > 500:
        return None
    a, b = geometry['axis_ends']; length = math.dist(a, b)
    if length < 1e-6:
        return center
    side = (-(b[1]-a[1])/length, (b[0]-a[0])/length)
    candidates = [(center[0]+sign*offset*side[0], center[1]+sign*offset*side[1])
                  for offset in (20, 40, 80, 120) for sign in (-1, 1)]
    route = E11.e5.fast_open_route(current, list(structural)+[('future_owner', -1, center)])
    route_points = [current]+[n[2] for n in route]
    for p, q in zip(route_points, route_points[1:]):
        for t in (.25, .5, .75):
            x = (p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1]))
            if math.dist(x, center) <= 500:
                candidates.append(x)
    scored = []
    for point in candidates:
        if max(math.dist(point, v) for v in geometry['polygon']) > 1000-1e-6:
            continue
        middle = math.atan2(center[1]-point[1], center[0]-point[0])
        angles = [math.degrees((math.atan2(v[1]-point[1], v[0]-point[0])-middle+math.pi)%(2*math.pi)-math.pi)
                  for v in geometry['polygon']]
        low, high = min(angles)-1, max(angles)+1
        if high-low > 170:
            continue
        worst = 0.
        for i in range(9):
            theta = (math.degrees(middle)+low+(high-low)*i/8)%360
            polygon = list(geometry['polygon'])
            for h in d.wedge_halfplanes({'x': point[0], 'y': point[1], 'svd_deg': theta}):
                polygon = d.clip(polygon, h)
            if polygon:
                circle = d.mec(polygon)
                worst = max(worst, circle[1])
        detour = min([math.dist(route_points[-1], point)]+[
            math.dist(p, point)+math.dist(point, q)-math.dist(p, q) for p, q in zip(route_points, route_points[1:])])
        recovery_steps = math.ceil(max(0, worst-17.5)/35)
        score = detour/5+6+35*recovery_steps/5+3*recovery_steps+5
        scored.append((score, worst, math.dist(current, point), point))
    return min(scored)[-1] if scored else None
