"""Choose one certified localization location per source jointly with route order."""
import math


def choose_points(start, fixed, alternatives):
    from e12_coverage_hook import E11
    nodes = list(fixed) + [(kind, key, points[0]) for (kind, key), points in alternatives.items()]
    best = None
    for _ in range(6):
        route = E11.e5.fast_open_route(start, nodes)
        cost = E11.e5.route_cost(start, route)
        if best is None or cost < best[0]:
            best = (cost, list(route))
        updated = []
        for i, (kind, key, point) in enumerate(route):
            options = alternatives.get((kind, key))
            if options:
                before = start if i == 0 else route[i-1][2]
                after = route[i+1][2] if i+1 < len(route) else None
                point = min(options, key=lambda p: math.dist(before, p) + (math.dist(p, after) if after else 0.))
            updated.append((kind, key, point))
        if updated == route:
            break
        nodes = updated
    else:
        route = E11.e5.fast_open_route(start, nodes)
        cost = E11.e5.route_cost(start, route)
        if cost < best[0]:
            best = cost, route
    return best[1]


def joint_nodes(solver, remaining):
    fixed = [('cover', idx, p) for idx, p in remaining] + solver.eligible_targets()
    alternatives = {}
    for track in solver.broad_tracks():
        point = solver.route_fit_supp(track, fixed)
        if point is None:
            continue
        options = [point]
        g = track.get('geometry')
        if g and g['status'] == 'bounded':
            center = g['mec'][0]
            opposite = (2*center[0]-point[0], 2*center[1]-point[1])
            if (max(math.dist(opposite, v) for v in g['polygon']) <= 1000-1e-6
                and all(math.dist(opposite, (o['x'], o['y'])) > 1 for o in track['dirs'])):
                options.append(opposite)
        alternatives['supp', track['channel']] = options
    return choose_points(solver.env.pos, fixed, alternatives) if fixed or alternatives else []
