"""Forecast complete localization/clear chains, rather than isolated observation nodes."""
import math
from clear_risk import localization_point


def tasks(solver, remaining):
    nodes = [('cover', idx, p) for idx, p in remaining]
    for c, track in solver.tracks.items():
        if c in solver.env.cleared:
            continue
        g = track.get('geometry')
        if g and g['status'] == 'bounded':
            nodes.append(('source', c, g['mec'][0]))
    return nodes


def choose(solver, remaining, beam=False):
    from e12_coverage_hook import E11
    nodes = tasks(solver, remaining)
    if not nodes:
        return None
    if not beam:
        return E11.e5.fast_open_route(solver.env.pos, nodes)[0]
    # A macro's expected end is its feasible center; actual geometry is rechecked online.
    def edge(position, node):
        kind, key, center = node
        if kind == 'cover':
            return math.dist(position, center)/5 + 6*len([c for c in range(1, 21) if c not in solver.tracks])
        g = solver.tracks[key]['geometry']
        if g['mec'][1] <= 20:
            return math.dist(position, center)/5+5
        point = localization_point(g, position)
        if point is None:
            bank = solver.safe_bank(solver.tracks[key])
            point = min(bank, key=lambda p: math.dist(position, p)+math.dist(p, center)) if bank else center
        return (math.dist(position, point)+math.dist(point, center))/5+11+.1*g['mec'][1]/5

    def leaf(position, rest):
        if not rest:
            return 0.
        # A full open-path forecast, not just the spanning-tree lower bound.
        route = E11.e5.fast_open_route(position, rest)
        total = 0.
        for node in route:
            total += edge(position, node); position = node[2]
        return total

    frontier = [(0., 0., (), tuple(range(len(nodes))), solver.env.pos)]
    for depth in range(min(3, len(nodes))):
        children = []
        for _, accumulated, prefix, left, position in frontier:
            candidates = sorted(left, key=lambda i: (edge(position, nodes[i]), i))[:10]
            for i in candidates:
                node = nodes[i]; rest = tuple(j for j in left if j != i)
                cost = accumulated+edge(position, node)
                score = cost+leaf(node[2], [nodes[j] for j in rest])
                children.append((score, cost, prefix+(i,), rest, node[2]))
        frontier = sorted(children, key=lambda x: (x[0], x[2]))[:8]
    return nodes[min(frontier, key=lambda x: (x[0], x[2]))[2][0]]
