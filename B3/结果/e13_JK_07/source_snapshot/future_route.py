"""Observable future-task surrogates, not hidden source locations."""
import math


def future_nodes(solver, remaining, tasks):
    nodes = []; dependencies = {}; risks = {}
    for c, track in solver.tracks.items():
        if c in solver.env.cleared or solver.localized(track):
            continue
        g = track.get('geometry')
        if not g or g['status'] != 'bounded':
            continue
        # Center plus uncertainty cost; hull centroid is also observable.
        center = g['centers'][1]
        key = ('future_source', c)
        nodes.append((key[0], key[1], center))
        dependencies[key] = [('supp', c), ('observe', c)]
        risks[key] = .15*g['diameter']
    if len(solver.tracks) < 16:
        # One representative per unvisited coverage sector, never per unknown channel.
        for idx, p in remaining:
            length = math.hypot(*p)
            if not length:
                continue
            q = (p[0]*1500/length, p[1]*1500/length)
            key = ('future_discovery', idx)
            nodes.append((key[0], key[1], q))
            dependencies[key] = [('cover', idx)]
            risks[key] = 0.
    return nodes, dependencies, risks


def action_candidates(solver, tasks, remaining):
    nodes = list(tasks)
    if remaining:
        target = min(remaining, key=lambda x: math.dist(solver.env.pos, x[1]))[1]
        for t in solver.broad_tracks():
            choices = solver.segment_points(t, solver.env.pos, target, 1)
            if choices:
                _, p, _ = choices[0]
                nodes.append(('observe', t['channel'], p))
    # Keep diversity across coverage, guaranteed-clear, and localization actions.
    selected = []
    for kinds, limit in ((('cover',), 3), (('clear',), 3), (('supp', 'observe'), 4)):
        pool = [n for n in nodes if n[0] in kinds]
        selected.extend(sorted(pool, key=lambda n: (math.dist(solver.env.pos, n[2]), n[0], n[1]))[:limit])
    return nodes, selected
