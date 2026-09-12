"""Observable-state scenario rollout for the next real action.

The uniform arena / 13-of-20 occupancy prior is a ranking heuristic, never a
localization or discovery certificate. No case, transport, seed or truth input.
Scenarios keep identical quadrature points across decisions to avoid jitter.
"""
from functools import lru_cache
import math
import random

SCENARIOS = 8
PRIOR = 13 / 20
GOLDEN = math.pi * (3 - math.sqrt(5))
BANK = tuple((1800 * math.sqrt((i + .5) / 1024) * math.cos(i * GOLDEN),
              1800 * math.sqrt((i + .5) / 1024) * math.sin(i * GOLDEN))
             for i in range(1024))


@lru_cache(maxsize=256)
def unseen_support(silent):
    # A source cannot be inside a guaranteed 1000 m reception disk at silence.
    return tuple(p for p in BANK if all(math.dist(p, q) > 1000 for q in silent))


def scenarios(tasks, tracks, cleared, silent, remaining):
    task_ids = {(kind, key): i for i, (kind, key, _) in enumerate(tasks)}
    result = []
    for sample in range(SCENARIOS):
        nodes = list(tasks)
        requirements = [set() for _ in nodes]
        for channel, track in sorted(tracks.items()):
            if channel in cleared or ('clear', channel) in task_ids:
                continue
            g = track.get('geometry')
            if not g or g['status'] != 'bounded':
                continue
            poly = g['polygon']
            center = tuple(sum(p[d] for p in poly) / len(poly) for d in (0, 1))
            extreme = poly[(sample * len(poly)) // SCENARIOS]
            point = tuple((center[d] + extreme[d]) / 2 for d in (0, 1))
            nodes.append(('future_clear', channel, point))
            dependency = task_ids.get(('supp', channel))
            requirements.append(set() if dependency is None else {dependency})
        potential = []
        if remaining and len(tracks) < 16:
            for channel in range(1, 21):
                if channel in tracks:
                    continue
                support = unseen_support(tuple(sorted(tuple(p) for p in silent[channel])))
                fraction = len(support) / len(BANK)
                probability = PRIOR * fraction / (1 - PRIOR + PRIOR * fraction)
                rng = random.Random(57000000 + sample * 100 + channel)
                draw = rng.random()
                if not support or draw >= probability:
                    continue
                point = support[rng.randrange(len(support))]
                # A potential source is available only after a covering observation.
                deps = {task_ids['cover', idx] for idx, p in remaining
                        if ('cover', idx) in task_ids and math.dist(point, p) <= 1000}
                if deps:
                    potential.append((draw / probability, channel, point, deps))
            for _, channel, point, deps in sorted(potential)[:16-len(tracks)]:
                nodes.append(('future_discovery', channel, point))
                requirements.append(deps)
        result.append((nodes, requirements))
    return result


def route_cost_with_first(start, nodes, requirements, first):
    """A feasible precedence-respecting heuristic tail, not an optimal bound.

    Requirement sets use OR: one observing cover/supp node reveals that task.
    The fixed first node is real. Three alternate second nodes reduce NN bias;
    improving 2-opt moves are allowed only when all dependencies stay valid.
    """
    n = len(nodes)
    dist = [[math.dist(a[2], b[2]) for b in nodes] for a in nodes]
    entry = math.dist(start, nodes[first][2])

    def allowed(i, done):
        return not requirements[i] or bool(requirements[i] & done)

    def valid(route):
        done = set()
        for i in route:
            if not allowed(i, done):
                return False
            done.add(i)
        return True

    seconds = sorted((i for i in range(n) if i != first and allowed(i, {first})),
                     key=lambda i: (dist[first][i], i))[:3]
    best = math.inf
    for second in seconds or [None]:
        route = [first] if second is None else [first, second]
        done = set(route)
        while len(route) < n:
            choices = [i for i in range(n) if i not in done and allowed(i, done)]
            if not choices:
                raise ValueError('cyclic or unsatisfied scenario dependency')
            nxt = min(choices, key=lambda i: (dist[route[-1]][i], i))
            route.append(nxt); done.add(nxt)
        for _ in range(3):
            changed = False
            for lo in range(1, n - 1):
                for hi in range(lo + 1, n):
                    a, b, c = route[lo-1], route[lo], route[hi]
                    old, new = dist[a][b], dist[a][c]
                    if hi+1 < n:
                        d = route[hi+1]; old += dist[c][d]; new += dist[b][d]
                    if new < old - 1e-7:
                        candidate = route[:lo] + route[lo:hi+1][::-1] + route[hi+1:]
                        if valid(candidate):
                            route = candidate; changed = True
            if not changed:
                break
        best = min(best, entry + sum(dist[a][b] for a, b in zip(route, route[1:])))
    return best


def choose(position, tasks, tracks, cleared, silent, remaining):
    # Keep current baseline first action in the candidate set as an explicit control.
    from e12_coverage_hook import E11
    first = E11.e5.fast_open_route(position, tasks)[0]
    candidates = [tasks.index(first)]
    for kind, count in (('cover', 3), ('clear', 3), ('supp', 3)):
        nearby = sorted((i for i, n in enumerate(tasks) if n[0] == kind),
                        key=lambda i: (math.dist(position, tasks[i][2]), i))[:count]
        candidates.extend(i for i in nearby if i not in candidates)
    worlds = scenarios(tasks, tracks, cleared, silent, remaining)
    scores = []
    for i in candidates:
        lengths = [route_cost_with_first(position, nodes, deps, i) for nodes, deps in worlds]
        scores.append((sum(lengths) / len(lengths), i))
    score, index = min(scores)
    return tasks[index], {'expected_route_m': score, 'scenario_count': len(worlds),
                          'future_counts': [len(nodes)-len(tasks) for nodes, _ in worlds],
                          'candidate_scores': [[tasks[i][:2], value] for value, i in scores],
                          'baseline_first': first[:2]}
