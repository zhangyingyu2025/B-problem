"""Width-8, depth-3 rolling horizon with observable future-task dependencies."""
import math
from future_route import future_nodes, action_candidates


def choose(solver, tasks, remaining, width=8, depth=3):
    actual, first_candidates = action_candidates(solver, tasks, remaining)
    future, dependencies, risks = future_nodes(solver, remaining, tasks)
    nodes = actual+future
    by_key = {(n[0], n[1]): i for i, n in enumerate(nodes)}
    initial = [by_key[(n[0], n[1])] for n in first_candidates]
    points = [n[2] for n in nodes]
    distances = [[math.dist(p, q) for q in points] for p in points]
    start_dist = [math.dist(solver.env.pos, p) for p in points]
    costs = []
    for kind, key, p in nodes:
        if kind == 'cover': cost = 6*len([c for c in range(1, 21) if c not in solver.tracks])
        elif kind in ('supp', 'observe'): cost = 6
        else: cost = 5
        costs.append(cost+risks.get((kind, key), 0)/5)

    def reduced(left, chosen):
        kind, key, _ = nodes[chosen]
        return frozenset(i for i in left if i != chosen and not
                         (kind in ('supp', 'observe') and nodes[i][0] in ('supp', 'observe') and nodes[i][1] == key))

    mst_cache = {}
    def leaf(left, endpoint):
        if not left: return 0.
        if left not in mst_cache:
            todo = set(left); root = min(todo); todo.remove(root)
            near = {i: distances[root][i] for i in todo}; cost = 0.
            while todo:
                node = min(todo, key=lambda i: (near[i], i)); cost += near[node]; todo.remove(node)
                for i in todo: near[i] = min(near[i], distances[node][i])
            mst_cache[left] = cost
        entry = min(distances[endpoint][i] for i in left)
        return (entry+mst_cache[left])/5 + sum(costs[i] for i in left)

    def available(left, prefix):
        done = {(nodes[i][0], nodes[i][1]) for i in prefix}
        result = []
        for i in left:
            key = (nodes[i][0], nodes[i][1])
            if key not in dependencies or any(dep in done for dep in dependencies[key]):
                result.append(i)
        return result

    all_ids = frozenset(range(len(nodes)))
    beam = []
    for i in initial:
        left = reduced(all_ids, i); cost = start_dist[i]/5+costs[i]
        beam.append((cost+leaf(left, i), cost, (i,), left))
    beam.sort(key=lambda x: (x[0], x[2])); beam = beam[:width]
    for _ in range(depth-1):
        children = []
        for _, cost, prefix, left in beam:
            choices = sorted(available(left, prefix), key=lambda i: (distances[prefix[-1]][i], i))[:10]
            if not choices:
                children.append((cost, cost, prefix, left))
            for i in choices:
                rest = reduced(left, i); next_cost = cost+distances[prefix[-1]][i]/5+costs[i]
                children.append((next_cost+leaf(rest, i), next_cost, prefix+(i,), rest))
        beam = sorted(children, key=lambda x: (x[0], x[2]))[:width]
    best = min(beam, key=lambda x: (x[0], x[2]))
    first = nodes[best[2][0]]
    assert first[0] in ('cover', 'clear', 'supp', 'observe')
    return first, {'leaf_score_s': best[0], 'prefix': [nodes[i][:2] for i in best[2]],
                   'actual_tasks': len(actual), 'future_tasks': len(future)}
