"""Preserve coverage sweep direction while inserting live source tasks."""
import math


def insert_tasks(start, skeleton, tasks):
    route = list(skeleton)
    # Farthest insertion first reduces unstable nearest-task ordering.
    for task in sorted(tasks, key=lambda t: (-math.dist(start, t[2]), t[1])):
        choices = []
        for i in range(len(route)+1):
            before = start if i == 0 else route[i-1][2]
            if i == len(route): delta = math.dist(before, task[2])
            else: delta = math.dist(before, task[2])+math.dist(task[2], route[i][2])-math.dist(before, route[i][2])
            choices.append((delta, i))
        _, index = min(choices)
        route.insert(index, task)
    return route


def choose(solver, remaining, macro=True):
    from e12_coverage_hook import E11
    if macro:
        from macro_route import tasks
        others = [n for n in tasks(solver, []) if n[0] != 'cover']
    else:
        others = [n for n in solver.joint_nodes(remaining) if n[0] != 'cover']
    if not hasattr(solver, 'sweep_order'):
        best = None
        ordered = sorted(remaining)
        if ordered:
            for start in range(len(ordered)):
                for direction in (-1, 1):
                    covers = [ordered[(start+direction*j)%len(ordered)] for j in range(len(ordered))]
                    anchors = [('cover', idx, p) for idx, p in covers]
                    route = insert_tasks(solver.env.pos, anchors, others)
                    cost = E11.e5.route_cost(solver.env.pos, route)
                    if best is None or cost < best[0]: best = (cost, [idx for idx, p in covers])
            solver.sweep_order = best[1]
        else: solver.sweep_order = []
    positions = dict(remaining)
    skeleton = [('cover', idx, positions[idx]) for idx in solver.sweep_order if idx in positions]
    route = insert_tasks(solver.env.pos, skeleton, others)
    return route[0] if route else None
