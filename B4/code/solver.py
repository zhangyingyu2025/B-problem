"""Final consolidated B4 solver.

This file is the single current implementation, not a historical historical numbered solver versions chain.
Private classes are semantic implementation layers kept only to preserve readable
separation of discovery, routing, safe sensing, localization and clearing logic.
Use B4Solver as the public solver class.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from .b4_geometry import geometry, lower_distance, fallback_cover
from .discovery_mesh import mesh25
from .discovery_certificate import verify_mesh, absent_certified
from .discovery_convex_certificate import verify_local_convex_certificate
from .paired_fan import paired_fan

class InvariantFailure(RuntimeError):
    pass

class RobotPort:
    """Observation/action facade; no source-case metadata in the public API."""

    def __init__(self, client):
        self.__client = client

    @property
    def position(self):
        return self.__client.position

    @property
    def channel(self):
        return self.__client.channel

    @property
    def virtual_time_s(self):
        return self.__client.virtual_time_s

    def measure(self, p, c, reason):
        return self.__client.measure(p, c, reason)

    def clear(self, p, c, reason):
        return self.__client.clear(p, c, reason)

    def record(self, event):
        return self.__client.record(event)

@dataclass
class SourceTrackB4:
    channel: int
    dirs: list = field(default_factory=list)
    signals: list = field(default_factory=list)
    ambiguous_no_signal: list = field(default_factory=list)
    proven_backside: list = field(default_factory=list)
    clear_fail_points: list = field(default_factory=list)
    near: object = None
    geometry: object = None
    mec: object = None
    type_state: str = 'unknown'
    last_signal_point: object = None
    last_signal_bearing: object = None
    fan_steps: int = 0

class _CoreDiscoverySolver:

    def __init__(self, port, mesh=None, fan_limit=8):
        self.port = port
        self.mesh = mesh or mesh25()
        if self.mesh.get('certificate_kind') == 'local_convex':
            self.certificate = verify_local_convex_certificate(self.mesh)
        else:
            self.certificate = verify_mesh(self.mesh)
        self.fan_limit = fan_limit
        self.tracks = {}
        self.cleared = set()
        self.absent = set()
        self.checked = {c: set() for c in range(1, 21)}
        self.stats = {'certificate_sites_visited': 0, 'fan_steps': 0, 'fan_first_success': 0, 'fan_second_required': 0, 'fan_invariant_failures': 0, 'guaranteed_clear_count': 0, 'guaranteed_clear_failures': 0, 'fallback_clear_cover_count': 0, 'fallback_triggers': 0}
        self.detection_times = {}

    def discovery_complete(self):
        return len(self.tracks) == 16 or all((c in self.tracks or c in self.absent for c in range(1, 21)))

    def observe(self, p, c, reason, site=None, backside=False):
        body = self.port.measure(p, c, reason)
        kind = body['measure_result']
        if c in self.cleared:
            return body
        if kind == 'no_signal':
            if c in self.tracks:
                t = self.tracks[c]
                t.ambiguous_no_signal.append(tuple(p))
                if backside:
                    t.proven_backside.append(tuple(p))
                    t.type_state = 'directional'
            elif site is not None:
                self.checked[c].add(site)
                if self.mesh.get('certificate_kind') == 'local_convex':
                    if len(self.checked[c]) == len(self.mesh['vertices']):
                        self.absent.add(c)
                elif absent_certified(self.checked[c], self.mesh):
                    self.absent.add(c)
            return body
        if c not in self.tracks:
            self.tracks[c] = SourceTrackB4(c)
            self.detection_times[c] = self.port.virtual_time_s
        t = self.tracks[c]
        t.last_signal_point = tuple(p)
        if tuple(p) not in t.signals:
            t.signals.append(tuple(p))
        if kind == 'near':
            t.near = tuple(p)
        else:
            t.last_signal_bearing = body['svd_deg']
            obs = {'x': p[0], 'y': p[1], 'svd_deg': body['svd_deg']}
            if obs not in t.dirs:
                t.dirs.append(obs)
            g = geometry(t.dirs, t.signals)
            if g['status'] != 'bounded':
                raise InvariantFailure('B4 outer geometry numerically uncertain; preserve trace')
            t.geometry = g
            t.mec = g['mec']
        return body

    def clear(self, p, c, guaranteed=False, reason='B4_clear'):
        body = self.port.clear(p, c, reason)
        success = body['clear_result'] == 'success'
        if guaranteed:
            self.stats['guaranteed_clear_count'] += 1
            if not success:
                self.stats['guaranteed_clear_failures'] += 1
                raise InvariantFailure('guaranteed B4 clear failed')
        if success:
            self.cleared.add(c)
        else:
            self.tracks[c].clear_fail_points.append(tuple(p))
        return success

    def resolve(self, c):
        t = self.tracks[c]
        while c not in self.cleared:
            if t.near is not None:
                self.clear(t.near, c, True, 'B4_near_clear')
                return
            if t.mec and t.mec[1] <= 20 - 1e-06:
                self.clear(t.mec[0], c, True, 'B4_guaranteed_clear')
                return
            if t.fan_steps >= self.fan_limit:
                self.fallback(c)
                return
            anchor = t.last_signal_point
            lb = lower_distance(anchor, t.geometry['polygon'])
            if lb <= 20:
                if anchor not in t.clear_fail_points and self.clear(anchor, c, False, 'B4_anchor_clear'):
                    return
                lb = max(lb, 20.0)
            pair = paired_fan(anchor, t.last_signal_bearing, lb)
            pair = sorted(pair, key=lambda p: math.dist(self.port.position, p))
            t.fan_steps += 1
            self.stats['fan_steps'] += 1
            self.port.record({'kind': 'B4_fan', 'channel': c, 'anchor': anchor, 'lower_bound_m': lb, 'points': pair})
            first = self.observe(pair[0], c, 'B4_fan_first', backside=True)
            if first['measure_result'] == 'no_signal':
                self.stats['fan_second_required'] += 1
                second = self.observe(pair[1], c, 'B4_fan_second', backside=True)
                if second['measure_result'] == 'no_signal':
                    self.stats['fan_invariant_failures'] += 1
                    raise InvariantFailure('both SPF45 branches returned no_signal')
            else:
                self.stats['fan_first_success'] += 1

    def fallback(self, c):
        t = self.tracks[c]
        self.stats['fallback_triggers'] += 1
        todo = fallback_cover(t.dirs[0], t.geometry['polygon'] if t.geometry else None)
        for _ in range(len(todo)):
            p = min(todo, key=lambda p: math.dist(self.port.position, p))
            todo.remove(p)
            self.stats['fallback_clear_cover_count'] += 1
            if self.clear(p, c, False, 'B4_finite_fallback'):
                return
        raise InvariantFailure('finite coverage exhausted without clearing discovered source')

    def run_all(self):
        remaining = list(range(len(self.mesh['vertices'])))
        while remaining and (not self.discovery_complete()):
            site = min(remaining, key=lambda i: math.dist(self.port.position, self.mesh['vertices'][i]))
            remaining.remove(site)
            p = self.mesh['vertices'][site]
            unknown = [c for c in range(1, 21) if c not in self.tracks and c not in self.absent]
            unknown.sort(key=lambda c: (c != self.port.channel, c))
            self.stats['certificate_sites_visited'] += 1
            for c in unknown:
                if len(self.tracks) == 16:
                    break
                self.observe(p, c, 'B4_certificate', site=site)
            while any((c not in self.cleared for c in self.tracks)):
                c = min((c for c in self.tracks if c not in self.cleared), key=lambda c: math.dist(self.port.position, self.tracks[c].mec[0] if self.tracks[c].mec else self.tracks[c].near))
                self.resolve(c)
        if not self.discovery_complete() or self.cleared != set(self.tracks):
            raise InvariantFailure('incomplete discovery or clearance')
from .b4_geometry import lower_distance

def _route_cost(start, nodes):
    p = start
    z = 0.0
    for _, _, q in nodes:
        z += math.dist(p, q)
        p = q
    return z

def fast_open_route(start, nodes):
    nodes = list(nodes)
    if len(nodes) <= 1:
        return nodes
    best = None
    starts = sorted(range(len(nodes)), key=lambda i: math.dist(start, nodes[i][2]))[:min(5, len(nodes))]
    for first in starts:
        rem = set(range(len(nodes)))
        rem.remove(first)
        order = [first]
        p = nodes[first][2]
        while rem:
            j = min(rem, key=lambda i: math.dist(p, nodes[i][2]))
            rem.remove(j)
            order.append(j)
            p = nodes[j][2]
        improved = True
        passes = 0
        while improved and passes < 6:
            improved = False
            passes += 1
            for i in range(len(order) - 1):
                a = start if i == 0 else nodes[order[i - 1]][2]
                bb = nodes[order[i]][2]
                for j in range(i + 1, len(order)):
                    c = nodes[order[j]][2]
                    dpt = nodes[order[j + 1]][2] if j + 1 < len(order) else None
                    old = math.dist(a, bb) + (math.dist(c, dpt) if dpt else 0)
                    new = math.dist(a, c) + (math.dist(bb, dpt) if dpt else 0)
                    if new + 1e-09 < old:
                        order[i:j + 1] = reversed(order[i:j + 1])
                        improved = True
        seq = [nodes[i] for i in order]
        c = _route_cost(start, seq)
        if best is None or c < best[0]:
            best = (c, seq)
    return best[1]

class _RouteSolver(_CoreDiscoverySolver):

    def source_task(self, c):
        t = self.tracks[c]
        if t.near is not None:
            return ('near', c, t.near)
        if t.mec and t.mec[1] <= 20 - 1e-06:
            return ('guaranteed', c, t.mec[0])
        if t.fan_steps >= self.fan_limit:
            return ('fallback', c, t.mec[0])
        anchor = t.last_signal_point
        lb = lower_distance(anchor, t.geometry['polygon'])
        if lb <= 20 and anchor not in t.clear_fail_points:
            return ('anchor', c, anchor)
        pair = paired_fan(anchor, t.last_signal_bearing, max(lb, 20.0))
        return ('fan', c, min(pair, key=lambda p: math.dist(self.port.position, p)))

    def scan_certificate(self, site):
        p = self.mesh['vertices'][site]
        unknown = [c for c in range(1, 21) if c not in self.tracks and c not in self.absent]
        unknown.sort(key=lambda c: (c != self.port.channel, c))
        self.stats['certificate_sites_visited'] += 1
        for c in unknown:
            if len(self.tracks) == 16:
                break
            self.observe(p, c, 'B4_certificate', site=site)

    def execute_source(self, task):
        kind, c, p = task
        t = self.tracks[c]
        if kind in ('near', 'guaranteed', 'anchor'):
            reason = {'near': 'B4_near_clear', 'guaranteed': 'B4_guaranteed_clear', 'anchor': 'B4_anchor_clear'}[kind]
            self.clear(p, c, kind != 'anchor', reason)
            return
        if kind == 'fallback':
            self.fallback(c)
            return
        if kind != 'fan':
            raise InvariantFailure('unknown source task')
        anchor = t.last_signal_point
        lb = max(20.0, lower_distance(anchor, t.geometry['polygon']))
        pair = sorted(paired_fan(anchor, t.last_signal_bearing, lb), key=lambda q: math.dist(self.port.position, q))
        if tuple(p) != tuple(pair[0]):
            raise InvariantFailure('stale fan task')
        t.fan_steps += 1
        self.stats['fan_steps'] += 1
        self.port.record({'kind': 'B4_fan', 'channel': c, 'anchor': anchor, 'lower_bound_m': lb, 'points': pair})
        first = self.observe(pair[0], c, 'B4_fan_first', backside=True)
        if first['measure_result'] == 'no_signal':
            self.stats['fan_second_required'] += 1
            second = self.observe(pair[1], c, 'B4_fan_second', backside=True)
            if second['measure_result'] == 'no_signal':
                self.stats['fan_invariant_failures'] += 1
                raise InvariantFailure('both SPF45 branches returned no_signal')
        else:
            self.stats['fan_first_success'] += 1

    def run_all(self):
        remaining = list(range(len(self.mesh['vertices'])))
        for _ in range(1000):
            nodes = [] if self.discovery_complete() else [('cover', i, self.mesh['vertices'][i]) for i in remaining]
            nodes += [self.source_task(c) for c in self.tracks if c not in self.cleared]
            if not nodes:
                break
            task = self.select_task(nodes)
            self.port.record({'kind': 'B4_schedule', 'task': task, 'active_tasks': len(nodes)})
            if task[0] == 'cover':
                self.scan_certificate(task[1])
                remaining.remove(task[1])
            else:
                self.execute_source(task)
        else:
            raise InvariantFailure('finite decision guard')
        if not self.discovery_complete() or self.cleared != set(self.tracks):
            raise InvariantFailure('incomplete discovery or clearance')

    def select_task(self, nodes):
        return fast_open_route(self.port.position, nodes)[0]

def line_angle_deg(a, b):
    d = abs((a - b + 180.0) % 360.0 - 180.0)
    return min(d, 180.0 - d)

class _OpportunisticSolver(_RouteSolver):

    def __init__(self, *args, opportunity_per_site=4, max_proxy_distance=1250.0, min_line_angle=14.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.opportunity_per_site = opportunity_per_site
        self.max_proxy_distance = max_proxy_distance
        self.min_line_angle = min_line_angle
        self.stats.update({'opportunity_measurements': 0, 'opportunity_directions': 0, 'opportunity_no_signal': 0, 'opportunity_near': 0, 'fan_deferred_cycles': 0})

    def _proxy(self, t):
        if t.mec:
            return t.mec[0]
        if t.geometry and t.geometry.get('centers'):
            return t.geometry['centers'][0]
        return None

    def _already_sampled(self, t, p):
        for q in t.signals + t.ambiguous_no_signal:
            if math.dist(q, p) < 1e-07:
                return True
        return False

    def opportunity_score(self, c, p):
        t = self.tracks[c]
        if c in self.cleared or t.near is not None or (not t.dirs) or self._already_sampled(t, p):
            return None
        proxy = self._proxy(t)
        if proxy is None:
            return None
        dist = math.dist(p, proxy)
        if dist > self.max_proxy_distance:
            return None
        cand = math.degrees(math.atan2(proxy[1] - p[1], proxy[0] - p[0]))
        separations = []
        for o in t.dirs:
            old = math.degrees(math.atan2(proxy[1] - o['y'], proxy[0] - o['x']))
            separations.append(line_angle_deg(cand, old))
        sep = max(separations) if separations else 0.0
        if sep < self.min_line_angle:
            return None
        radius = t.mec[1] if t.mec else 1000.0
        range_bonus = max(0.0, (self.max_proxy_distance - dist) / self.max_proxy_distance)
        return sep + 0.018 * min(radius, 1000.0) + 8.0 * range_bonus

    def has_future_opportunity(self, c, remaining):
        for site in remaining:
            if self.opportunity_score(c, self.mesh['vertices'][site]) is not None:
                return True
        return False

    def scan_certificate(self, site):
        super().scan_certificate(site)
        p = self.mesh['vertices'][site]
        candidates = []
        for c, t in self.tracks.items():
            if c in self.cleared:
                continue
            score = self.opportunity_score(c, p)
            if score is not None:
                candidates.append((score, c))
        candidates.sort(reverse=True)
        for _, c in candidates[:self.opportunity_per_site]:
            body = self.observe(p, c, 'B4_certificate_opportunity')
            self.stats['opportunity_measurements'] += 1
            kind = body['measure_result']
            if kind == 'direction':
                self.stats['opportunity_directions'] += 1
            elif kind == 'near':
                self.stats['opportunity_near'] += 1
            else:
                self.stats['opportunity_no_signal'] += 1

    def run_all(self):
        remaining = list(range(len(self.mesh['vertices'])))
        for _ in range(1400):
            nodes = [] if self.discovery_complete() else [('cover', i, self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:
                    continue
                task = self.source_task(c)
                if task[0] == 'fan' and remaining and (not self.discovery_complete()) and self.has_future_opportunity(c, remaining):
                    self.stats['fan_deferred_cycles'] += 1
                    continue
                nodes.append(task)
            if not nodes:
                break
            task = self.select_task(nodes)
            self.port.record({'kind': 'B4_schedule', 'task': task, 'active_tasks': len(nodes), 'policy': 'opportunistic_route'})
            if task[0] == 'cover':
                self.scan_certificate(task[1])
                remaining.remove(task[1])
            else:
                self.execute_source(task)
        else:
            raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared != set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')

def _path_cost(start, nodes, order):
    if not order:
        return 0.0
    z = math.dist(start, nodes[order[0]][2])
    for a, b in zip(order, order[1:]):
        z += math.dist(nodes[a][2], nodes[b][2])
    return z

def better_open_route(start, nodes):
    nodes = list(nodes)
    n = len(nodes)
    if n <= 1:
        return nodes
    candidates = []
    starts = sorted(range(n), key=lambda i: math.dist(start, nodes[i][2]))[:min(10, n)]
    for first in starts:
        rem = set(range(n))
        rem.remove(first)
        order = [first]
        p = nodes[first][2]
        while rem:
            j = min(rem, key=lambda i: math.dist(p, nodes[i][2]))
            rem.remove(j)
            order.append(j)
            p = nodes[j][2]
        candidates.append(order)
    first = min(range(n), key=lambda i: math.dist(start, nodes[i][2]))
    order = [first]
    rem = set(range(n))
    rem.remove(first)
    while rem:
        best = None
        for j in rem:
            for k in range(len(order) + 1):
                prev = start if k == 0 else nodes[order[k - 1]][2]
                if k == len(order):
                    delta = math.dist(prev, nodes[j][2])
                else:
                    nxt = nodes[order[k]][2]
                    delta = math.dist(prev, nodes[j][2]) + math.dist(nodes[j][2], nxt) - math.dist(prev, nxt)
                if best is None or delta < best[0]:
                    best = (delta, j, k)
        _, j, k = best
        order.insert(k, j)
        rem.remove(j)
    candidates.append(order)
    best_order = None
    best_cost = None
    for order in candidates:
        for _ in range(30):
            improved = False
            base = _path_cost(start, nodes, order)
            move = None
            mc = base
            for i in range(len(order) - 1):
                for j in range(i + 1, len(order)):
                    cand = order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:]
                    c = _path_cost(start, nodes, cand)
                    if c + 1e-08 < mc:
                        mc = c
                        move = cand
            if move is not None:
                order = move
                improved = True
                base = mc
            move = None
            mc = base
            for i in range(len(order)):
                x = order[i]
                rest = order[:i] + order[i + 1:]
                for k in range(len(rest) + 1):
                    cand = rest[:k] + [x] + rest[k:]
                    c = _path_cost(start, nodes, cand)
                    if c + 1e-08 < mc:
                        mc = c
                        move = cand
            if move is not None:
                order = move
                improved = True
            if not improved:
                break
        c = _path_cost(start, nodes, order)
        if best_cost is None or c < best_cost:
            best_cost = c
            best_order = order
    return [nodes[i] for i in best_order]

class _TransitOpportunitySolver(_OpportunisticSolver):
    transit_per_leg = 4
    transit_fracs = (0.2, 0.4, 0.6, 0.8)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, opportunity_per_site=6, max_proxy_distance=1000.0, min_line_angle=14.0, **kwargs)
        self.stats.update({'transit_opportunity_measurements': 0, 'transit_opportunity_directions': 0, 'transit_opportunity_no_signal': 0, 'transit_opportunity_near': 0})

    def transit_opportunities(self, destination, exclude_channel=None):
        start = self.port.position
        length = math.dist(start, destination)
        if length < 1e-07 or self.transit_per_leg <= 0:
            return
        options = []
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None or (not t.dirs):
                continue
            best = None
            for u in self.transit_fracs:
                p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
                score = self.opportunity_score(c, p)
                if score is not None and (best is None or score > best[0]):
                    best = (score, u, p, c)
            if best is not None:
                options.append(best)
        chosen = sorted(options, reverse=True)[:self.transit_per_leg]
        chosen.sort(key=lambda x: x[1])
        for _, u, p, c in chosen:
            if c in self.cleared or self.tracks[c].near is not None or self._already_sampled(self.tracks[c], p):
                continue
            body = self.observe(p, c, 'B4_transit_opportunity')
            self.stats['transit_opportunity_measurements'] += 1
            kind = body['measure_result']
            if kind == 'direction':
                self.stats['transit_opportunity_directions'] += 1
            elif kind == 'near':
                self.stats['transit_opportunity_near'] += 1
            else:
                self.stats['transit_opportunity_no_signal'] += 1

    def run_all(self):
        remaining = list(range(len(self.mesh['vertices'])))
        for _ in range(1600):
            nodes = [] if self.discovery_complete() else [('cover', i, self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:
                    continue
                task = self.source_task(c)
                if task[0] == 'fan' and remaining and (not self.discovery_complete()) and self.has_future_opportunity(c, remaining):
                    self.stats['fan_deferred_cycles'] += 1
                    continue
                nodes.append(task)
            if not nodes:
                break
            task = self.select_task(nodes)
            dest = task[2]
            exclude = None if task[0] == 'cover' else task[1]
            self.transit_opportunities(dest, exclude_channel=exclude)
            self.port.record({'kind': 'B4_schedule', 'task': task, 'active_tasks': len(nodes), 'policy': 'transit_opportunity'})
            if task[0] == 'cover':
                self.scan_certificate(task[1])
                remaining.remove(task[1])
            else:
                self.execute_source(task)
        else:
            raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared != set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')

class _ClearDeferralSolver(_TransitOpportunitySolver):
    min_future_saving_m = 200.0

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'guaranteed_clear_deferred_cycles': 0})

    def opportunity_score(self, c, p):
        t = self.tracks[c]
        if t.mec and t.mec[1] <= 20 - 1e-06:
            return None
        return super().opportunity_score(c, p)

    def _defer_clear(self, task, remaining):
        if task[0] != 'guaranteed' or not remaining or self.discovery_complete():
            return False
        q = task[2]
        now = math.dist(self.port.position, q)
        future = min((math.dist(self.mesh['vertices'][i], q) for i in remaining))
        return future + self.min_future_saving_m < now

    def run_all(self):
        remaining = list(range(len(self.mesh['vertices'])))
        for _ in range(1700):
            nodes = [] if self.discovery_complete() else [('cover', i, self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:
                    continue
                task = self.source_task(c)
                if task[0] == 'fan' and remaining and (not self.discovery_complete()) and self.has_future_opportunity(c, remaining):
                    self.stats['fan_deferred_cycles'] += 1
                    continue
                if self._defer_clear(task, remaining):
                    self.stats['guaranteed_clear_deferred_cycles'] += 1
                    continue
                nodes.append(task)
            if not nodes:
                break
            task = self.select_task(nodes)
            dest = task[2]
            exclude = None if task[0] == 'cover' else task[1]
            self.transit_opportunities(dest, exclude_channel=exclude)
            self.port.record({'kind': 'B4_schedule', 'task': task, 'active_tasks': len(nodes), 'policy': 'clear_defer'})
            if task[0] == 'cover':
                self.scan_certificate(task[1])
                remaining.remove(task[1])
            else:
                self.execute_source(task)
        else:
            raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared != set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')

class _ImmediateClearSolver(_ClearDeferralSolver):

    def _defer_clear(self, task, remaining):
        return False

class _SafeClearSolver(_ImmediateClearSolver):
    clear_margin = 2e-06

    def source_task(self, c):
        task = super().source_task(c)
        if task[0] != 'guaranteed':
            return task
        t = self.tracks[c]
        center = t.mec[0]
        r = t.mec[1]
        slack = max(0.0, 20.0 - r - self.clear_margin)
        here = self.port.position
        d = math.dist(here, center)
        if d <= slack:
            return ('guaranteed', c, here)
        if d > 0 and slack > 0:
            q = (center[0] + (here[0] - center[0]) * slack / d, center[1] + (here[1] - center[1]) * slack / d)
            return ('guaranteed', c, q)
        return task

def _circle_intersections(a, b, R):
    dx, dy = (b[0] - a[0], b[1] - a[1])
    d = math.hypot(dx, dy)
    if d < 1e-12 or d > 2 * R:
        return []
    x = d / 2.0
    h2 = R * R - x * x
    if h2 < -1e-10:
        return []
    h = math.sqrt(max(0.0, h2))
    ux, uy = (dx / d, dy / d)
    mx, my = (a[0] + x * ux, a[1] + x * uy)
    px, py = (-uy, ux)
    if h < 1e-12:
        return [(mx, my)]
    return [(mx + h * px, my + h * py), (mx - h * px, my - h * py)]

def nearest_clear_point(here, vertices, R=19.99998):
    if not vertices:
        return None

    def feasible(q):
        return all((math.dist(q, v) <= R + 1e-08 for v in vertices))
    if feasible(here):
        return here
    candidates = []
    for v in vertices:
        d = math.dist(v, here)
        if d > 1e-12:
            q = (v[0] + R * (here[0] - v[0]) / d, v[1] + R * (here[1] - v[1]) / d)
            if feasible(q):
                candidates.append(q)
    for i, a in enumerate(vertices):
        for b in vertices[i + 1:]:
            if math.dist(a, b) <= 2 * R + 1e-08:
                for q in _circle_intersections(a, b, R):
                    if feasible(q):
                        candidates.append(q)
    return min(candidates, key=lambda q: math.dist(here, q)) if candidates else None

class _FullRegionClearSolver(_SafeClearSolver):

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'full_clear_region_used': 0, 'full_clear_region_gain_m': 0.0})

    def source_task(self, c):
        task = super().source_task(c)
        if task[0] != 'guaranteed':
            return task
        t = self.tracks[c]
        vertices = t.geometry.get('polygon', []) if t.geometry else []
        q = nearest_clear_point(self.port.position, vertices)
        if q is None:
            return task
        old = math.dist(self.port.position, task[2])
        new = math.dist(self.port.position, q)
        self.stats['full_clear_region_used'] += 1
        self.stats['full_clear_region_gain_m'] += max(0.0, old - new)
        return ('guaranteed', c, q)
R_CLEAR = 19.99998

def segment_clear_interval(a, b, vertices, R=R_CLEAR):
    dx, dy = (b[0] - a[0], b[1] - a[1])
    A = dx * dx + dy * dy
    if A < 1e-14:
        return (0.0, 1.0) if all((math.dist(a, v) <= R for v in vertices)) else None
    lo, hi = (0.0, 1.0)
    for vx, vy in vertices:
        ox, oy = (a[0] - vx, a[1] - vy)
        B = 2 * (ox * dx + oy * dy)
        C = ox * ox + oy * oy - R * R
        disc = B * B - 4 * A * C
        if disc < 0:
            if C > 0:
                return None
            continue
        root = math.sqrt(max(0.0, disc))
        l = (-B - root) / (2 * A)
        h = (-B + root) / (2 * A)
        lo = max(lo, l)
        hi = min(hi, h)
        if lo > hi + 1e-12:
            return None
    if hi < 0 or lo > 1:
        return None
    return (max(0.0, lo), min(1.0, hi))

class _TransitRegionClearSolver(_FullRegionClearSolver):

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'transit_full_region_clears': 0})

    def transit_opportunities(self, destination, exclude_channel=None):
        start = self.port.position
        length = math.dist(start, destination)
        if length < 1e-07:
            return
        plan = []
        options = []
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None or (not t.dirs):
                continue
            best = None
            for u in self.transit_fracs:
                p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
                score = self.opportunity_score(c, p)
                if score is not None and (best is None or score > best[0]):
                    best = (score, u, p, c)
            if best is not None:
                options.append(best)
        for score, u, p, c in sorted(options, reverse=True)[:self.transit_per_leg]:
            plan.append((u, 1, 'measure', c, p))
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or (not t.mec) or (t.mec[1] > 20 - 1e-06) or (not t.geometry):
                continue
            verts = t.geometry.get('polygon', [])
            interval = segment_clear_interval(start, destination, verts)
            if interval is None:
                continue
            u = interval[0]
            p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
            plan.append((u, 0, 'clear', c, p))
        plan.sort(key=lambda x: (x[0], x[1]))
        for u, _, kind, c, p in plan:
            if c in self.cleared:
                continue
            if kind == 'clear':
                t = self.tracks[c]
                if not t.geometry or not all((math.dist(p, v) <= R_CLEAR + 1e-07 for v in t.geometry.get('polygon', []))):
                    continue
                self.clear(p, c, True, 'B4_transit_full_region_clear')
                self.stats['transit_full_region_clears'] += 1
            else:
                t = self.tracks[c]
                if t.near is not None or self._already_sampled(t, p):
                    continue
                body = self.observe(p, c, 'B4_transit_opportunity')
                self.stats['transit_opportunity_measurements'] += 1
                k = body['measure_result']
                if k == 'direction':
                    self.stats['transit_opportunity_directions'] += 1
                elif k == 'near':
                    self.stats['transit_opportunity_near'] += 1
                else:
                    self.stats['transit_opportunity_no_signal'] += 1
from .discovery_convex_certificate import mesh21

class _Convex21CertificateSolver(_TransitRegionClearSolver):

    def __init__(self, port, *a, **kw):
        if 'mesh' not in kw:
            kw['mesh'] = mesh21()
        super().__init__(port, *a, **kw)

class _Convex21ConfiguredSolver(_Convex21CertificateSolver):
    transit_per_leg = 6
    outer_radius = 1868.7
    inner_radius = 999.9

    def __init__(self, port, *a, **kw):
        if 'mesh' not in kw:
            kw['mesh'] = mesh21(self.outer_radius, self.inner_radius)
        super().__init__(port, *a, **kw)

class _FinalCertificateGeometrySolver(_Convex21ConfiguredSolver):
    outer_radius = 1868.0
    inner_radius = 997.5

class _FinalCertificateRoutingSolver(_FinalCertificateGeometrySolver):
    transit_per_leg = 8
ALPHA_DEG = 1.0

def paired_fan_beta(anchor, bearing_deg, lb, beta_deg, factor):
    if not (beta_deg > ALPHA_DEG and 0 < lb <= 1500):
        raise ValueError('invalid SPF-beta input')
    limit = min(math.cos(math.radians(ALPHA_DEG)) / math.cos(math.radians(beta_deg)), 2 * math.cos(math.radians(beta_deg + ALPHA_DEG)))
    if not factor < limit:
        raise ValueError(f'unsafe SPF-beta factor {factor} >= {limit}')
    step = factor * lb
    return tuple(((anchor[0] + step * math.cos(math.radians(bearing_deg + s * beta_deg)), anchor[1] + step * math.sin(math.radians(bearing_deg + s * beta_deg))) for s in (-1, 1)))

class _PairedFanSolver(_FinalCertificateRoutingSolver):
    beta_deg = 10.0
    fan_factor = 1.0
    transit_per_leg = 4

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.opportunity_per_site = 4
        self.max_proxy_distance = 1000.0
        self.min_line_angle = 10.0
        self.stats.update({'spf_beta_deg': self.beta_deg, 'spf_factor': self.fan_factor})

    def _fan_pair(self, t, lb):
        return paired_fan_beta(t.last_signal_point, t.last_signal_bearing, lb, self.beta_deg, self.fan_factor)

    def source_task(self, c):
        t = self.tracks[c]
        if t.near is not None:
            return ('near', c, t.near)
        if t.mec and t.mec[1] <= 20 - 1e-06:
            return super().source_task(c)
        if t.fan_steps >= self.fan_limit:
            return ('fallback', c, t.mec[0])
        anchor = t.last_signal_point
        lb = lower_distance(anchor, t.geometry['polygon'])
        if lb <= 20 and anchor not in t.clear_fail_points:
            return ('anchor', c, anchor)
        pair = self._fan_pair(t, max(lb, 20.0))
        return ('fan', c, min(pair, key=lambda p: math.dist(self.port.position, p)))

    def execute_source(self, task):
        kind, c, p = task
        t = self.tracks[c]
        if kind != 'fan':
            return super().execute_source(task)
        anchor = t.last_signal_point
        lb = max(20.0, lower_distance(anchor, t.geometry['polygon']))
        pair = sorted(self._fan_pair(t, lb), key=lambda q: math.dist(self.port.position, q))
        if math.dist(p, pair[0]) > 1e-06:
            raise InvariantFailure('stale SPF-beta task')
        t.fan_steps += 1
        self.stats['fan_steps'] += 1
        self.port.record({'kind': 'B4_fan', 'channel': c, 'anchor': anchor, 'lower_bound_m': lb, 'points': pair, 'beta_deg': self.beta_deg, 'factor': self.fan_factor})
        first = self.observe(pair[0], c, 'B4_fan_first', backside=True)
        if first['measure_result'] == 'no_signal':
            self.stats['fan_second_required'] += 1
            second = self.observe(pair[1], c, 'B4_fan_second', backside=True)
            if second['measure_result'] == 'no_signal':
                self.stats['fan_invariant_failures'] += 1
                raise InvariantFailure('both SPF-beta branches returned no_signal')
        else:
            self.stats['fan_first_success'] += 1
SAFE = {4.0: 1.0, 5.0: 1.0, 7.5: 1.0, 10.0: 1.0, 15.0: 1.02, 20.0: 1.04, 25.0: 1.08, 30.0: 1.13, 45.0: 1.3}

class _AdaptiveFanSolver(_PairedFanSolver):
    schedule = ((0.3, 30.0), (0.5, 15.0), (0.7, 7.5), (2.0, 4.0))

    def _choice(self, t, lb):
        proxy = self._proxy(t)
        est = math.dist(t.last_signal_point, proxy) if proxy is not None else lb
        ratio = min(1.0, lb / max(lb, est, 1e-09))
        for hi, beta in self.schedule:
            if ratio < hi:
                return (beta, SAFE[beta], ratio)
        beta = self.schedule[-1][1]
        return (beta, SAFE[beta], ratio)

    def _fan_pair(self, t, lb):
        b, k, _ = self._choice(t, lb)
        return paired_fan_beta(t.last_signal_point, t.last_signal_bearing, lb, b, k)

    def execute_source(self, task):
        if task[0] == 'fan':
            t = self.tracks[task[1]]
            from .b4_geometry import lower_distance
            lb = max(20.0, lower_distance(t.last_signal_point, t.geometry['polygon']))
            b, k, r = self._choice(t, lb)
            self.port.record({'kind': 'B4_adaptive_fan_choice', 'channel': task[1], 'beta_deg': b, 'factor': k, 'lb_proxy_ratio': r})
        return super().execute_source(task)

class _FinalFanPolicySolver(_AdaptiveFanSolver):
    schedule = ((0.35, 15.0), (0.65, 7.5), (2.0, 4.0))

class _CenterProbeSolver(_FinalFanPolicySolver):
    probe_radius = 250.0

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.center_probed = set()
        self.stats.update({'center_probes': 0, 'center_probe_signal': 0, 'center_probe_no_signal': 0, 'center_probe_near': 0})

    def source_task(self, c):
        task = super().source_task(c)
        if task[0] == 'fan' and c not in self.center_probed:
            t = self.tracks[c]
            if t.mec and 20.0 < t.mec[1] <= self.probe_radius and (not self._already_sampled(t, t.mec[0])):
                return ('probe', c, t.mec[0])
        return task

    def execute_source(self, task):
        if task[0] != 'probe':
            return super().execute_source(task)
        _, c, p = task
        t = self.tracks[c]
        if not t.mec or t.mec[1] > self.probe_radius + 1e-07:
            return super().execute_source(self.source_task(c))
        self.center_probed.add(c)
        self.stats['center_probes'] += 1
        body = self.observe(p, c, 'B4_center_probe', backside=True)
        k = body['measure_result']
        if k == 'direction':
            self.stats['center_probe_signal'] += 1
        elif k == 'near':
            self.stats['center_probe_near'] += 1
        else:
            self.stats['center_probe_no_signal'] += 1

class _CenterProbe80Solver(_CenterProbeSolver):
    probe_radius = 80.0
ALPHA_DEG = 1.0
ALPHA = math.radians(ALPHA_DEG)
SAFE_R = 1000.0
MAX_SOURCE_RANGE = 1500.0
EPS = 2e-07

def _unit(deg):
    a = math.radians(deg)
    return (math.cos(a), math.sin(a))

def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]

def _local(obs, p):
    """Coordinates relative to first measured bearing centerline."""
    u = _unit(obs['svd_deg'])
    n = (-u[1], u[0])
    dx, dy = (p[0] - obs['x'], p[1] - obs['y'])
    return (dx * u[0] + dy * u[1], dx * n[0] + dy * n[1])

def sector_extremes(obs):
    o = (obs['x'], obs['y'])
    pts = []
    for s in (-1.0, 1.0):
        u = _unit(obs['svd_deg'] + s * ALPHA_DEG)
        pts.append((o[0] + MAX_SOURCE_RANGE * u[0], o[1] + MAX_SOURCE_RANGE * u[1]))
    return (o, pts[0], pts[1])

def safe_rho(obs, p):
    return max((math.dist(p, c) for c in sector_extremes(obs)))

def _disk_interval(a, b, c, r=SAFE_R):
    """t interval in [0,1] for a+t(b-a) inside a closed disk."""
    dx, dy = (b[0] - a[0], b[1] - a[1])
    ox, oy = (a[0] - c[0], a[1] - c[1])
    A = dx * dx + dy * dy
    if A < 1e-18:
        return (0.0, 1.0) if ox * ox + oy * oy <= r * r + 1e-09 else None
    B = 2 * (ox * dx + oy * dy)
    C = ox * ox + oy * oy - r * r
    disc = B * B - 4 * A * C
    if disc < -1e-08:
        return None
    root = math.sqrt(max(0.0, disc))
    lo = (-B - root) / (2 * A)
    hi = (-B + root) / (2 * A)
    lo = max(0.0, lo)
    hi = min(1.0, hi)
    return (lo, hi) if lo <= hi + 1e-12 else None

def _linear_interval(a, b, fa, fb, positive=True):
    """t interval where a linear signed quantity f(t) is >=0 (or <=0)."""
    if not positive:
        fa, fb = (-fa, -fb)
    if fa >= -1e-12 and fb >= -1e-12:
        return (0.0, 1.0)
    if fa < -1e-12 and fb < -1e-12:
        return None
    den = fb - fa
    if abs(den) < 1e-18:
        return None
    t = -fa / den
    if fa >= -1e-12:
        return (0.0, min(1.0, max(0.0, t)))
    return (max(0.0, min(1.0, t)), 1.0)

def safe_segment_interval(obs, a, b, side=0):
    """Exact line-segment intersection with the three-disk safe region.

    side=+1 additionally requires the point to lie above the +alpha boundary ray;
    side=-1 below the -alpha ray.  Those side constraints guarantee that one upper
    and one lower point bracket every feasible first-bearing ray.
    """
    lo, hi = (0.0, 1.0)
    for c in sector_extremes(obs):
        z = _disk_interval(a, b, c)
        if z is None:
            return None
        lo = max(lo, z[0])
        hi = min(hi, z[1])
        if lo > hi + 1e-12:
            return None
    if side:
        xa, ya = _local(obs, a)
        xb, yb = _local(obs, b)
        if side > 0:
            fa = ya - xa * math.tan(ALPHA)
            fb = yb - xb * math.tan(ALPHA)
        else:
            fa = -(ya + xa * math.tan(ALPHA))
            fb = -(yb + xb * math.tan(ALPHA))
        z = _linear_interval(a, b, fa, fb, True)
        if z is None:
            return None
        lo = max(lo, z[0])
        hi = min(hi, z[1])
        if lo > hi + 1e-12:
            return None
    return (max(0.0, lo), min(1.0, hi))

def safe_point_on_segment(obs, a, b, side=0):
    """Pick a strong zero-detour probe from the exact feasible interval.

    Endpoints and midpoint suffice for policy scoring; safety itself is exact because
    every candidate lies inside the certified interval.  Prefer smaller rho, then a
    larger angular displacement from the first bearing centerline.
    """
    iv = safe_segment_interval(obs, a, b, side)
    if iv is None:
        return None
    lo, hi = iv
    ts = {lo, hi, (lo + hi) / 2}
    cand = []
    for t in ts:
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        rho = safe_rho(obs, p)
        if rho > SAFE_R + 2e-06:
            continue
        x, y = _local(obs, p)
        angle = abs(math.degrees(math.atan2(y, x)))
        cand.append((rho, -angle, t, p))
    return min(cand)[-1] if cand else None

def _bracket_side(obs, p):
    x, y = _local(obs, p)
    ta = math.tan(ALPHA)
    up = y - x * ta
    low = -(y + x * ta)
    if up > 1e-08:
        return 1
    if low > 1e-08:
        return -1
    return 0

def chord_radial_upper(obs, p_upper, p_lower):
    """Worst intersection radius of chord p_upper--p_lower over +/-alpha rays.

    Requires the two probes to bracket the full first-bearing wedge. Returns None if
    the geometric preconditions are not numerically satisfied.
    """
    if _bracket_side(obs, p_upper) != 1 or _bracket_side(obs, p_lower) != -1:
        return None
    qu = _local(obs, p_upper)
    ql = _local(obs, p_lower)
    d = (ql[0] - qu[0], ql[1] - qu[1])
    num = _cross(qu, ql)
    A = d[1]
    B = -d[0]
    base = math.atan2(B, A)
    es = [-ALPHA, ALPHA]
    for k in range(-2, 3):
        e = base + k * math.pi
        if -ALPHA - 1e-15 <= e <= ALPHA + 1e-15:
            es.append(e)
    vals = []
    for e in es:
        den = A * math.cos(e) + B * math.sin(e)
        if abs(den) < 1e-12:
            continue
        r = num / den
        if r > 0 and math.isfinite(r):
            vals.append(r)
    if not vals:
        return None
    R = max(vals)
    return R if R <= MAX_SOURCE_RANGE + 1e-05 else R

def _clip_halfplane(poly, a, b, c):
    if not poly:
        return []
    out = []
    prev = poly[-1]
    old = a * prev[0] + b * prev[1] - c
    for p in poly:
        val = a * p[0] + b * p[1] - c
        pin = val <= 1e-10
        oin = old <= 1e-10
        if pin != oin:
            den = old - val
            if abs(den) > 1e-18:
                t = old / den
                out.append((prev[0] + t * (p[0] - prev[0]), prev[1] + t * (p[1] - prev[1])))
        if pin:
            out.append(tuple(p))
        prev, old = (p, val)
    return out

def _circle2(a, b):
    c = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    return (c, math.dist(a, b) / 2)

def _circle3(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    aa = ax * ax + ay * ay
    bb = bx * bx + by * by
    cc = cx * cx + cy * cy
    ux = (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by)) / d
    uy = (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax)) / d
    q = (ux, uy)
    return (q, math.dist(q, a))

def _min_circle(points):
    pts = list(points)
    if not pts:
        return None
    c = pts[0]
    r = 0.0
    for i, p in enumerate(pts):
        if math.dist(c, p) > r + 1e-08:
            c = p
            r = 0.0
            for j, q in enumerate(pts[:i]):
                if math.dist(c, q) > r + 1e-08:
                    c, r = _circle2(p, q)
                    for z in pts[:j]:
                        if math.dist(c, z) > r + 1e-08:
                            w = _circle3(p, q, z)
                            if w is not None:
                                c, r = w
    r = max((math.dist(c, p) for p in pts)) + 2e-07
    return (c, r)

def clip_radial_outer(geometry, center, radius, count=96):
    """Intersect an existing outer polygon with a circumscribed radius bound."""
    if not geometry or geometry.get('status') != 'bounded' or (not geometry.get('polygon')):
        return geometry
    poly = [tuple(p) for p in geometry['polygon']]
    rr = radius + 2e-06
    for k in range(count):
        a = math.cos(2 * math.pi * k / count)
        b = math.sin(2 * math.pi * k / count)
        poly = _clip_halfplane(poly, a, b, a * center[0] + b * center[1] + rr)
        if not poly:
            return geometry
    mec = _min_circle(poly)
    if mec is None:
        return geometry
    g = dict(geometry)
    g['polygon'] = poly
    g['mec'] = mec
    g['centers'] = [mec[0]]
    return g

class _SafeBackboneSolver(_CenterProbe80Solver):
    """Conservative first implementation of the analytical backbone resolver.

    It accepts only zero-detour probes on already-paid certificate/source travel legs.
    Safe no_signal observations are paired when possible to add a certified radial
    upper bound.  No active movement is introduced by the new mechanism; the center-probe fallback remains
    the correctness fallback.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.safe_negative = {}
        self.radial_upper = {}
        self.stats.update({'safe_probe_measurements': 0, 'safe_probe_directions': 0, 'safe_probe_no_signal': 0, 'safe_probe_near': 0, 'safe_pairs': 0, 'safe_pair_radial_updates': 0, 'safe_pair_best_radius_m': None, 'safe_site_probes': 0, 'safe_transit_probes': 0, 'fan_deferred_by_safe': 0})

    def observe(self, p, c, reason, site=None, backside=False):
        body = super().observe(p, c, reason, site=site, backside=backside)
        if c in self.radial_upper and c in self.tracks and self.tracks[c].geometry:
            center, r = self.radial_upper[c]
            g = clip_radial_outer(self.tracks[c].geometry, center, r)
            self.tracks[c].geometry = g
            self.tracks[c].mec = g['mec']
        return body

    def _first_obs(self, c):
        t = self.tracks.get(c)
        return t.dirs[0] if t and t.dirs else None

    def _safe_site(self, c, p, side=0):
        obs = self._first_obs(c)
        if obs is None or self._already_sampled(self.tracks[c], p):
            return False
        if safe_rho(obs, p) > SAFE_R + 1e-06:
            return False
        return side == 0 or _bracket_side(obs, p) == side

    def _pair_update(self, c, p):
        obs = self._first_obs(c)
        if obs is None:
            return
        side = _bracket_side(obs, p)
        if side == 0:
            return
        rec = self.safe_negative.setdefault(c, [])
        if all((math.dist(p, q) > 1e-07 for _, q in rec)):
            rec.append((side, tuple(p)))
        upp = [q for s, q in rec if s == 1]
        low = [q for s, q in rec if s == -1]
        best = None
        for u in upp:
            for l in low:
                R = chord_radial_upper(obs, u, l)
                if R is not None and (best is None or R < best[0]):
                    best = (R, u, l)
        if best is None:
            return
        self.stats['safe_pairs'] += 1
        R, u, l = best
        if R >= MAX_SOURCE_RANGE - 1e-06:
            return
        old = self.radial_upper.get(c)
        if old is not None and R >= old[1] - 1e-06:
            return
        center = (obs['x'], obs['y'])
        self.radial_upper[c] = (center, R)
        self.stats['safe_pair_radial_updates'] += 1
        b = self.stats['safe_pair_best_radius_m']
        self.stats['safe_pair_best_radius_m'] = R if b is None else min(b, R)
        t = self.tracks[c]
        if t.geometry:
            g = clip_radial_outer(t.geometry, center, R)
            t.geometry = g
            t.mec = g['mec']
        self.port.record({'kind': 'B4_safe_pair_radius', 'channel': c, 'upper_m': R, 'upper_probe': u, 'lower_probe': l})

    def _record_safe_result(self, c, p, body, where):
        self.stats['safe_probe_measurements'] += 1
        if where == 'site':
            self.stats['safe_site_probes'] += 1
        else:
            self.stats['safe_transit_probes'] += 1
        k = body['measure_result']
        if k == 'direction':
            self.stats['safe_probe_directions'] += 1
        elif k == 'near':
            self.stats['safe_probe_near'] += 1
        else:
            self.stats['safe_probe_no_signal'] += 1
            self._pair_update(c, p)

    def _safe_priority(self, c, p):
        """Higher is better; all returned points are already safe."""
        obs = self._first_obs(c)
        side = _bracket_side(obs, p)
        x, y = _local(obs, p)
        angle = abs(math.degrees(math.atan2(y, x)))
        rho = safe_rho(obs, p)
        rec = self.safe_negative.get(c, [])
        opposite = any((s == -side for s, _ in rec)) if side else False
        return (10000.0 if opposite else 5000.0) + 10.0 * angle + (SAFE_R - rho) / 5.0

    def has_future_opportunity(self, c, remaining):
        if super().has_future_opportunity(c, remaining):
            return True
        for i in remaining:
            p = self.mesh['vertices'][i]
            if self._safe_site(c, p):
                self.stats['fan_deferred_by_safe'] += 1
                return True
        return False

    def scan_certificate(self, site):
        _RouteSolver.scan_certificate(self, site)
        p = self.mesh['vertices'][site]
        candidates = []
        for c, t in self.tracks.items():
            if c in self.cleared or t.near is not None or self._already_sampled(t, p):
                continue
            if self._safe_site(c, p):
                candidates.append((self._safe_priority(c, p), 1, c))
            else:
                score = self.opportunity_score(c, p)
                if score is not None:
                    candidates.append((score, 0, c))
        candidates.sort(reverse=True)
        for _, safe, c in candidates[:self.opportunity_per_site]:
            if c in self.cleared or self._already_sampled(self.tracks[c], p):
                continue
            body = self.observe(p, c, 'B4_certificate_safe_opportunity' if safe else 'B4_certificate_opportunity', backside=bool(safe))
            self.stats['opportunity_measurements'] += 1
            k = body['measure_result']
            if k == 'direction':
                self.stats['opportunity_directions'] += 1
            elif k == 'near':
                self.stats['opportunity_near'] += 1
            else:
                self.stats['opportunity_no_signal'] += 1
            if safe:
                self._record_safe_result(c, p, body, 'site')

    def _best_safe_on_leg(self, c, start, destination):
        t = self.tracks[c]
        obs = self._first_obs(c)
        if obs is None:
            return None
        sides = {s for s, _ in self.safe_negative.get(c, [])}
        wanted = []
        if 1 in sides and -1 not in sides:
            wanted = [-1, 0]
        elif -1 in sides and 1 not in sides:
            wanted = [1, 0]
        else:
            wanted = [1, -1, 0]
        best = None
        for side in wanted:
            p = safe_point_on_segment(obs, start, destination, side)
            if p is None or self._already_sampled(t, p):
                continue
            length = math.dist(start, destination)
            u = 0.0 if length < 1e-12 else math.dist(start, p) / length
            dx, dy = (destination[0] - start[0], destination[1] - start[1])
            L2 = dx * dx + dy * dy
            if L2 > 0:
                u = max(0.0, min(1.0, ((p[0] - start[0]) * dx + (p[1] - start[1]) * dy) / L2))
            score = self._safe_priority(c, p)
            item = (score, u, p, c)
            if best is None or item[0] > best[0]:
                best = item
            if side and best is not None:
                break
        return best

    def transit_opportunities(self, destination, exclude_channel=None):
        start = self.port.position
        length = math.dist(start, destination)
        if length < 1e-07:
            return
        plan = []
        options = []
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None or (not t.dirs):
                continue
            safe = self._best_safe_on_leg(c, start, destination)
            if safe is not None:
                score, u, p, c = safe
                options.append((score + 20000.0, u, p, c, True))
                continue
            best = None
            for u in self.transit_fracs:
                p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
                score = self.opportunity_score(c, p)
                if score is not None and (best is None or score > best[0]):
                    best = (score, u, p, c, False)
            if best is not None:
                options.append(best)
        for score, u, p, c, safe in sorted(options, reverse=True)[:self.transit_per_leg]:
            plan.append((u, 1, 'measure', c, p, safe))
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or (not t.mec) or (t.mec[1] > 20 - 1e-06) or (not t.geometry):
                continue
            verts = t.geometry.get('polygon', [])
            interval = segment_clear_interval(start, destination, verts)
            if interval is None:
                continue
            u = interval[0]
            p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
            plan.append((u, 0, 'clear', c, p, False))
        plan.sort(key=lambda x: (x[0], x[1]))
        for u, _, kind, c, p, safe in plan:
            if c in self.cleared:
                continue
            if kind == 'clear':
                t = self.tracks[c]
                if not t.geometry or not all((math.dist(p, v) <= R_CLEAR + 1e-07 for v in t.geometry.get('polygon', []))):
                    continue
                self.clear(p, c, True, 'B4_transit_full_region_clear')
                self.stats['transit_full_region_clears'] += 1
            else:
                t = self.tracks[c]
                if t.near is not None or self._already_sampled(t, p):
                    continue
                body = self.observe(p, c, 'B4_transit_safe_probe' if safe else 'B4_transit_opportunity', backside=bool(safe))
                self.stats['transit_opportunity_measurements'] += 1
                k = body['measure_result']
                if k == 'direction':
                    self.stats['transit_opportunity_directions'] += 1
                elif k == 'near':
                    self.stats['transit_opportunity_near'] += 1
                else:
                    self.stats['transit_opportunity_no_signal'] += 1
                if safe:
                    self._record_safe_result(c, p, body, 'transit')

class _SafeDistanceSolver(_SafeBackboneSolver):
    """Safe-backbone resolver plus the full distance consequence of every safe probe.

    If q is chosen with rho(q)=max_{g in K0}|q-g|<=1000, then before observing
    anything we already know the true source belongs to B(q,rho(q)).  Intersecting
    this certified disk with the bearing outer set is valid for direction *and*
    no_signal outcomes and is typically much stronger than the generic R<=1500 disk.
    """

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.certified_disks = {}
        self.stats.update({'safe_distance_bounds': 0, 'safe_distance_best_rho_m': None})

    def _apply_certified_disks(self, c):
        if c not in self.tracks:
            return
        t = self.tracks[c]
        if not t.geometry:
            return
        g = t.geometry
        for center, r in self.certified_disks.get(c, []):
            g = clip_radial_outer(g, center, r)
        if c in self.radial_upper:
            center, r = self.radial_upper[c]
            g = clip_radial_outer(g, center, r)
        t.geometry = g
        t.mec = g['mec']

    def observe(self, p, c, reason, site=None, backside=False):
        body = super().observe(p, c, reason, site=site, backside=backside)
        self._apply_certified_disks(c)
        return body

    def _record_safe_result(self, c, p, body, where):
        obs = self._first_obs(c)
        if obs is not None:
            rho = safe_rho(obs, p)
            if rho <= SAFE_R + 3e-06:
                lst = self.certified_disks.setdefault(c, [])
                if all((math.dist(p, q) > 1e-07 for q, _ in lst)):
                    lst.append((tuple(p), rho + 3e-06))
                    self.stats['safe_distance_bounds'] += 1
                    b = self.stats['safe_distance_best_rho_m']
                    self.stats['safe_distance_best_rho_m'] = rho if b is None else min(b, rho)
        super()._record_safe_result(c, p, body, where)
        self._apply_certified_disks(c)
STRIP_CLEAR_R = 19.9995

def _strip_polygon(obs0, D0, obs1, D1):
    """Conservative parallelogram from two bounded-error bearing strips."""
    u0 = _unit(obs0['svd_deg'])
    u1 = _unit(obs1['svd_deg'])
    n0 = (-u0[1], u0[0])
    n1 = (-u1[1], u1[0])
    det = _cross(n0, n1)
    if abs(det) < math.sin(math.radians(3.0)):
        return None
    w0 = D0 * math.sin(ALPHA) + 2e-05
    w1 = D1 * math.sin(ALPHA) + 2e-05
    b0 = n0[0] * obs0['x'] + n0[1] * obs0['y']
    b1 = n1[0] * obs1['x'] + n1[1] * obs1['y']
    verts = []
    for s0 in (-1.0, 1.0):
        for s1 in (-1.0, 1.0):
            c0 = b0 + s0 * w0
            c1 = b1 + s1 * w1
            x = (c0 * n1[1] - n0[1] * c1) / det
            y = (n0[0] * c1 - c0 * n1[0]) / det
            verts.append((x, y))
    cx = sum((x for x, y in verts)) / 4
    cy = sum((y for x, y in verts)) / 4
    verts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    return verts

def _segment_polygon_clear_plan(a, b, poly, R=STRIP_CLEAR_R, max_clears=24):
    """Clear a whole convex polygon using centers constrained to segment a--b.

    If every polygon vertex has normal distance <=D<R and its along-segment
    projection lies in [smin,smax] within the segment, centers spaced so that every
    along-coordinate is within h=sqrt(R^2-D^2) cover the entire polygon.  This is a
    rectangle outer-cover argument, hence conservative for the polygon interior.
    """
    L = math.dist(a, b)
    if L < 1e-09:
        return None
    ux, uy = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
    nx, ny = (-uy, ux)
    ss = []
    ds = []
    for p in poly:
        dx, dy = (p[0] - a[0], p[1] - a[1])
        ss.append(dx * ux + dy * uy)
        ds.append(dx * nx + dy * ny)
    smin, smax = (min(ss), max(ss))
    D = max((abs(x) for x in ds))
    if D >= R - 1e-08 or smin < -1e-07 or smax > L + 1e-07:
        return None
    h = math.sqrt(max(0.0, R * R - D * D))
    span = max(0.0, smax - smin)
    n = max(1, math.ceil(span / (2 * h))) if h > 1e-12 else max_clears + 1
    if n > max_clears:
        return None
    centers = []
    if span < 1e-10:
        ss2 = [(smin + smax) / 2]
    else:
        step = span / n
        ss2 = [smin + (i + 0.5) * step for i in range(n)]
    for s in ss2:
        centers.append((a[0] + s * ux, a[1] + s * uy, s / L))
    return {'centers': centers, 'count': n, 'normal_bound_m': D, 'span_m': span}

class _RouteStripClearSolver(_SafeDistanceSolver):
    """Use safe directions to clear sources for zero extra movement on later paid legs."""
    route_strip_max_clears = 20

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.safe_direction_records = {}
        self.stats.update({'strip_route_plans': 0, 'strip_route_clear_attempts': 0, 'strip_route_clear_successes': 0, 'strip_route_invariant_failures': 0, 'safe_direction_records': 0})

    def _record_safe_result(self, c, p, body, where):
        obs0 = self._first_obs(c)
        rho = safe_rho(obs0, p) if obs0 is not None else None
        super()._record_safe_result(c, p, body, where)
        if body['measure_result'] == 'direction' and rho is not None and (rho <= SAFE_R + 3e-06):
            rec = {'x': p[0], 'y': p[1], 'svd_deg': body['svd_deg'], 'D': rho + 3e-06}
            lst = self.safe_direction_records.setdefault(c, [])
            if all((math.dist(p, (z['x'], z['y'])) > 1e-07 for z in lst)):
                lst.append(rec)
                self.stats['safe_direction_records'] += 1

    def _bearing_records(self, c):
        t = self.tracks[c]
        if not t.dirs:
            return []
        safe = {(round(z['x'], 7), round(z['y'], 7)): z['D'] for z in self.safe_direction_records.get(c, [])}
        out = []
        for o in t.dirs:
            D = safe.get((round(o['x'], 7), round(o['y'], 7)), MAX_SOURCE_RANGE)
            out.append((o, D))
        return out

    def _route_strip_plan(self, c, start, destination):
        rec = self._bearing_records(c)
        if len(rec) < 2:
            return None
        best = None
        for i, (a, Da) in enumerate(rec):
            for b, Db in rec[i + 1:]:
                if min(Da, Db) > SAFE_R + 0.0001:
                    continue
                poly = _strip_polygon(a, Da, b, Db)
                if poly is None:
                    continue
                plan = _segment_polygon_clear_plan(start, destination, poly, max_clears=self.route_strip_max_clears)
                if plan is None:
                    continue
                key = (plan['count'], plan['span_m'], plan['normal_bound_m'])
                if best is None or key < best[0]:
                    best = (key, plan)
        return None if best is None else best[1]

    def transit_opportunities(self, destination, exclude_channel=None):
        start = self.port.position
        length = math.dist(start, destination)
        if length < 1e-07:
            return
        plan = []
        options = []
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None or (not t.dirs):
                continue
            safe = self._best_safe_on_leg(c, start, destination)
            if safe is not None:
                score, u, p, c = safe
                options.append((score + 20000.0, u, p, c, True))
                continue
            best = None
            for u in self.transit_fracs:
                p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
                score = self.opportunity_score(c, p)
                if score is not None and (best is None or score > best[0]):
                    best = (score, u, p, c, False)
            if best is not None:
                options.append(best)
        for score, u, p, c, safe in sorted(options, reverse=True)[:self.transit_per_leg]:
            plan.append((u, 2, 'measure', c, p, safe, None))
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or (not t.mec) or (t.mec[1] > 20 - 1e-06) or (not t.geometry):
                continue
            verts = t.geometry.get('polygon', [])
            interval = segment_clear_interval(start, destination, verts)
            if interval is None:
                continue
            u = interval[0]
            p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
            plan.append((u, 0, 'clear', c, p, False, None))
        gid = 0
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None:
                continue
            z = self._route_strip_plan(c, start, destination)
            if z is None:
                continue
            gid += 1
            self.stats['strip_route_plans'] += 1
            centers = z['centers']
            for j, (p0, p1, u) in enumerate(((p[0], p[1], p[2]) for p in centers)):
                plan.append((u, 1, 'strip_clear', c, (p0, p1), False, (gid, j == len(centers) - 1)))
        plan.sort(key=lambda x: (x[0], x[1]))
        strip_done = set()
        for u, _, kind, c, p, safe, extra in plan:
            if c in self.cleared:
                continue
            if kind == 'clear':
                t = self.tracks[c]
                if not t.geometry or not all((math.dist(p, v) <= R_CLEAR + 1e-07 for v in t.geometry.get('polygon', []))):
                    continue
                self.clear(p, c, True, 'B4_transit_full_region_clear')
                self.stats['transit_full_region_clears'] += 1
            elif kind == 'strip_clear':
                gid, last = extra
                if gid in strip_done:
                    continue
                self.stats['strip_route_clear_attempts'] += 1
                if self.clear(p, c, False, 'B4_strip_route_clear'):
                    strip_done.add(gid)
                    self.stats['strip_route_clear_successes'] += 1
                elif last:
                    self.stats['strip_route_invariant_failures'] += 1
                    raise RuntimeError('certified strip route cover exhausted without clearing')
            else:
                t = self.tracks[c]
                if t.near is not None or self._already_sampled(t, p):
                    continue
                body = self.observe(p, c, 'B4_transit_safe_probe' if safe else 'B4_transit_opportunity', backside=bool(safe))
                self.stats['transit_opportunity_measurements'] += 1
                k = body['measure_result']
                if k == 'direction':
                    self.stats['transit_opportunity_directions'] += 1
                elif k == 'near':
                    self.stats['transit_opportunity_near'] += 1
                else:
                    self.stats['transit_opportunity_no_signal'] += 1
                if safe:
                    self._record_safe_result(c, p, body, 'transit')

def _line_intersection_obs(a, b):
    u = _unit(a['svd_deg'])
    v = _unit(b['svd_deg'])
    det = _cross(u, v)
    if abs(det) < 1e-10:
        return None
    d = (b['x'] - a['x'], b['y'] - a['y'])
    t = _cross(d, v) / det
    return (a['x'] + t * u[0], a['y'] + t * u[1])

def _acute_line_angle_deg(a, b):
    d = abs((a - b + 180.0) % 360.0 - 180.0)
    return min(d, 180.0 - d)

def _active_strip_centers(a, Da, b, Db, here, R=STRIP_CLEAR_R, max_clears=30):
    """Certified 1-D disk cover of the two-strip parallelogram.

    Use the narrower strip as the sweep axis. The intersection of the two centerlines
    is the parallelogram center.  A rectangle of half-width w_narrow and half-length S
    contains the whole parallelogram, so equally spaced clear centers on that axis cover
    it whenever their worst-corner distance is <20 m.
    """
    if Db < Da:
        a, Da, b, Db = (b, Db, a, Da)
    gamma = math.radians(_acute_line_angle_deg(a['svd_deg'], b['svd_deg']))
    if gamma < math.radians(3.0):
        return None
    wA = Da * math.sin(ALPHA) + 2e-05
    wB = Db * math.sin(ALPHA) + 2e-05
    if wA >= R:
        return None
    center = _line_intersection_obs(a, b)
    if center is None:
        return None
    s = abs(math.sin(gamma))
    co = abs(math.cos(gamma))
    S = (wB + wA * co) / s
    h = math.sqrt(max(0.0, R * R - wA * wA))
    n = max(1, math.ceil(S / h)) if h > 1e-12 else max_clears + 1
    if n > max_clears:
        return None
    u = _unit(a['svd_deg'])
    step = 2 * S / n
    pts = [(center[0] + (-S + (i + 0.5) * step) * u[0], center[1] + (-S + (i + 0.5) * step) * u[1]) for i in range(n)]
    if math.dist(here, pts[-1]) < math.dist(here, pts[0]):
        pts.reverse()
    movement = math.dist(here, pts[0]) + sum((math.dist(x, y) for x, y in zip(pts, pts[1:])))
    worst_seconds = movement / 5.0 + 3.0 * (n - 1) + 5.0
    return {'centers': pts, 'count': n, 'movement_m': movement, 'worst_seconds': worst_seconds, 'gamma_deg': math.degrees(gamma), 'narrow_D_m': Da}

class _ActiveStripClearSolver(_RouteStripClearSolver):
    strip_seconds_limit = 100.0
    active_strip_max_clears = 20

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'active_strip_tasks': 0, 'active_strip_attempts': 0, 'active_strip_successes': 0, 'active_strip_invariant_failures': 0})

    def _best_active_strip(self, c):
        rec = self._bearing_records(c)
        if len(rec) < 2:
            return None
        best = None
        for i, (a, Da) in enumerate(rec):
            for b, Db in rec[i + 1:]:
                if min(Da, Db) > SAFE_R + 0.0001:
                    continue
                z = _active_strip_centers(a, Da, b, Db, self.port.position, max_clears=self.active_strip_max_clears)
                if z is None or z['worst_seconds'] > self.strip_seconds_limit:
                    continue
                key = (z['worst_seconds'], z['count'], z['movement_m'])
                if best is None or key < best[0]:
                    best = (key, z)
        return None if best is None else best[1]

    def source_task(self, c):
        base = super().source_task(c)
        if base[0] != 'fan':
            return base
        z = self._best_active_strip(c)
        if z is None:
            return base
        return ('strip', c, z['centers'][0])

    def execute_source(self, task):
        if task[0] != 'strip':
            return super().execute_source(task)
        _, c, _ = task
        z = self._best_active_strip(c)
        if z is None:
            return super().execute_source(super().source_task(c))
        self.stats['active_strip_tasks'] += 1
        self.port.record({'kind': 'B4_active_strip', 'channel': c, 'count': z['count'], 'worst_seconds': z['worst_seconds'], 'movement_m': z['movement_m'], 'gamma_deg': z['gamma_deg'], 'narrow_D_m': z['narrow_D_m']})
        for j, p in enumerate(z['centers']):
            self.stats['active_strip_attempts'] += 1
            if self.clear(p, c, False, 'B4_active_strip_clear'):
                self.stats['active_strip_successes'] += 1
                return
        self.stats['active_strip_invariant_failures'] += 1
        raise RuntimeError('certified active strip cover exhausted without clearing')

def predicted_second_clear_count(obs, p, R=STRIP_CLEAR_R):
    """Pre-measurement worst-case clear count if q returns direction.

    Uses only the first +/-alpha sector and q's certified rho, so it is valid before
    seeing the second bearing.  The same-side far endpoint gives the smallest possible
    second measured-line angle; the two-strip rectangle bound then yields a clear count.
    """
    x, y = _local(obs, p)
    rho = safe_rho(obs, p)
    ay = abs(y)
    sx = MAX_SOURCE_RANGE * math.cos(ALPHA)
    sy = MAX_SOURCE_RANGE * math.sin(ALPHA)
    delta = math.atan2(max(0.0, ay - sy), sx - x)
    gamma = max(0.0, delta - ALPHA)
    if gamma <= 1e-10:
        return 10 ** 9
    w0 = MAX_SOURCE_RANGE * math.sin(ALPHA) + 2e-05
    w1 = rho * math.sin(ALPHA) + 2e-05
    if w1 >= R:
        return 10 ** 9
    S = (w0 + w1 * abs(math.cos(gamma))) / math.sin(gamma)
    h = math.sqrt(max(0.0, R * R - w1 * w1))
    return max(1, math.ceil(S / h)) if h > 1e-12 else 10 ** 9

class _PassiveSafeSolver(_ActiveStripClearSolver):
    """Use safe mathematics only on measurements the center-probe policy would already take.

    No extra probe is created: certificate opportunities use the ordinary opportunity score;
    transit opportunities use the original fixed fractions.  If such a paid measurement
    happens to satisfy rho<=1000, it is upgraded to a proven-backside-capable safe probe
    and can feed the active strip resolver.
    """
    strip_seconds_limit = 100.0

    def has_future_opportunity(self, c, remaining):
        return _CenterProbe80Solver.has_future_opportunity(self, c, remaining)

    def scan_certificate(self, site):
        _RouteSolver.scan_certificate(self, site)
        p = self.mesh['vertices'][site]
        candidates = []
        for c, t in self.tracks.items():
            if c in self.cleared:
                continue
            score = self.opportunity_score(c, p)
            if score is not None:
                candidates.append((score, c))
        candidates.sort(reverse=True)
        for _, c in candidates[:self.opportunity_per_site]:
            if c in self.cleared or self._already_sampled(self.tracks[c], p):
                continue
            safe = self._first_obs(c) is not None and safe_rho(self._first_obs(c), p) <= SAFE_R + 1e-06
            body = self.observe(p, c, 'B4_certificate_opportunity', backside=safe)
            self.stats['opportunity_measurements'] += 1
            k = body['measure_result']
            if k == 'direction':
                self.stats['opportunity_directions'] += 1
            elif k == 'near':
                self.stats['opportunity_near'] += 1
            else:
                self.stats['opportunity_no_signal'] += 1
            if safe:
                self._record_safe_result(c, p, body, 'site')

    def transit_opportunities(self, destination, exclude_channel=None):
        start = self.port.position
        length = math.dist(start, destination)
        if length < 1e-07:
            return
        plan = []
        options = []
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or t.near is not None or (not t.dirs):
                continue
            best = None
            for u in self.transit_fracs:
                p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
                score = self.opportunity_score(c, p)
                if score is not None and (best is None or score > best[0]):
                    best = (score, u, p, c)
            if best is not None:
                options.append(best)
        for score, u, p, c in sorted(options, reverse=True)[:self.transit_per_leg]:
            plan.append((u, 1, 'measure', c, p))
        for c, t in self.tracks.items():
            if c in self.cleared or c == exclude_channel or (not t.mec) or (t.mec[1] > 20 - 1e-06) or (not t.geometry):
                continue
            verts = t.geometry.get('polygon', [])
            interval = segment_clear_interval(start, destination, verts)
            if interval is None:
                continue
            u = interval[0]
            p = (start[0] + u * (destination[0] - start[0]), start[1] + u * (destination[1] - start[1]))
            plan.append((u, 0, 'clear', c, p))
        plan.sort(key=lambda x: (x[0], x[1]))
        for u, _, kind, c, p in plan:
            if c in self.cleared:
                continue
            if kind == 'clear':
                t = self.tracks[c]
                if not t.geometry or not all((math.dist(p, v) <= R_CLEAR + 1e-07 for v in t.geometry.get('polygon', []))):
                    continue
                self.clear(p, c, True, 'B4_transit_full_region_clear')
                self.stats['transit_full_region_clears'] += 1
            else:
                t = self.tracks[c]
                if t.near is not None or self._already_sampled(t, p):
                    continue
                safe = self._first_obs(c) is not None and safe_rho(self._first_obs(c), p) <= SAFE_R + 1e-06
                body = self.observe(p, c, 'B4_transit_opportunity', backside=safe)
                self.stats['transit_opportunity_measurements'] += 1
                k = body['measure_result']
                if k == 'direction':
                    self.stats['transit_opportunity_directions'] += 1
                elif k == 'near':
                    self.stats['transit_opportunity_near'] += 1
                else:
                    self.stats['transit_opportunity_no_signal'] += 1
                if safe:
                    self._record_safe_result(c, p, body, 'transit')

def feasible(q, verts):
    return q is not None and all((math.dist(q, v) <= R_CLEAR + 1e-07 for v in verts))

def candidates(a, b, verts):
    out = []
    iv = segment_clear_interval(a, b, verts)
    if iv is not None:
        for t in (iv[0], iv[1], (iv[0] + iv[1]) / 2):
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if feasible(q, verts):
                out.append(q)
    for x in (a, b):
        q = nearest_clear_point(x, verts, R_CLEAR)
        if feasible(q, verts):
            out.append(q)
    for i, u in enumerate(verts):
        for v in verts[i + 1:]:
            if math.dist(u, v) <= 2 * R_CLEAR + 1e-08:
                for q in _circle_intersections(u, v, R_CLEAR):
                    if feasible(q, verts):
                        out.append(q)
    uniq = []
    for q in out:
        if all((math.dist(q, z) > 1e-07 for z in uniq)):
            uniq.append(q)
    return uniq

def weighted_clear_point(a, b, verts, w):
    cand = candidates(a, b, verts)
    return min(cand, key=lambda q: math.dist(a, q) + w * math.dist(q, b)) if cand else None
CLEAR_R = 19.9999
CELL = math.sqrt(2.0) * CLEAR_R * 0.999999

def _diameter_axis(poly):
    if len(poly) < 2:
        return (1.0, 0.0)
    best = None
    for i, a in enumerate(poly):
        for b in poly[i + 1:]:
            d = (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            if best is None or d > best[0]:
                best = (d, a, b)
    if not best or best[0] < 1e-16:
        return (1.0, 0.0)
    _, a, b = best
    L = math.sqrt(best[0])
    return ((b[0] - a[0]) / L, (b[1] - a[1]) / L)

def rectangle_clear_plan(poly, start):
    if not poly:
        return None
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    xs = [p[0] * u[0] + p[1] * u[1] for p in poly]
    ys = [p[0] * v[0] + p[1] * v[1] for p in poly]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = max(0.0, xmax - xmin)
    wy = max(0.0, ymax - ymin)
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    sx = wx / nx if nx else 0.0
    sy = wy / ny if ny else 0.0
    if nx * ny > 39:
        return None
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    pts = []
    for j in range(ny):
        row = []
        yy = (ymin + ymax) / 2 if wy < 1e-12 else ymin + (j + 0.5) * sy
        for i in range(nx):
            xx = (xmin + xmax) / 2 if wx < 1e-12 else xmin + (i + 0.5) * sx
            row.append((xx * u[0] + yy * v[0], xx * u[1] + yy * v[1]))
        if j % 2:
            row.reverse()
        pts += row
    nodes = [('cell', i, p) for i, p in enumerate(pts)]
    route = fast_open_route(start, nodes)
    pts = [z[2] for z in route]
    move = math.dist(start, pts[0]) + sum((math.dist(a, b) for a, b in zip(pts, pts[1:]))) if pts else 0.0
    worst = move / 5.0 + 3.0 * max(0, len(pts) - 1) + 5.0
    return {'centers': pts, 'count': len(pts), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2}

class _FiniteClearSolver(_PassiveSafeSolver):
    time_limit = 80.0

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'finite_clear_tasks': 0, 'finite_clear_attempts': 0, 'finite_clear_successes': 0, 'finite_clear_invariant_failures': 0})

    def _plan(self, c):
        t = self.tracks[c]
        if not t.geometry or not t.geometry.get('polygon'):
            return None
        return rectangle_clear_plan(t.geometry['polygon'], self.port.position)

    def source_task(self, c):
        base = super().source_task(c)
        if base[0] != 'probe':
            return base
        z = self._plan(c)
        if z is None or z['worst_seconds'] > self.time_limit:
            return base
        return ('gridclear', c, z['centers'][0])

    def execute_source(self, task):
        if task[0] != 'gridclear':
            return super().execute_source(task)
        _, c, _ = task
        z = self._plan(c)
        if z is None or z['worst_seconds'] > self.time_limit:
            return super().execute_source(super().source_task(c))
        self.stats['finite_clear_tasks'] += 1
        self.port.record({'kind': 'B4_finite_clear', 'channel': c, 'count': z['count'], 'worst_seconds': z['worst_seconds'], 'movement_m': z['movement_m'], 'nx': z['nx'], 'ny': z['ny'], 'cell_halfdiag': z['cell_halfdiag']})
        for p in z['centers']:
            self.stats['finite_clear_attempts'] += 1
            if self.clear(p, c, False, 'B4_finite_clear_step'):
                self.stats['finite_clear_successes'] += 1
                return
        self.stats['finite_clear_invariant_failures'] += 1
        raise InvariantFailure('certified rectangle cover exhausted without clear')

class _FiniteClearAnySolver(_FiniteClearSolver):
    """Allow the same certified rectangle cover to replace either a center probe or
    an active fan whenever its complete worst-case time is <= the fixed limit."""

    def source_task(self, c):
        base = super(_FiniteClearSolver, self).source_task(c)
        if base[0] not in ('probe', 'fan'):
            return base
        z = self._plan(c)
        if z is None or z['worst_seconds'] > self.time_limit:
            return base
        return ('gridclear', c, z['centers'][0])

def _axis_meta(poly, u):
    v = (-u[1], u[0])
    xs = [p[0] * u[0] + p[1] * u[1] for p in poly]
    ys = [p[0] * v[0] + p[1] * v[1] for p in poly]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = xmax - xmin
    wy = ymax - ymin
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    n = nx * ny
    if n > 39:
        return None
    sx = wx / nx
    sy = wy / ny
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    snake = max(0.0, (nx - 1) * sx * ny) + (ny - 1) * sy
    proxy = 3.0 * max(0, n - 1) + 5.0 + snake / 5.0
    return (proxy, n, wx + wy, u, (xmin, xmax, ymin, ymax, nx, ny, sx, sy))

def _plan_from_meta(meta, start):
    _, n, _, u, data = meta
    xmin, xmax, ymin, ymax, nx, ny, sx, sy = data
    v = (-u[1], u[0])
    pts = []
    for j in range(ny):
        yy = (ymin + ymax) / 2 if ymax - ymin < 1e-12 else ymin + (j + 0.5) * sy
        row = []
        for i in range(nx):
            xx = (xmin + xmax) / 2 if xmax - xmin < 1e-12 else xmin + (i + 0.5) * sx
            row.append((xx * u[0] + yy * v[0], xx * u[1] + yy * v[1]))
        if j % 2:
            row.reverse()
        pts += row
    nodes = [('cell', i, p) for i, p in enumerate(pts)]
    route = fast_open_route(start, nodes)
    pts = [z[2] for z in route]
    move = math.dist(start, pts[0]) + sum((math.dist(a, b) for a, b in zip(pts, pts[1:]))) if pts else 0.0
    worst = move / 5.0 + 3.0 * max(0, len(pts) - 1) + 5.0
    return {'centers': pts, 'count': len(pts), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2, 'axis': u}

def best_oriented_rectangle_plan(poly, start):
    if not poly:
        return None
    axes = []
    seen = set()

    def add(u):
        L = math.hypot(*u)
        if L < 1e-12:
            return
        u = (u[0] / L, u[1] / L)
        a = math.atan2(u[1], u[0]) % math.pi
        key = round(a, 7)
        if key not in seen:
            seen.add(key)
            axes.append(u)
    add(_diameter_axis(poly))
    for a, b in zip(poly, poly[1:] + poly[:1]):
        add((b[0] - a[0], b[1] - a[1]))
    metas = [m for u in axes if (m := _axis_meta(poly, u)) is not None]
    if not metas:
        return None
    metas.sort(key=lambda m: (m[0], m[1], m[2]))
    best = None
    for m in metas[:5]:
        z = _plan_from_meta(m, start)
        if best is None or (z['worst_seconds'], z['count'], z['movement_m']) < (best['worst_seconds'], best['count'], best['movement_m']):
            best = z
    return best

class _FiniteClear160Solver(_FiniteClearAnySolver):
    time_limit = 160.0

def _point_rect_distance(x, y, xmin, xmax, ymin, ymax):
    dx = 0.0 if xmin <= x <= xmax else xmin - x if x < xmin else x - xmax
    dy = 0.0 if ymin <= y <= ymax else ymin - y if y < ymin else y - ymax
    return math.hypot(dx, dy)

def triangular_clear_plan(poly, start, time_limit=160.0):
    """Strict disk cover via a triangular lattice of covering radius <20 m.

    Nearest-neighbor spacing d=sqrt(3)R gives triangular-cell circumradius R.
    Keeping every lattice center whose R disk intersects the oriented bounding
    rectangle preserves coverage of the full rectangle, hence of the polygon.
    """
    if not poly:
        return None
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    xs = [p[0] * u[0] + p[1] * u[1] for p in poly]
    ys = [p[0] * v[0] + p[1] * v[1] for p in poly]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    R = 19.9998
    d = math.sqrt(3.0) * R
    h = 1.5 * R
    max_count = max(1, int((time_limit - 2.0) // 3) + 1)
    best = None
    for yphase in (0.0, h / 2):
        for xphase in (0.0, d / 2):
            y0 = ymin - R - h + yphase
            y1 = ymax + R + h
            j0 = math.floor((ymin - R - y0) / h) - 1
            j1 = math.ceil((ymax + R - y0) / h) + 1
            rows = []
            pts = []
            for j in range(j0, j1 + 1):
                yy = y0 + j * h
                off = (j & 1) * d / 2 + xphase
                x0 = xmin - R - d + off
                i0 = math.floor((xmin - R - x0) / d) - 1
                i1 = math.ceil((xmax + R - x0) / d) + 1
                row = []
                for i in range(i0, i1 + 1):
                    xx = x0 + i * d
                    if _point_rect_distance(xx, yy, xmin, xmax, ymin, ymax) <= R + 1e-08:
                        row.append((xx * u[0] + yy * v[0], xx * u[1] + yy * v[1]))
                if row:
                    row.sort(key=lambda p: p[0] * u[0] + p[1] * u[1])
                    if len(rows) % 2:
                        row.reverse()
                    rows.append(row)
                    pts.extend(row)
            if not pts or len(pts) > max_count:
                continue
            if math.dist(start, pts[-1]) < math.dist(start, pts[0]):
                pts = list(reversed(pts))
            move = math.dist(start, pts[0]) + sum((math.dist(a, b) for a, b in zip(pts, pts[1:])))
            worst = move / 5.0 + 3.0 * max(0, len(pts) - 1) + 5.0
            z = {'centers': pts, 'count': len(pts), 'movement_m': move, 'worst_seconds': worst, 'nx': None, 'ny': len(rows), 'cell_halfdiag': R, 'lattice': 'triangular'}
            if worst <= time_limit + 1e-09 and (best is None or (worst, len(pts), move) < (best['worst_seconds'], best['count'], best['movement_m'])):
                best = z
    return best

def _clip_uv(poly, axis, value, keep_le=True):
    if not poly:
        return []
    out = []
    prev = poly[-1]
    old = (prev[axis] - value) * (1 if keep_le else -1)
    for p in poly:
        val = (p[axis] - value) * (1 if keep_le else -1)
        pin = val <= 1e-10
        oin = old <= 1e-10
        if pin != oin:
            den = old - val
            if abs(den) > 1e-18:
                t = old / den
                out.append((prev[0] + t * (p[0] - prev[0]), prev[1] + t * (p[1] - prev[1])))
        if pin:
            out.append(p)
        prev, old = (p, val)
    return out

def _cell_intersects(poly_uv, xl, xh, yl, yh):
    q = poly_uv
    q = _clip_uv(q, 0, xh, True)
    q = _clip_uv(q, 0, xl, False)
    q = _clip_uv(q, 1, yh, True)
    q = _clip_uv(q, 1, yl, False)
    return bool(q)

def polygon_pruned_clear_plan(poly, start, time_limit=160.0):
    if not poly:
        return None
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    uv = [(p[0] * u[0] + p[1] * u[1], p[0] * v[0] + p[1] * v[1]) for p in poly]
    xs = [p[0] for p in uv]
    ys = [p[1] for p in uv]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = xmax - xmin
    wy = ymax - ymin
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    if nx * ny > 300:
        return None
    sx = wx / nx
    sy = wy / ny
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    pts = []
    for j in range(ny):
        yl = ymin + j * sy
        yh = ymax if j == ny - 1 else ymin + (j + 1) * sy
        yy = (yl + yh) / 2
        row = []
        for i in range(nx):
            xl = xmin + i * sx
            xh = xmax if i == nx - 1 else xmin + (i + 1) * sx
            if not _cell_intersects(uv, xl, xh, yl, yh):
                continue
            xx = (xl + xh) / 2
            row.append((xx * u[0] + yy * v[0], xx * u[1] + yy * v[1]))
        if j % 2:
            row.reverse()
        pts += row
    max_count = max(1, int((time_limit - 5.0) // 3) + 1)
    if not pts or len(pts) > max_count:
        return None
    nodes = [('cell', i, p) for i, p in enumerate(pts)]
    route = fast_open_route(start, nodes)
    pts = [z[2] for z in route]
    move = math.dist(start, pts[0]) + sum((math.dist(a, b) for a, b in zip(pts, pts[1:])))
    worst = move / 5.0 + 3.0 * max(0, len(pts) - 1) + 5.0
    if worst > time_limit + 1e-09:
        return None
    return {'centers': pts, 'count': len(pts), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2, 'lattice': 'pruned_square'}

def rectangle_clear_plan_limit(poly, start, time_limit):
    """Same strict oriented-rectangle cover as the final finite-clear policy, but let the actual certified
    time budget determine the action-count cap instead of the old 39-cell cap.
    Every kept center still covers its whole grid cell with radius <20m.
    """
    if not poly:
        return None
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    xs = [p[0] * u[0] + p[1] * u[1] for p in poly]
    ys = [p[0] * v[0] + p[1] * v[1] for p in poly]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = max(0.0, xmax - xmin)
    wy = max(0.0, ymax - ymin)
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    sx = wx / nx if nx else 0.0
    sy = wy / ny if ny else 0.0
    max_count = max(1, int((time_limit - 5.0) // 3) + 1)
    if nx * ny > max_count:
        return None
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    pts = []
    for j in range(ny):
        yy = (ymin + ymax) / 2 if wy < 1e-12 else ymin + (j + 0.5) * sy
        row = []
        for i in range(nx):
            xx = (xmin + xmax) / 2 if wx < 1e-12 else xmin + (i + 0.5) * sx
            row.append((xx * u[0] + yy * v[0], xx * u[1] + yy * v[1]))
        if j % 2:
            row.reverse()
        pts += row
    nodes = [('cell', i, p) for i, p in enumerate(pts)]
    route = fast_open_route(start, nodes)
    pts = [z[2] for z in route]
    move = math.dist(start, pts[0]) + sum((math.dist(a, b) for a, b in zip(pts, pts[1:]))) if pts else 0.0
    worst = move / 5.0 + 3.0 * max(0, len(pts) - 1) + 5.0
    if worst > time_limit + 1e-09:
        return None
    return {'centers': pts, 'count': len(pts), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2, 'lattice': 'dynamic_square'}

class _RouteWideTSPNSolver(_FiniteClear160Solver):
    rounds = 3
    exit_weight = 1.0

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'route_repoints': 0, 'route_est_gain_m': 0.0})

    @staticmethod
    def _cost(start, route):
        p = start
        z = 0.0
        for t in route:
            z += math.dist(p, t[2])
            p = t[2]
        return z

    def _repoint(self, nodes):
        work = list(nodes)
        base = fast_open_route(self.port.position, work)
        basecost = self._cost(self.port.position, base)
        best_route = base
        best_cost = basecost
        for _ in range(self.rounds):
            route = fast_open_route(self.port.position, work)
            changed = []
            anychange = False
            for i, t in enumerate(route):
                if t[0] != 'guaranteed':
                    changed.append(t)
                    continue
                tr = self.tracks.get(t[1])
                verts = tr.geometry.get('polygon', []) if tr and tr.geometry else []
                if not verts:
                    changed.append(t)
                    continue
                prev = self.port.position if i == 0 else route[i - 1][2]
                if i + 1 < len(route):
                    nxt = route[i + 1][2]
                    q = weighted_clear_point(prev, nxt, verts, self.exit_weight)
                else:
                    q = weighted_clear_point(prev, prev, verts, 0.0)
                if q is None or not feasible(q, verts):
                    changed.append(t)
                    continue
                nt = (t[0], t[1], q)
                changed.append(nt)
                anychange |= math.dist(q, t[2]) > 1e-07
            cand = fast_open_route(self.port.position, changed)
            cc = self._cost(self.port.position, cand)
            if cc + 1e-08 < best_cost:
                best_route, best_cost = (cand, cc)
            work = changed
            if not anychange:
                break
        if best_cost + 1e-08 < basecost:
            self.stats['route_est_gain_m'] += basecost - best_cost
            for a, b in zip(base, best_route):
                if a[0] == 'guaranteed' and b[0] == 'guaranteed' and (a[1] == b[1]) and (math.dist(a[2], b[2]) > 1e-07):
                    self.stats['route_repoints'] += 1
        return best_route

    def select_task(self, nodes):
        route = self._repoint(nodes)
        return route[0] if route else None

def _clip_axis(poly, axis, bound, le):
    """Clip a uv polygon to coord<=bound (le) or coord>=bound."""
    if not poly:
        return []
    out = []
    prev = poly[-1]

    def val(p):
        return p[axis] - bound if le else bound - p[axis]
    old = val(prev)
    for p in poly:
        cur = val(p)
        pin = cur <= 1e-12
        oin = old <= 1e-12
        if pin != oin:
            den = old - cur
            if abs(den) > 1e-20:
                t = old / den
                out.append((prev[0] + t * (p[0] - prev[0]), prev[1] + t * (p[1] - prev[1])))
        if pin:
            out.append(p)
        prev, old = (p, cur)
    return out

def _cell_area(poly_uv, xl, xh, yl, yh):
    q = _clip_axis(poly_uv, 0, xh, True)
    q = _clip_axis(q, 0, xl, False)
    q = _clip_axis(q, 1, yh, True)
    q = _clip_axis(q, 1, yl, False)
    if len(q) < 3:
        return 0.0
    return abs(sum((a[0] * b[1] - a[1] * b[0] for a, b in zip(q, q[1:] + q[:1])))) * 0.5

def _expected_objective(start, ordered):
    """Expected virtual seconds to success under disjoint cell masses.

    ordered entries are (point, probability).  Moving edge k is paid iff the target
    has not been found earlier; a failed clear costs 3 s, terminal success 5 s.
    """
    remaining = sum((w for _, w in ordered))
    if remaining <= 0:
        return math.inf
    p = start
    obj = 5.0
    survived = 1.0
    total = remaining
    for i, (q, w) in enumerate(ordered):
        obj += math.dist(p, q) / 5.0 * survived
        prob = max(0.0, w) / total
        survived = max(0.0, survived - prob)
        if i + 1 < len(ordered):
            obj += 3.0 * survived
        p = q
    return obj

def _full_movement(start, order):
    if not order:
        return 0.0
    return math.dist(start, order[0][0]) + sum((math.dist(a[0], b[0]) for a, b in zip(order, order[1:])))

def _weighted_search_route(start, pts_weights):
    """Small deterministic heuristic for weighted minimum-latency search.

    Seed with several probability/distance greedy policies, then adjacent/swap local
    search directly on the expected-time objective. n<=39 under the inherited gate.
    """
    n = len(pts_weights)
    if n <= 1:
        return list(pts_weights)
    weights = [max(0.0, w) for _, w in pts_weights]
    if sum(weights) <= 1e-16:
        nodes = [('cell', i, p) for i, (p, _) in enumerate(pts_weights)]
        route = fast_open_route(start, nodes)
        by = {i: pts_weights[i] for i in range(n)}
        return [by[t[1]] for t in route]
    seeds = []
    nodes = [('cell', i, p) for i, (p, _) in enumerate(pts_weights)]
    tsp = fast_open_route(start, nodes)
    seeds.append([t[1] for t in tsp])
    for alpha in (0.35, 0.6, 0.85, 1.0, 1.25, 1.6):
        rem = set(range(n))
        cur = start
        order = []
        while rem:

            def key(i):
                d = math.dist(cur, pts_weights[i][0])
                w = max(weights[i], 1e-12)
                return (d / w ** alpha, d, -w, i)
            i = min(rem, key=key)
            rem.remove(i)
            order.append(i)
            cur = pts_weights[i][0]
        seeds.append(order)
    best = None
    for order in seeds:
        seq = [pts_weights[i] for i in order]
        score = _expected_objective(start, seq)
        if best is None or score < best[0]:
            best = (score, list(order))
    score, order = best
    for _ in range(5):
        improved = False
        for gap in (1, 2, 3, 5, 8):
            for i in range(n - gap):
                j = i + gap
                cand = list(order)
                cand[i], cand[j] = (cand[j], cand[i])
                cs = _expected_objective(start, [pts_weights[k] for k in cand])
                if cs + 1e-09 < score:
                    order, score = (cand, cs)
                    improved = True
        if not improved:
            break
    return [pts_weights[i] for i in order]

def probability_rectangle_clear_plan(poly, start, time_limit=160.0):
    if not poly:
        return None
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    uv = [(p[0] * u[0] + p[1] * u[1], p[0] * v[0] + p[1] * v[1]) for p in poly]
    xs = [p[0] for p in uv]
    ys = [p[1] for p in uv]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = max(0.0, xmax - xmin)
    wy = max(0.0, ymax - ymin)
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    if nx * ny > 39:
        return None
    sx = wx / nx if nx else 0.0
    sy = wy / ny if ny else 0.0
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    pw = []
    for j in range(ny):
        yl = ymin + j * sy if wy >= 1e-12 else ymin
        yh = ymax if j == ny - 1 else ymin + (j + 1) * sy if wy >= 1e-12 else ymax
        yy = (ymin + ymax) / 2 if wy < 1e-12 else (yl + yh) / 2
        for i in range(nx):
            xl = xmin + i * sx if wx >= 1e-12 else xmin
            xh = xmax if i == nx - 1 else xmin + (i + 1) * sx if wx >= 1e-12 else xmax
            xx = (xmin + xmax) / 2 if wx < 1e-12 else (xl + xh) / 2
            p = (xx * u[0] + yy * v[0], xx * u[1] + yy * v[1])
            w = _cell_area(uv, xl, xh, yl, yh) if wx >= 1e-12 and wy >= 1e-12 else 0.0
            pw.append((p, w))
    ordered = _weighted_search_route(start, pw)
    centers = [p for p, _ in ordered]
    move = _full_movement(start, ordered)
    worst = move / 5.0 + 3.0 * max(0, len(centers) - 1) + 5.0
    if worst > time_limit + 1e-09:
        nodes = [('cell', i, p) for i, (p, _) in enumerate(pw)]
        route = fast_open_route(start, nodes)
        centers = [z[2] for z in route]
        move = math.dist(start, centers[0]) + sum((math.dist(a, b) for a, b in zip(centers, centers[1:]))) if centers else 0.0
        worst = move / 5.0 + 3.0 * max(0, len(centers) - 1) + 5.0
        if worst > time_limit + 1e-09:
            return None
        weighted_used = False
        expected = None
    else:
        weighted_used = True
        expected = _expected_objective(start, ordered)
    return {'centers': centers, 'count': len(centers), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2, 'weighted_used': weighted_used, 'expected_seconds_proxy': expected, 'positive_mass_cells': sum((w > 1e-12 for _, w in pw))}

class _ProbeFirstClearSolver(_RouteWideTSPNSolver):

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.probe_clear_tried = set()
        self.stats.update({'probe_clear_attempts': 0, 'probe_clear_successes': 0, 'probe_clear_failures': 0})

    def _plan(self, c):
        t = self.tracks[c]
        if not t.geometry or not t.geometry.get('polygon'):
            return None
        return probability_rectangle_clear_plan(t.geometry['polygon'], self.port.position, self.time_limit)

    def source_task(self, c):
        base = super().source_task(c)
        if base[0] == 'probe' and c not in self.probe_clear_tried:
            return ('tryclear', c, base[2])
        return base

    def execute_source(self, task):
        if task[0] != 'tryclear':
            return super().execute_source(task)
        _, c, p = task
        self.probe_clear_tried.add(c)
        self.stats['probe_clear_attempts'] += 1
        if self.clear(p, c, False, 'B4_probe_first_clear'):
            self.stats['probe_clear_successes'] += 1
        else:
            self.stats['probe_clear_failures'] += 1
CLEAR_EXCLUSION_R = 20.0
PROOF_R = math.nextafter(CLEAR_EXCLUSION_R, 0.0) - 1e-09

def _world(u, v, x, y):
    return (x * u[0] + y * v[0], x * u[1] + y * v[1])

def _rect_corners(u, v, xl, xh, yl, yh):
    return (_world(u, v, xl, yl), _world(u, v, xl, yh), _world(u, v, xh, yl), _world(u, v, xh, yh))

def _rect_in_one_failed_disk(u, v, xl, xh, yl, yh, q):
    r2 = PROOF_R * PROOF_R
    return all(((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= r2 for p in _rect_corners(u, v, xl, xh, yl, yh)))

def _rect_covered_by_failed_union(u, v, xl, xh, yl, yh, failed, depth=3):
    """One-sided proof that a rotated rectangle lies in a union of failed disks.

    True is rigorous up to the inward radius guard. False means only "not proved".
    Recursive subdivision permits several failed disks to jointly cover a cell.
    """
    if not failed:
        return False
    if any((_rect_in_one_failed_disk(u, v, xl, xh, yl, yh, q) for q in failed)):
        return True
    if depth <= 0 or xh - xl < 1e-08 or yh - yl < 1e-08:
        return False
    xm = (xl + xh) / 2
    ym = (yl + yh) / 2
    return all((_rect_covered_by_failed_union(u, v, a, b, c, d, failed, depth - 1) for a, b, c, d in ((xl, xm, yl, ym), (xm, xh, yl, ym), (xl, xm, ym, yh), (xm, xh, ym, yh))))

def _outside_fraction_proxy(u, v, xl, xh, yl, yh, failed):
    """Ordering-only estimate of cell mass surviving the failed-clear disks."""
    if not failed:
        return 1.0
    good = 0
    total = 25
    r2 = CLEAR_EXCLUSION_R * CLEAR_EXCLUSION_R
    for j in range(5):
        y = yl + (j + 0.5) * (yh - yl) / 5
        for i in range(5):
            x = xl + (i + 0.5) * (xh - xl) / 5
            p = _world(u, v, x, y)
            if not any(((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= r2 for q in failed)):
                good += 1
    return good / total

def failure_aware_rectangle_clear_plan(poly, start, failed, time_limit=160.0, soft_weights=False):
    if not poly:
        return None
    failed = [tuple(q) for q in failed]
    u = _diameter_axis(poly)
    v = (-u[1], u[0])
    uv = [(p[0] * u[0] + p[1] * u[1], p[0] * v[0] + p[1] * v[1]) for p in poly]
    xs = [p[0] for p in uv]
    ys = [p[1] for p in uv]
    xmin, xmax = (min(xs), max(xs))
    ymin, ymax = (min(ys), max(ys))
    wx = max(0.0, xmax - xmin)
    wy = max(0.0, ymax - ymin)
    nx = max(1, math.ceil(wx / CELL))
    ny = max(1, math.ceil(wy / CELL))
    if nx * ny > 39:
        return None
    sx = wx / nx if nx else 0.0
    sy = wy / ny if ny else 0.0
    if math.hypot(sx, sy) / 2 >= 20 - 1e-06:
        return None
    entries = []
    pruned = 0
    raw_mass = 0.0
    surviving_mass = 0.0
    for j in range(ny):
        yl = ymin + j * sy if wy >= 1e-12 else ymin
        yh = ymax if j == ny - 1 else ymin + (j + 1) * sy if wy >= 1e-12 else ymax
        yy = (ymin + ymax) / 2 if wy < 1e-12 else (yl + yh) / 2
        for i in range(nx):
            xl = xmin + i * sx if wx >= 1e-12 else xmin
            xh = xmax if i == nx - 1 else xmin + (i + 1) * sx if wx >= 1e-12 else xmax
            xx = (xmin + xmax) / 2 if wx < 1e-12 else (xl + xh) / 2
            p = _world(u, v, xx, yy)
            if wx >= 1e-12 and wy >= 1e-12:
                base = _cell_area(uv, xl, xh, yl, yh)
            else:
                base = 0.0
            raw_mass += base
            if _rect_covered_by_failed_union(u, v, xl, xh, yl, yh, failed):
                pruned += 1
                continue
            frac = _outside_fraction_proxy(u, v, xl, xh, yl, yh, failed) if soft_weights else 1.0
            w = base * frac
            surviving_mass += w
            entries.append({'point': p, 'weight': w, 'raw_weight': base, 'bounds': (xl, xh, yl, yh)})
    if not entries:
        return None
    ordered_pairs = _weighted_search_route(start, [(e['point'], e['weight']) for e in entries])
    by_point = {e['point']: e for e in entries}
    ordered = [by_point[p] for p, _ in ordered_pairs]
    centers = [e['point'] for e in ordered]
    move = math.dist(start, centers[0]) + sum((math.dist(a, b) for a, b in zip(centers, centers[1:]))) if centers else 0.0
    worst = move / 5.0 + 3.0 * max(0, len(centers) - 1) + 5.0
    weighted_used = True
    if worst > time_limit + 1e-09:
        nodes = [('cell', i, e['point']) for i, e in enumerate(entries)]
        route = fast_open_route(start, nodes)
        idx = {e['point']: e for e in entries}
        ordered = [idx[z[2]] for z in route]
        centers = [e['point'] for e in ordered]
        move = math.dist(start, centers[0]) + sum((math.dist(a, b) for a, b in zip(centers, centers[1:]))) if centers else 0.0
        worst = move / 5.0 + 3.0 * max(0, len(centers) - 1) + 5.0
        if worst > time_limit + 1e-09:
            return None
        weighted_used = False
    expected = _expected_objective(start, [(e['point'], e['weight']) for e in ordered]) if weighted_used else None
    return {'centers': centers, 'cells': ordered, 'count': len(centers), 'movement_m': move, 'worst_seconds': worst, 'nx': nx, 'ny': ny, 'cell_halfdiag': math.hypot(sx, sy) / 2, 'weighted_used': weighted_used, 'expected_seconds_proxy': expected, 'positive_mass_cells': sum((e['weight'] > 1e-12 for e in entries)), 'failure_pruned_cells': pruned, 'failure_raw_cells': nx * ny, 'failure_raw_mass': raw_mass, 'failure_surviving_mass_proxy': surviving_mass, 'axis_u': u, 'axis_v': v}

class _FailureAwareClearSolver(_ProbeFirstClearSolver):
    soft_weights = False

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.stats.update({'failure_aware_plans': 0, 'failure_planned_pruned_cells': 0, 'failure_dynamic_pruned_cells': 0, 'failure_grid_failures_seen': 0})

    def _plan(self, c):
        t = self.tracks[c]
        if not t.geometry or not t.geometry.get('polygon'):
            return None
        z = failure_aware_rectangle_clear_plan(t.geometry['polygon'], self.port.position, t.clear_fail_points, self.time_limit, self.soft_weights)
        if z:
            self.stats['failure_aware_plans'] += 1
            self.stats['failure_planned_pruned_cells'] += z.get('failure_pruned_cells', 0)
        return z

    def execute_source(self, task):
        if task[0] != 'gridclear':
            return super().execute_source(task)
        _, c, _ = task
        z = self._plan(c)
        if z is None or z['worst_seconds'] > self.time_limit:
            return super().execute_source(super().source_task(c))
        self.stats['finite_clear_tasks'] += 1
        self.port.record({'kind': 'B4_failure_aware_finite_clear', 'channel': c, 'count': z['count'], 'worst_seconds': z['worst_seconds'], 'movement_m': z['movement_m'], 'nx': z['nx'], 'ny': z['ny'], 'cell_halfdiag': z['cell_halfdiag'], 'pruned_before_start': z['failure_pruned_cells']})
        pending = list(z['cells'])
        u = z['axis_u']
        v = z['axis_v']
        while pending:
            e = pending.pop(0)
            p = e['point']
            if p in self.tracks[c].clear_fail_points:
                self.stats['failure_dynamic_pruned_cells'] += 1
                continue
            self.stats['finite_clear_attempts'] += 1
            if self.clear(p, c, False, 'B4_failure_aware_clear_step'):
                self.stats['finite_clear_successes'] += 1
                return
            self.stats['failure_grid_failures_seen'] += 1
            failed = self.tracks[c].clear_fail_points
            keep = []
            for q in pending:
                xl, xh, yl, yh = q['bounds']
                if _rect_covered_by_failed_union(u, v, xl, xh, yl, yh, failed):
                    self.stats['failure_dynamic_pruned_cells'] += 1
                else:
                    keep.append(q)
            pending = keep
        self.stats['finite_clear_invariant_failures'] += 1
        raise InvariantFailure('certified failure-aware cover exhausted without clear')

class B4Solver(_FailureAwareClearSolver):
    soft_weights = True

def _triangle_inside_disk(tri, center):
    r2 = PROOF_R * PROOF_R
    return all(((p[0] - center[0]) ** 2 + (p[1] - center[1]) ** 2 <= r2 for p in tri))

def _triangle_covered_by_disks(tri, centers, depth=6):
    """One-sided proof a triangle is contained in a union of clear disks."""
    if any((_triangle_inside_disk(tri, c) for c in centers)):
        return True
    if depth <= 0:
        return False
    a, b, c = tri
    ab = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    bc = ((b[0] + c[0]) / 2, (b[1] + c[1]) / 2)
    ca = ((c[0] + a[0]) / 2, (c[1] + a[1]) / 2)
    return all((_triangle_covered_by_disks(t, centers, depth - 1) for t in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca))))

def _polygon_covered_by_disks(poly, centers, depth=6):
    if not poly:
        return False
    if len(poly) < 3:
        return all((any((math.dist(p, c) <= PROOF_R for c in centers)) for p in poly))
    a = poly[0]
    return all((_triangle_covered_by_disks((a, poly[i], poly[i + 1]), centers, depth) for i in range(1, len(poly) - 1)))

def _two_lobe_cover(poly, center):
    """Certify center disk + two symmetric diameter-axis disks cover the polygon.

    Search only a 1-D family; returning None never affects correctness.  The first
    center disk is known empty when this plan is used, leaving at most two clear
    actions to finish the source.
    """
    if not poly or len(poly) < 2:
        return None
    u = _diameter_axis(poly)
    for k in range(30, 141):
        d = 0.5 * k
        qplus = (center[0] + d * u[0], center[1] + d * u[1])
        qminus = (center[0] - d * u[0], center[1] - d * u[1])
        if _polygon_covered_by_disks(poly, (center, qplus, qminus), 6):
            return {'offset_m': d, 'plus': qplus, 'minus': qminus, 'worst_after_center_failure_s': 3 * d / 5.0 + 8.0}
    return None

def _lobe_mass_proxy(poly, center, u, step=3.0):
    """Ordering-only area proxy outside the already-failed center disk."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]

    def inside(p):
        sign = None
        for a, b in zip(poly, poly[1:] + poly[:1]):
            z = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
            if abs(z) <= 1e-09:
                continue
            s = z > 0
            if sign is None:
                sign = s
            elif s != sign:
                return False
        return True
    neg = pos = 0
    x = min(xs)
    while x <= max(xs) + 1e-09:
        y = min(ys)
        while y <= max(ys) + 1e-09:
            p = (x, y)
            if inside(p) and math.dist(p, center) > CLEAR_EXCLUSION_R:
                proj = (x - center[0]) * u[0] + (y - center[1]) * u[1]
                if proj >= 0:
                    pos += 1
                else:
                    neg += 1
            y += step
        x += step
    return (neg, pos)

__all__ = ["B4Solver", "RobotPort", "InvariantFailure"]
