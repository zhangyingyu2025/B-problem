"""E13 source-blind engine. Frozen E11 is inherited, never edited."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
sys.path.insert(0, str(HERE))
from e12_coverage_hook import E11
from run_e11_rehearsal import build_solver, ClientEnv
from b1plus import intersection
from clear_risk import classify, localization_point


class E13Solver(E11.E11Solver):
    def __init__(self, case, variant='A'):
        if variant not in ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R'):
            raise ValueError('unknown E13 variant: '+str(variant))
        super().__init__(case)
        self.variant = variant
        self.history_silent = {c: [] for c in range(1, 21)}
        self.localizations = {}
        self.b1plus_updates = 0
        self.b1plus_empty = 0
        self.guaranteed_clear_failures = 0
        self.policy_localizations = 0
        self.risky_clear_tasks = 0
        self.recovery_movement_m = 0.

    def handle(self, p, c, res, site_idx=None, reason=''):
        if c in self.env.cleared:
            return
        point = tuple(p)
        if res['measure_result'] == 'no_signal':
            if point not in self.history_silent[c]:
                self.history_silent[c].append(point)
            if site_idx is not None and c not in self.tracks:
                self.checked[c].add(site_idx)
            if c not in self.tracks:
                return
        t = self.tracks.setdefault(c, {'channel': c, 'dirs': [], 'nosig': [], 'near': None, 'poly': [], 'mec': None, 'signals': []})
        t['nosig'] = list(self.history_silent[c])
        if res['measure_result'] == 'near':
            t['near'] = point
            if point not in t['signals']:
                t['signals'].append(point)
        elif res['measure_result'] == 'direction':
            o = {'x': p[0], 'y': p[1], 'svd_deg': res['svd_deg']}
            if o not in t['dirs']:
                t['dirs'].append(o)
            if point not in t['signals']:
                t['signals'].append(point)
        if t['dirs']:
            g = intersection(t['dirs'], t['signals'], t['nosig'])
            t['geometry'] = g
            self.b1plus_updates += 1
            if g['status'] == 'bounded':
                t['poly'], t['mec'] = g['polygon'], g['mec']
            else:
                self.b1plus_empty += 1
                # Empty numerical result cannot prove disappearance; retain bearing fallback.
                E11.e5.e1.exact_track_geometry(t)

    def mec_info(self, t):
        return t.get('mec')

    def localized(self, t):
        return bool(t.get('mec') and t['mec'][1] <= 19.999)

    def segment_points(self, t, a, b, maxpts=2):
        if self.variant not in ('E', 'F', 'P'):
            return super().segment_points(t, a, b, maxpts)
        g = t.get('geometry')
        if not g or g['status'] != 'bounded' or self.localized(t):
            return []
        center, radius = g['mec']; choices = []
        for k in range(1, 40):
            u = k/40; point = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
            if max(math.dist(point, vertex) for vertex in g['polygon']) > 1000-1e-6:
                continue
            dist = math.dist(point, center)
            if dist < 1e-6:
                continue
            new_angle = math.atan2(center[1]-point[1], center[0]-point[0])
            separation = max(abs(math.sin(new_angle-math.atan2(center[1]-o['y'], center[0]-o['x']))) for o in t['dirs'])
            if separation < .1:
                continue
            predicted_diameter = 4*dist*math.tan(math.radians(1))/separation
            if predicted_diameter >= .65*g['diameter']:
                continue
            if any(math.dist(point, (o['x'], o['y'])) < 5 for o in t['dirs']):
                continue
            choices.append((predicted_diameter, u, point, separation))
        if not choices:
            return []
        _, u, point, separation = min(choices)
        return [(u, point, separation)]

    def eligible_targets(self):
        result = []
        for c, t in self.tracks.items():
            if c in self.env.cleared:
                continue
            state = classify(t.get('geometry'), self.env.pos)
            if state['kind'] == 'guaranteed_clear':
                result.append(('clear', c, state['point']))
            elif self.variant in ('J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R') and state['kind'] == 'risky_clear':
                result.append(('clear', c, state['point']))
        return result

    def broad_tracks(self):
        return [t for c, t in self.tracks.items() if c not in self.env.cleared and t['dirs']
                and not self.localized(t) and self.joint_supp_count.get(c, 0) < 3
                and not (self.variant in ('J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R') and t.get('mec') and t['mec'][1] <= 80)]

    def route_fit_supp(self, t, structural):
        g = t.get('geometry')
        if g and g['status'] == 'bounded':
            if self.variant == 'K':
                from measurement_route import choose
                p = choose(g, self.env.pos, structural)
            else:
                p = localization_point(g, self.env.pos, allow_wide=self.variant in ('O', 'P', 'Q', 'R'))
            if p is not None and all(math.dist(p, (o['x'], o['y'])) > 1 for o in t['dirs']):
                return p
        return super().route_fit_supp(t, structural)

    def joint_nodes(self, remaining):
        if self.variant == 'Q':
            from measurement_options import joint_nodes
            return joint_nodes(self, remaining)
        return super().joint_nodes(remaining)

    def joint_phase(self, remaining_cover):
        if self.variant in ('M', 'N', 'O', 'P', 'Q', 'R'):
            from event_route import run
            return run(self, remaining_cover)
        if self.variant in ('A', 'E', 'J', 'K'):
            return super().joint_phase(remaining_cover)
        if self.variant == 'F':
            return self.future_measure_phase(remaining_cover)
        if self.variant in ('C', 'D', 'G'):
            return self.macro_phase(remaining_cover)
        if self.variant == 'H':
            return self.sweep_phase(remaining_cover)
        from beam_router import choose
        remaining = list(remaining_cover)
        for step in range(120):
            nodes = self.joint_nodes(remaining)
            if not nodes:
                return
            if self.variant == 'L':
                from scenario_route import choose as scenario_choose
                (kind, key, point), diagnostic = scenario_choose(
                    self.env.pos, nodes, self.tracks, self.env.cleared, self.history_silent, remaining)
            else:
                (kind, key, point), diagnostic = choose(self, nodes, remaining)
            self.env.c.record({'kind': 'E13_beam_decision', 'step': step, 'action': [kind, key, point], **diagnostic})
            self.transit_measure(self.env.pos, point, exclude=key if kind != 'cover' else None)
            if kind == 'cover':
                self.scan_site(key, point)
                remaining = [x for x in remaining if x[0] != key]
                if self.discovery_complete() and len(self.tracks) >= 16:
                    remaining = []
            elif kind == 'clear':
                if key not in self.env.cleared and self.local_clear_from_mec(self.tracks[key]):
                    self.joint_clears += 1
            elif key not in self.env.cleared:
                self.measure(point, key, None, 'E13_'+kind)
                self.stats['supplemental_points'] += 1; self.joint_supp += 1
                self.joint_supp_count[key] = self.joint_supp_count.get(key, 0)+1
                self.maybe_clear_near(self.tracks[key])
            self.joint_steps += 1
            if not remaining and self.discovery_complete() and not self.eligible_targets() and not self.broad_tracks():
                return
        raise RuntimeError('E13 joint routing guard exceeded')

    def macro_phase(self, remaining_cover):
        from macro_route import choose
        remaining = list(remaining_cover)
        attempts = {}
        for step in range(120):
            if self.variant == 'G':
                from skeleton_route import choose as sweep_choose
                action = sweep_choose(self, remaining)
            else:
                action = choose(self, remaining, beam=self.variant == 'D')
            if action is None:
                return
            kind, key, target = action
            self.env.c.record({'kind': 'E13_macro_decision', 'step': step, 'action': action})
            self.transit_measure(self.env.pos, target, exclude=key if kind == 'source' else None)
            if kind == 'cover':
                self.scan_site(key, target)
                remaining = [x for x in remaining if x[0] != key]
                if self.discovery_complete() and len(self.tracks) >= 16:
                    remaining = []
            elif key not in self.env.cleared:
                track = self.tracks[key]
                g = track.get('geometry')
                attempts[key] = attempts.get(key, 0)+1
                if g and g['mec'][1] <= 500:
                    if self.local_clear_from_mec(track, 500):
                        self.joint_clears += 1
                else:
                    point = self.route_fit_supp(track, [('cover', idx, p) for idx, p in remaining])
                    if point is not None:
                        self.measure(point, key, None, 'E13_macro_supplement')
                        self.stats['supplemental_points'] += 1; self.joint_supp += 1
                        self.maybe_clear_near(track)
                if attempts[key] > 6 and key not in self.env.cleared:
                    # Retain inherited finite grid for a genuinely difficult source.
                    self.full_grid(track)
            self.joint_steps += 1
            if not remaining and self.discovery_complete() and all(c in self.env.cleared for c in self.tracks):
                return
        raise RuntimeError('E13 macro routing guard exceeded')

    def future_measure_phase(self, remaining_cover):
        """Value upcoming coverage measurements before committing a dedicated detour."""
        remaining = list(remaining_cover)
        for step in range(120):
            structural = [('cover', idx, p) for idx, p in remaining]+self.eligible_targets()
            nodes = list(structural)
            blocked = set()
            for track in self.broad_tracks():
                # These are measurements along a mandatory future edge, not guessed source truth.
                future_edges = [(self.env.pos, p) for _, p in remaining]
                useful = [self.segment_points(track, a, b, 1) for a, b in future_edges]
                if any(useful):
                    blocked.add(track['channel'])
                    continue
                point = self.route_fit_supp(track, structural)
                if point is not None:
                    nodes.append(('supp', track['channel'], point))
            if not nodes:
                return
            # Forecast where deferred localization will create a clear task.
            virtual = [('future', c, self.tracks[c]['geometry']['mec'][0]) for c in sorted(blocked)]
            route = E11.e5.fast_open_route(self.env.pos, nodes+virtual)
            action = next(n for n in route if n[0] != 'future')
            kind, key, point = action
            self.env.c.record({'kind': 'E13_future_measure', 'deferred_channels': sorted(blocked), 'action': action})
            self.transit_measure(self.env.pos, point, exclude=key if kind != 'cover' else None)
            if kind == 'cover':
                self.scan_site(key, point); remaining = [x for x in remaining if x[0] != key]
                if self.discovery_complete() and len(self.tracks) >= 16:
                    remaining = []
            elif kind == 'clear':
                if key not in self.env.cleared and self.local_clear_from_mec(self.tracks[key]):
                    self.joint_clears += 1
            elif key not in self.env.cleared:
                self.measure(point, key, None, 'E13_nondeferrable_supplement')
                self.stats['supplemental_points'] += 1; self.joint_supp += 1
                self.joint_supp_count[key] = self.joint_supp_count.get(key, 0)+1
                self.maybe_clear_near(self.tracks[key])
            self.joint_steps += 1
        raise RuntimeError('future-measure route guard exceeded')

    def sweep_phase(self, remaining_cover):
        from skeleton_route import choose
        remaining = list(remaining_cover)
        for step in range(120):
            action = choose(self, remaining, macro=False)
            if action is None:
                return
            kind, key, point = action
            self.transit_measure(self.env.pos, point, exclude=key if kind != 'cover' else None)
            if kind == 'cover':
                self.scan_site(key, point); remaining = [x for x in remaining if x[0] != key]
                if self.discovery_complete() and len(self.tracks) >= 16: remaining = []
            elif kind == 'clear':
                if key not in self.env.cleared and self.local_clear_from_mec(self.tracks[key]): self.joint_clears += 1
            elif key not in self.env.cleared:
                self.measure(point, key, None, 'E13_sweep_supplement')
                self.stats['supplemental_points'] += 1; self.joint_supp += 1
                self.joint_supp_count[key] = self.joint_supp_count.get(key, 0)+1
                self.maybe_clear_near(self.tracks[key])
            self.joint_steps += 1
        raise RuntimeError('sweep route guard exceeded')

    def local_clear_from_mec(self, t, Rmax=500):
        c = t['channel']
        if c in self.env.cleared:
            return True
        if t.get('near') is not None:
            return self.env.clear(t['near'], c)
        state = classify(t.get('geometry'), self.env.pos)
        if state['kind'] == 'guaranteed_clear':
            ok = self.env.clear(state['point'], c)
            if not ok:
                self.guaranteed_clear_failures += 1
            return ok
        if self.variant in ('J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R') and state['kind'] == 'risky_clear':
            # Explicit bounded-risk action: R<=80 needs at most q=35,70 after
            # a failed center clear and fresh direction. This is not labelled guaranteed.
            self.risky_clear_tasks += 1
            self.env.c.record({'kind': 'E13_risky_clear', 'channel': c, 'radius_m': t['mec'][1],
                               'maximum_ray_distance_m': 70})
            before = self.env.stats['movement_m']; approach = math.dist(self.env.pos, t['mec'][0])
            result = super().local_clear_from_mec(t, 80)
            self.recovery_movement_m += max(0., self.env.stats['movement_m']-before-approach)
            return result
        # During inherited finish, one bounded local measurement can replace a distant supplement.
        g = t.get('geometry')
        if g and g['status'] == 'bounded' and self.localizations.get(c, 0) < 2:
            p = localization_point(g, self.env.pos)
            if p is not None:
                self.localizations[c] = self.localizations.get(c, 0)+1
                self.policy_localizations += 1
                self.measure(p, c, None, 'E13_localize_before_clear')
                self.maybe_clear_near(t)
                if c in self.env.cleared:
                    return True
                return self.local_clear_from_mec(t, Rmax)
        # Explicit risky recovery, with the frozen finite ray/grid guarantees retained.
        return super().local_clear_from_mec(t, Rmax)


def build_e13(client, variant='A'):
    facade = SimpleNamespace(e9=E11.e9, E11Solver=lambda case: E13Solver(case, variant))
    return build_solver(facade, ClientEnv(client))
