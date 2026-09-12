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
    def __init__(self, case, variant='R'):
        if variant != 'R':
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

    def eligible_targets(self):
        result = []
        for c, t in self.tracks.items():
            if c in self.env.cleared:
                continue
            state = classify(t.get('geometry'), self.env.pos)
            if state['kind'] == 'guaranteed_clear':
                result.append(('clear', c, state['point']))
            elif state['kind'] == 'risky_clear':
                result.append(('clear', c, state['point']))
        return result

    def broad_tracks(self):
        return [t for c, t in self.tracks.items() if c not in self.env.cleared and t['dirs']
                and not self.localized(t) and self.joint_supp_count.get(c, 0) < 3
                and not (t.get('mec') and t['mec'][1] <= 80)]

    def route_fit_supp(self, t, structural):
        g = t.get('geometry')
        if g and g['status'] == 'bounded':
            p = localization_point(g, self.env.pos, allow_wide=True)
            if p is not None and all(math.dist(p, (o['x'], o['y'])) > 1 for o in t['dirs']):
                return p
        return super().route_fit_supp(t, structural)

    def joint_phase(self, remaining_cover):
        from event_route import run
        return run(self, remaining_cover)

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
        if state['kind'] == 'risky_clear':
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

def build_e13(client, variant='R'):
    facade = SimpleNamespace(e9=E11.e9, E11Solver=lambda case: E13Solver(case, variant))
    return build_solver(facade, ClientEnv(client))
