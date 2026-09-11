"""Offline-only coverage extension of the frozen E11 engine; no copied scheduler."""
import math
from types import SimpleNamespace
from coverage_templates import CoverageTemplate, E11_TEMPLATE, default_selector
from run_e11_rehearsal import load_e11, build_solver, ClientEnv

E11 = load_e11()


class E12Solver(E11.E11Solver):
    def __init__(self, case, selector=default_selector):
        super().__init__(case)
        self.selector = selector
        self.coverage_template = E11_TEMPLATE
        self.k0 = None

    def choose_rotation(self):
        # Inherited run_search calls this after the origin scan, before any outer action.
        self.k0 = len(self.tracks)
        template = self.selector(self.k0)
        if not isinstance(template, CoverageTemplate):
            raise TypeError('selector must return a validated CoverageTemplate')
        self.coverage_template = template
        if template.n == 6:
            return super().choose_rotation()
        proxies = []
        for track in self.tracks.values():
            if not track['dirs']:
                continue
            o = track['dirs'][0]
            a = math.radians(o['svd_deg'])
            proxies.append((o['x']+self.proxy_r*math.cos(a), o['y']+self.proxy_r*math.sin(a)))
        best = None
        for deg in template.rotation_candidates:
            outer = template.outer_sites(deg)
            nodes = [('cover', k, p) for k, p in enumerate(outer)]+[('proxy', i, p) for i, p in enumerate(proxies)]
            cost = E11.e5.route_cost((0., 0.), E11.e5.fast_open_route((0., 0.), nodes))
            if best is None or cost < best[0]:
                best = (cost, deg, outer)
        self.rotation = best[1]
        return best[2]

    def scan_site(self, idx, site):
        if self.coverage_template.n == 6:
            return super().scan_site(idx, site)
        # Same scan ordering; evidence must include all n+1 stations, not hardcoded 7.
        unknown = sorted([c for c in range(1, 21) if c not in self.tracks and
                          len(self.checked[c]) < len(self.sites)], reverse=bool(idx % 2))
        if self.env.channel in unknown:
            unknown.remove(self.env.channel)
            unknown.insert(0, self.env.channel)
        for c in unknown:
            if self.discovery_complete():
                break
            self.measure(site, c, idx, 'discovery')
            if c in self.tracks:
                self.maybe_clear_near(self.tracks[c])


def build_offline_solver(client, selector=None, baseline=False):
    """Reuse the E11 adapter injection, with no access to hidden source truth."""
    env = ClientEnv(client)
    if baseline:
        return build_solver(E11, env)
    facade = SimpleNamespace(e9=E11.e9,
                             E11Solver=lambda case: E12Solver(case, selector or default_selector))
    return build_solver(facade, env)
