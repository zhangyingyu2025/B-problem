"""Certified coverage templates. No strategy tuning or simulator access."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoverageTemplate:
    n: int = 6
    rho: float = 1130.0
    rotation_candidates: tuple = tuple(range(0, 60, 5))

    def __post_init__(self):
        if type(self.n) is not int or self.n not in (6, 7):
            raise ValueError('Phase A supports only 6 or 7 outer sites')
        if not math.isfinite(self.rho) or self.rho <= 0:
            raise ValueError('rho must be finite and positive')
        if not isinstance(self.rotation_candidates, tuple) or not self.rotation_candidates:
            raise ValueError('rotation candidates must be a nonempty immutable tuple')
        if any(not math.isfinite(x) or not 0 <= x < 360/self.n for x in self.rotation_candidates):
            raise ValueError('rotations must lie within one symmetry sector')
        if len(set(self.rotation_candidates)) != len(self.rotation_candidates):
            raise ValueError('duplicate rotations')
        if self.n == 6 and (self.rho != 1130 or self.rotation_candidates != tuple(range(0, 60, 5))):
            raise ValueError('six-site fallback is locked to the exact E11 configuration')
        if max(self.endpoint_distances()) > 1000:
            raise ValueError('template does not guarantee reception coverage')

    def endpoint_distances(self):
        return tuple(math.sqrt(r*r+self.rho*self.rho-2*r*self.rho*math.cos(math.pi/self.n))
                     for r in (1000, 1800))

    def outer_sites(self, rotation):
        if rotation not in self.rotation_candidates:
            raise ValueError('rotation not in template candidates')
        return [(self.rho*math.cos(math.radians(rotation+360/self.n*k)),
                 self.rho*math.sin(math.radians(rotation+360/self.n*k))) for k in range(self.n)]

    def certificate(self):
        d0, d1 = self.endpoint_distances()
        return {'n': self.n, 'rho_m': self.rho, 'd1000_m': d0, 'd1800_m': d1,
                'reception_margin_m': 1000-max(d0, d1),
                'open_route_m': self.rho+2*(self.n-1)*self.rho*math.sin(math.pi/self.n)}


E11_TEMPLATE = CoverageTemplate()
SEVEN_TEMPLATE = CoverageTemplate(7, 1005.0, tuple(range(0, 51, 5)))


def default_selector(k0):
    """Pure hook receiving only origin-discovered channel count (including cleared)."""
    if type(k0) is not int or not 0 <= k0 <= 16:
        raise ValueError('invalid origin discovery count')
    return E11_TEMPLATE
