"""G5 DEV variants: FIM-inspired opportunity ranking for bearing measurements.

For a target proxy at range r and bearing-line crossing angle phi, bearing-only
information scales with sin(phi)^2 / r^2. We use that only as a *ranking heuristic*;
all correctness remains set-membership G0/G2/G4.
"""
import math
from solver_g2 import line_angle_deg
from solver_g4 import G4TransitSolver

class _G5InfoBase(G4TransitSolver):
    range_power=1.0
    distance_floor=350.0
    max_proxy_distance=1050.0
    def __init__(self,*a,**kw):
        # Bypass G4's hard-coded 1000 by calling then replacing knob.
        super().__init__(*a,**kw)
        self.max_proxy_distance=self.__class__.max_proxy_distance
    def opportunity_score(self,c,p):
        t=self.tracks[c]
        if c in self.cleared or t.near is not None or not t.dirs or self._already_sampled(t,p):return None
        proxy=self._proxy(t)
        if proxy is None:return None
        dist=math.dist(p,proxy)
        if dist>self.max_proxy_distance:return None
        cand=math.degrees(math.atan2(proxy[1]-p[1],proxy[0]-p[0]))
        sep=max((line_angle_deg(cand,math.degrees(math.atan2(proxy[1]-o['y'],proxy[0]-o['x']))) for o in t.dirs),default=0.)
        if sep<12.:return None
        geom=math.sin(math.radians(sep))**2
        rf=(1000./max(self.distance_floor,dist))**self.range_power
        radius=t.mec[1] if t.mec else 1000.
        urgency=0.35+0.65*min(radius,300.)/300.
        return 100.*geom*rf*urgency

class G5Info1(_G5InfoBase): range_power=1.0
class G5Info15(_G5InfoBase): range_power=1.5
class G5Info2(_G5InfoBase): range_power=2.0
class G5Info2R1000(G5Info2): max_proxy_distance=1000.
class G5Info15R1000(G5Info15): max_proxy_distance=1000.
