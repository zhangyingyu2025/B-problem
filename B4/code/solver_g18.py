"""G18 DEV: replace the 25-site short-edge triangulation discovery mesh with
a 21-site local-convex directional discovery certificate.

Correctness condition: for every possible source g in the 1800 m target disk, g lies
in the convex hull of tested sites guaranteed within 1000 m of g. Then every closed
emitting half-plane through g contains at least one such site. Absence is concluded
only after all 21 sites returned no_signal for the channel. Localization/clear/fan and
finite fallback are inherited unchanged from G15/G14.
"""
from discovery_convex_certificate import mesh21
from solver_g14 import G14TransitFullClear

class G18Convex21(G14TransitFullClear):
    def __init__(self,port,*a,**kw):
        if 'mesh' not in kw: kw['mesh']=mesh21()
        super().__init__(port,*a,**kw)

class G18P8(G18Convex21):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw); self.opportunity_per_site=8

class G18T6(G18Convex21):
    transit_per_leg=6

class G18P8T6(G18P8):
    transit_per_leg=6

class G18Wide(G18P8T6):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw); self.max_proxy_distance=1100.; self.min_line_angle=12.

class G18Compact21(G18Convex21):
    def __init__(self,port,*a,**kw):
        if 'mesh' not in kw: kw['mesh']=mesh21(outer_radius=1864.0,inner_radius=992.5)
        super().__init__(port,*a,**kw)

class G18CompactT6(G18Compact21):
    transit_per_leg=6
class G18T8(G18Convex21):
    transit_per_leg=8

class _G18ParamT6(G18Convex21):
    transit_per_leg=6
    outer_radius=1868.7
    inner_radius=999.9
    def __init__(self,port,*a,**kw):
        if 'mesh' not in kw: kw['mesh']=mesh21(self.outer_radius,self.inner_radius)
        super().__init__(port,*a,**kw)
class G18A(_G18ParamT6): outer_radius=1866.0; inner_radius=995.0
class G18B(_G18ParamT6): outer_radius=1868.0; inner_radius=997.5
class G18D(_G18ParamT6): outer_radius=1872.0; inner_radius=1000.0
class G18B8(G18B): transit_per_leg=8

class _G18RouteGuard(G18B8):
    detour_limit=250.0
    def select_task(self,nodes):
        import math
        from solver_g2 import better_open_route
        covers=[t for t in nodes if t[0]=='cover']
        if not covers:return super().select_task(nodes)
        cover=better_open_route(self.port.position,covers)[0]
        a=self.port.position;b=cover[2];direct=math.dist(a,b)
        eligible=[]
        for t in nodes:
            if t[0]=='cover':continue
            det=math.dist(a,t[2])+math.dist(t[2],b)-direct
            if det<=self.detour_limit:eligible.append((det,math.dist(a,t[2]),t))
        return min(eligible,key=lambda x:(x[0],x[1]))[2] if eligible else cover
class G18R100(_G18RouteGuard): detour_limit=100.
class G18R250(_G18RouteGuard): detour_limit=250.
class G18R500(_G18RouteGuard): detour_limit=500.
