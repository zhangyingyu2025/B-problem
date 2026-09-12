"""G15 DEV: tighten the certified 25-point discovery mesh without changing topology.

The original inner ring radius 950 m had unused edge slack. Moving it inward to
925 m keeps every triangle edge strictly below 1000 m (verified by the exact
certificate verifier) and reduces mandatory route length slightly.
"""
import math
from discovery_mesh import mesh25
from discovery_certificate import verify_mesh
from solver_g14 import G14TransitFullClear


def tight_mesh25(inner_radius=925.0, boundary_margin=0.05):
    outer_radius=(1800.0+boundary_margin)/math.cos(math.radians(15))
    polar=lambda r,a:(r*math.cos(math.radians(a)),r*math.sin(math.radians(a)))
    vertices=[(0.,0.)]
    vertices += [polar(inner_radius,15+30*k) for k in range(12)]
    vertices += [polar(outer_radius,30*k) for k in range(12)]
    inner=lambda k:1+k%12; outer=lambda k:13+k%12
    triangles=[]
    for k in range(12):
        triangles += [(0,inner(k),inner(k+1)),
                      (inner(k),outer(k),outer(k+1)),
                      (inner(k),outer(k+1),inner(k+1))]
    m={'name':'G15-tight-25mesh','vertices':vertices,'triangles':triangles,
       'boundary':list(range(13,25)),'target_radius':1800.,'range':1000.}
    verify_mesh(m)
    return m

class G15TightMesh(G14TransitFullClear):
    def __init__(self,port,*a,**kw):
        if 'mesh' not in kw:kw['mesh']=tight_mesh25()
        super().__init__(port,*a,**kw)

class G15SafeMesh(G14TransitFullClear):
    """Same 925 m inner ring, retaining the original 0.5 m boundary margin."""
    def __init__(self,port,*a,**kw):
        if 'mesh' not in kw: kw['mesh']=tight_mesh25(inner_radius=925.0,boundary_margin=0.5)
        super().__init__(port,*a,**kw)
