"""B4-safe outer geometry: ambiguous negative observations never clip position."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B3/experimental/E13'))
from b1plus import intersection


def geometry(directions, signals):
    return intersection(directions, signals, [])


def lower_distance(point, polygon):
    """Conservative lower bound from a closed convex outer polygon."""
    if not polygon:
        return 0.
    crosses = [(b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])
               for a,b in zip(polygon,polygon[1:]+polygon[:1])]
    if len(polygon) >= 3 and all(x >= -1e-8 for x in crosses):
        return 0.
    d = math.inf
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1]; n=dx*dx+dy*dy
        u=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/n)) if n else 0.
        d=min(d,math.dist(point,(a[0]+u*dx,a[1]+u*dy)))
    return max(0.,d-1e-6)


def fallback_cover(first_direction, polygon=None):
    """28m rectangular cells, half diagonal <20m, in first-bearing coordinates.

    Current K may tighten the bounding box. Cells are retained conservatively;
    none are dropped by an unreliable near-boundary polygon intersection test.
    """
    s=(first_direction['x'],first_direction['y'])
    a=math.radians(first_direction['svd_deg']); u=(math.cos(a),math.sin(a)); v=(-u[1],u[0])
    xmin,xmax=0.,1500.
    ymin,ymax=-1500*math.sin(math.radians(1)),1500*math.sin(math.radians(1))
    if polygon:
        local=[((p[0]-s[0])*u[0]+(p[1]-s[1])*u[1],(p[0]-s[0])*v[0]+(p[1]-s[1])*v[1]) for p in polygon]
        xmin=max(xmin,min(p[0] for p in local)-1e-6); xmax=min(xmax,max(p[0] for p in local)+1e-6)
        ymin=max(ymin,min(p[1] for p in local)-1e-6); ymax=min(ymax,max(p[1] for p in local)+1e-6)
    if xmin>xmax or ymin>ymax:
        return fallback_cover(first_direction)
    nx=max(1,math.ceil((xmax-xmin)/28)); ny=max(1,math.ceil((ymax-ymin)/28))
    points=[]
    for i in range(nx):
        for j in range(ny):
            x=xmin+(i+.5)*(xmax-xmin)/nx; y=ymin+(j+.5)*(ymax-ymin)/ny
            points.append((s[0]+x*u[0]+y*v[0],s[1]+x*u[1]+y*v[1]))
    return points
