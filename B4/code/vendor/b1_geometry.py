"""Minimal frozen B1 geometry dependency used by the B4 checkpoint.
Reconstructed from jty for sandbox/offline use. Only the two APIs consumed by
B3/experimental/E13/b1plus.py are included: observations_to_halfplanes and
_minimum_circle.
"""
from fractions import Fraction as F
from itertools import combinations
import math
from typing import NamedTuple

class HalfPlane(NamedTuple):
    ax: float
    ay: float
    b: float

def _number(value,label):
    if isinstance(value,bool) or not isinstance(value,(int,float)):
        raise ValueError(f"{label} must be a finite number")
    value=float(value)
    if not math.isfinite(value): raise ValueError(f"{label} must be finite")
    return value

def _unit(deg):
    angle=deg%360.0
    cardinals={0.0:(1.0,0.0),90.0:(0.0,1.0),180.0:(-1.0,0.0),270.0:(0.0,-1.0)}
    if angle in cardinals:return cardinals[angle]
    quadrant=int(angle//90);local=angle-90*quadrant;complement=local>45;base=90-local if complement else local
    c,s=math.cos(math.radians(base)),math.sin(math.radians(base))
    if complement:c,s=s,c
    return ((c,s),(-s,c),(-c,-s),(s,-c))[quadrant]

def observations_to_halfplanes(observations,error_deg=1.0):
    alpha=_number(error_deg,'error_deg')
    if not 0<alpha<90:raise ValueError('error_deg must lie strictly between 0 and 90 degrees')
    if not isinstance(observations,(list,tuple)) or not observations:raise ValueError('observations must be a nonempty list for one source')
    planes,unique,seen,duplicates=[],[],set(),[]
    for i,record in enumerate(observations):
        if not isinstance(record,dict) or set(record)!={'x','y','svd_deg'}:
            raise ValueError(f'observation {i}: require exactly x, y, svd_deg (direction readings only)')
        x,y,theta=(_number(record[k],f'observation {i}.{k}') for k in ('x','y','svd_deg'))
        if not 0<=theta<360:raise ValueError(f'observation {i}: svd_deg must be in [0,360)')
        key=(x,y,theta)
        if key in seen:duplicates.append(i);continue
        seen.add(key);unique.append({'x':x,'y':y,'svd_deg':theta})
        lower,upper=_unit(theta-alpha),_unit(theta+alpha)
        for a,b in ((lower[1],-lower[0]),(-upper[1],upper[0])):
            c=a*x+b*y
            if not math.isfinite(c):raise ValueError('coordinate/normal product overflow')
            planes.append(HalfPlane(a,b,c))
    return planes,unique,duplicates

def _dot(a,p):return a[0]*p[0]+a[1]*p[1]
def _squared_distance(a,b):return (a[0]-b[0])**2+(a[1]-b[1])**2

def _circle_for_three(a,b,c):
    u,v=(b[0]-a[0],b[1]-a[1]),(c[0]-a[0],c[1]-a[1])
    det=2*(u[0]*v[1]-u[1]*v[0])
    if det==0:return None
    uu,vv=_dot(u,u),_dot(v,v)
    offset=((uu*v[1]-u[1]*vv)/det,(u[0]*vv-uu*v[0])/det)
    center=(a[0]+offset[0],a[1]+offset[1])
    return center,_dot(offset,offset)

def _minimum_circle(vertices):
    best=None
    def consider(center,radius2,support):
        nonlocal best
        if best is not None and radius2>=best[1]:return
        if all(_squared_distance(p,center)<=radius2 for p in vertices):best=(center,radius2,list(support))
    for i,p in enumerate(vertices):consider(p,F(0),(i,))
    for i,j in combinations(range(len(vertices)),2):
        a,b=vertices[i],vertices[j];center=((a[0]+b[0])/2,(a[1]+b[1])/2)
        consider(center,_squared_distance(a,b)/4,(i,j))
    for i,j,k in combinations(range(len(vertices)),3):
        cand=_circle_for_three(vertices[i],vertices[j],vertices[k])
        if cand is not None:consider(*cand,(i,j,k))
    assert best is not None
    return best
