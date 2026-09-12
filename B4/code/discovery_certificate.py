"""Rational verifier for a triangulated directional discovery certificate.

The provided binary coordinates are treated exactly. Every triangle edge is
shorter than 1000 m; the convex boundary encloses the entire target disk.
"""
from collections import defaultdict
from fractions import Fraction as F
from itertools import combinations
import math


def cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def norm2(a, b):
    return (a[0]-b[0])**2+(a[1]-b[1])**2


def on_segment(a, b, p):
    return cross(a,b,p) == 0 and all(min(a[k],b[k]) <= p[k] <= max(a[k],b[k]) for k in (0,1))


def segments_intersect(a, b, c, d):
    x, y, z, w = cross(a,b,c), cross(a,b,d), cross(c,d,a), cross(c,d,b)
    return (x*y < 0 and z*w < 0) or any((v == 0 and on_segment(p,q,r)) for v,p,q,r in
            ((x,a,b,c),(y,a,b,d),(z,c,d,a),(w,c,d,b)))


def verify_mesh(mesh):
    v = [tuple(F(x) for x in p) for p in mesh['vertices']]
    t, boundary = mesh['triangles'], mesh['boundary']
    n = len(v); radius, reach = F(mesh['target_radius']), F(mesh['range'])
    if n < 3 or len(set(v)) != n or len(boundary) < 3 or len(set(boundary)) != len(boundary):
        raise ValueError('invalid vertices or boundary')
    if any(len(p) != 2 for p in v) or any(not 0 <= i < n for i in boundary):
        raise ValueError('invalid coordinate or boundary index')
    bedges = list(zip(boundary, boundary[1:]+boundary[:1]))
    area = F(0); margins = []
    for a,b in bedges:
        h = cross(v[a],v[b],(F(0),F(0)))
        if h <= 0 or h*h < radius*radius*norm2(v[a],v[b]):
            raise ValueError('boundary fails to contain target circle')
        if any(cross(v[a],v[b],p) < 0 for p in v):
            raise ValueError('vertices outside convex boundary')
        area += v[a][0]*v[b][1]-v[a][1]*v[b][0]
        margins.append(float(h)/math.sqrt(float(norm2(v[a],v[b])))-float(radius))
    incidences = defaultdict(list); used = set(); tri_area = F(0); longest = F(0)
    for index, tri in enumerate(t):
        if len(tri) != 3 or len(set(tri)) != 3 or any(not 0 <= i < n for i in tri):
            raise ValueError('invalid triangle')
        a,b,c = tri; aa = cross(v[a],v[b],v[c])
        if aa <= 0:
            raise ValueError('flipped or degenerate triangle')
        tri_area += aa; used.update(tri)
        for u,w in ((a,b),(b,c),(c,a)):
            d2 = norm2(v[u],v[w]); longest = max(longest,d2)
            if d2 >= reach*reach:
                raise ValueError('triangle edge not strictly shorter than range')
            incidences[tuple(sorted((u,w)))].append((u,w,index))
    if used != set(range(n)) or tri_area != area:
        raise ValueError('unused vertex or triangle area mismatch')
    boundary_set = set(bedges)
    adjacency = [set() for _ in t]
    for edge, occurrences in incidences.items():
        if len(occurrences) == 1:
            if occurrences[0][:2] not in boundary_set:
                raise ValueError('hole or invalid boundary edge')
        elif len(occurrences) == 2:
            a,b = occurrences
            if a[:2] != b[1::-1] or a[:2] in boundary_set or b[:2] in boundary_set:
                raise ValueError('inconsistent triangle gluing')
            adjacency[a[2]].add(b[2]); adjacency[b[2]].add(a[2])
        else:
            raise ValueError('non-manifold edge')
    if any(tuple(sorted(e)) not in incidences for e in bedges):
        raise ValueError('missing boundary edge')
    reached = {0}; todo = [0]
    while todo:
        for i in adjacency[todo.pop()]-reached:
            reached.add(i); todo.append(i)
    if len(reached) != len(t):
        raise ValueError('disconnected triangulation')
    for e,f in combinations(incidences,2):
        shared = set(e)&set(f)
        if not shared:
            if segments_intersect(v[e[0]],v[e[1]],v[f[0]],v[f[1]]):
                raise ValueError('crossing or touching nonincident edges')
        else:
            s = shared.pop(); a = next(i for i in e if i != s); b = next(i for i in f if i != s)
            if on_segment(v[s],v[a],v[b]) or on_segment(v[s],v[b],v[a]):
                raise ValueError('overlapping incident edges')
    return {'valid': True, 'vertices': n, 'triangles': len(t),
            'max_edge_m': math.sqrt(float(longest)),
            'minimum_boundary_margin_m': min(margins),
            'proof': 'nonoverlapping oriented triangulation; exact edge and disk containment inequalities',
            'numeric_domain': 'exact rational arithmetic on stored binary vertex coordinates'}


def absent_certified(measured_site_ids, mesh):
    # Caller must verify the static mesh and record only actual accepted
    # no_signal observations for an undiscovered channel at these exact sites.
    return set(range(len(mesh['vertices']))) <= set(measured_site_ids)
