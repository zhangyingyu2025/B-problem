"""Fixed directional discovery mesh; not a final performance design."""
import math


def mesh25():
    # A 0.5 m boundary margin avoids relying on rounded tangent equality.
    outer_radius = 1800.5/math.cos(math.radians(15))
    polar = lambda r, a: (r*math.cos(math.radians(a)), r*math.sin(math.radians(a)))
    vertices = [(0., 0.)]
    vertices += [polar(950., 15+30*k) for k in range(12)]
    vertices += [polar(outer_radius, 30*k) for k in range(12)]
    inner = lambda k: 1+k % 12
    outer = lambda k: 13+k % 12
    triangles = []
    for k in range(12):
        triangles += [(0, inner(k), inner(k+1)),
                      (inner(k), outer(k), outer(k+1)),
                      (inner(k), outer(k+1), inner(k+1))]
    return {'name': 'G0-25mesh', 'vertices': vertices, 'triangles': triangles,
            'boundary': list(range(13, 25)), 'target_radius': 1800., 'range': 1000.}
