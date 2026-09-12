"""Separate proved clear opportunities from uncertain localization tasks."""
import math


def classify(geometry, current):
    if not geometry or geometry.get('status') != 'bounded':
        return {'kind': 'needs_localization', 'point': None, 'risk_m': math.inf}
    center, radius = geometry['mec']
    current_radius = max(math.dist(current, p) for p in geometry['polygon'])
    if current_radius <= 20-1e-6:
        return {'kind': 'guaranteed_clear', 'point': current, 'risk_m': 0}
    if radius <= 20-1e-6:
        return {'kind': 'guaranteed_clear', 'point': center, 'risk_m': 0}
    return {'kind': 'risky_clear' if radius <= 80 else 'needs_localization',
            'point': center, 'risk_m': radius+geometry['diameter']/2}


def localization_point(geometry, current, allow_wide=False):
    """A nearby perpendicular bearing intersection, never a blind center clear.

    If R<=500 and offset<=120, distance to every feasible source<=620<1000.
    Actual near responses remain valid and are cleared immediately.
    """
    center, radius = geometry['mec']
    if radius > 500 and not allow_wide:
        return None
    a, b = geometry['axis_ends']; delta = (b[0]-a[0], b[1]-a[1]); length = math.hypot(*delta)
    if length < 1e-9:
        return center
    side = (-delta[1]/length, delta[0]/length)
    offset = min(120., max(40., radius*.35))
    choices = [(center[0]+sign*offset*side[0], center[1]+sign*offset*side[1]) for sign in (-1, 1)]
    if allow_wide:
        # The whole computed outer polygon must be in guaranteed reception range.
        choices = [p for p in choices if max(math.dist(p, v) for v in geometry['polygon']) <= 1000-1e-6]
        if not choices:
            return None
    return min(choices, key=lambda p: math.dist(current, p))
