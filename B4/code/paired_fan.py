"""SPF45 points from an actual signal anchor and a certified distance lower bound."""
import math

SAFETY_FACTOR = 1.30
BETA_DEG = 45.
ALPHA_DEG = 1.


def paired_fan(anchor, bearing_deg, distance_lower_m):
    values = (*anchor, bearing_deg, distance_lower_m)
    if len(anchor) != 2 or any(isinstance(x, bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in values):
        raise ValueError('finite fan inputs required')
    if not 0 < distance_lower_m <= 1500:
        raise ValueError('distance lower bound must be positive and <=1500')
    step = SAFETY_FACTOR*distance_lower_m
    points = []
    for sign in (-1,1):
        a = math.radians(bearing_deg+sign*BETA_DEG)
        p = (anchor[0]+step*math.cos(a), anchor[1]+step*math.sin(a))
        if any(abs(x) > 2_000_000 for x in p):
            raise ValueError('fan point outside protocol coordinate range')
        points.append(p)
    return tuple(points)


def theorem_margin():
    limit = min(1/math.cos(math.radians(44)),2*math.cos(math.radians(46)))
    return {'factor':SAFETY_FACTOR,'strict_limit':limit,'ratio_margin':limit-SAFETY_FACTOR}
