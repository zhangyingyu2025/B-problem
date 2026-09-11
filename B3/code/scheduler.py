"""Simple explicit virtual-cost decisions, no mixed precision/travel weights."""
import math


def insertion_distance(current,point,next_site):
    if next_site is None: return math.dist(current,point)
    return math.dist(current,point)+math.dist(point,next_site)-math.dist(current,next_site)


def useful_onroute(track,site):
    if track.cleared or not track.observations or track.clear_point(site) is not None: return False
    if len(track.observations)>=4: return False
    if any(math.dist(site,(o['x'],o['y']))<1e-7 for o in track.observations): return False
    first=track.observations[0];theta=math.radians(first['svd_deg'])
    proxy=(first['x']+750*math.cos(theta),first['y']+750*math.sin(theta))
    # Scheduling heuristic only: reception is not claimed guaranteed here.
    if math.dist(site,proxy)>1500: return False
    new=math.degrees(math.atan2(proxy[1]-site[1],proxy[0]-site[0]))
    separation=abs((new-first['svd_deg']+180)%360-180)
    return separation>=15
