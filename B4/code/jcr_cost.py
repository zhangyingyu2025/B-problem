"""G41 pure certificate/resolver costs in meters; no environment or truth access.

The objective is a conditional completion upper-bound surrogate, NOT a lower bound
on G40, nor a theorem that the difference of two upper bounds is realized savings.
"""
from dataclasses import dataclass
import math

from solver_g35 import (ALPHA, MAX_SOURCE_RANGE, STRIP_CLEAR_R, safe_rho,
                        _bracket_side, _local, chord_radial_upper,
                        _active_strip_centers)


def action_eq(action, channel=None, current_channel=None):
    if action=='measure':
        # Unknown future channel: 5 is a worst-case bound, not a predicted switch.
        return 25.+(0. if channel is not None and channel==current_channel else 5.)
    if action=='clear_success': return 25.
    if action=='clear_fail': return 15.
    raise ValueError('unknown action')


@dataclass(frozen=True)
class SafeSiteInfo:
    point: tuple
    safe: bool
    rho_m: float
    side: int
    predicted_dir_clear_count: object
    extra_measure_eq_m: float
    gamma_lower_rad: float
    strip_half_length_m: float
    dir_completion_eq_m: float
    resume_switch_eq_m: float


def safe_site_info(obs,point,channel=None,current_channel=None,already_committed_measure=False):
    point=tuple(point);rho=safe_rho(obs,point);safe=rho<=1000.
    extra=0. if already_committed_measure else action_eq('measure',channel,current_channel)
    count=None;gamma=0.;S=math.inf;completion=math.inf
    if safe:
        # Every first-sector candidate has |local y| <= H and |g-q| <= rho.
        # Thus acute angle(true q->g, first centerline) >= asin((|qy|-H)/rho).
        # A second bounded-error reading can decrease this by at most alpha.
        _,y=_local(obs,point);H=1500*math.sin(ALPHA)
        gamma=max(0.,math.asin(min(1.,max(0.,abs(y)-H)/rho))-ALPHA)
        w0=H+2e-5;w1=(rho+3e-6)*math.sin(ALPHA)+2e-5
        if gamma>=math.radians(3.)+1e-10 and w1<STRIP_CLEAR_R:
            S=(w0+w1*math.cos(gamma))/math.sin(gamma)
            h=math.sqrt(STRIP_CLEAR_R**2-w1*w1)
            count=max(1,math.ceil(S/h))
            # Sweep axis passes through q. Every possible source has axial
            # coordinate <=rho in magnitude. Centerline intersection differs
            # axially by <=S; a closed excursion covers the interval in <=2rho+4S.
            completion=2*rho+4*S+15*(count-1)+25.
    # Restoring the pre-probe measurement channel costs at most one more switch.
    # This is essential if the baseline would keep that channel for the next action.
    # We charge the virtual restoration as an upper bound; no artificial measure is
    # sent to implement it, and the actual next action may avoid that switch entirely.
    restore=0. if already_committed_measure or (channel is not None and channel==current_channel) else 5.
    return SafeSiteInfo(point,safe,rho,_bracket_side(obs,point) if safe else 0,
                        count,extra,gamma,S,completion,restore)


def radial_centers(obs,radius,here):
    """Finite cover of [0,R] x [-R sin(alpha),R sin(alpha)] in first coordinates."""
    if not 0<radius<=1500: return None
    w=radius*math.sin(ALPHA)+2e-5
    if w>=STRIP_CLEAR_R:return None
    h=math.sqrt(STRIP_CLEAR_R**2-w*w);n=max(1,math.ceil(radius/(2*h)))
    a=math.radians(obs['svd_deg']);u=(math.cos(a),math.sin(a));o=(obs['x'],obs['y'])
    pts=[(o[0]+(i+.5)*radius/n*u[0],o[1]+(i+.5)*radius/n*u[1]) for i in range(n)]
    if math.dist(here,pts[-1])<math.dist(here,pts[0]):pts.reverse()
    length=math.dist(here,pts[0])+sum(math.dist(p,q) for p,q in zip(pts,pts[1:]))+math.dist(pts[-1],here)
    return {'centers':pts,'count':n,'closed_movement_m':length,'worst_eq_m':length+15*(n-1)+25.}


@dataclass(frozen=True)
class ResolverPairInfo:
    site_i: int
    site_j: int
    q_plus: tuple
    q_minus: tuple
    chord_radius_upper_m: float
    dir_branch_clear_i: int
    dir_branch_clear_j: int
    nosignal_branch_clear_count: int
    worst_case_eq_m: float
    branch_costs_eq_m: tuple
    first: SafeSiteInfo
    second: SafeSiteInfo


def resolver_pair(obs,site_i,first,site_j,second):
    if not(first.safe and second.safe and first.side*second.side==-1):return None
    if first.predicted_dir_clear_count is None or second.predicted_dir_clear_count is None:return None
    up,lo=(first.point,second.point) if first.side==1 else (second.point,first.point)
    radius=chord_radial_upper(obs,up,lo)
    if radius is None:return None
    radius=min(1500.,radius+2e-6)
    radial=radial_centers(obs,radius,second.point)
    if radial is None:return None
    mi=first.extra_measure_eq_m+first.resume_switch_eq_m
    mj=second.extra_measure_eq_m+second.resume_switch_eq_m
    branches=(mi+first.dir_completion_eq_m,mi+mj+second.dir_completion_eq_m,mi+mj+radial['worst_eq_m'])
    return ResolverPairInfo(site_i,site_j,up,lo,radius,first.predicted_dir_clear_count,
                            second.predicted_dir_clear_count,radial['count'],max(branches),branches,first,second)


def direction_plan(obs,info,body,here):
    second={'x':info.point[0],'y':info.point[1],'svd_deg':body['svd_deg']}
    z=_active_strip_centers(obs,1500.,second,info.rho_m+3e-6,here,
                            max_clears=info.predicted_dir_clear_count or 0)
    if z is None:return None
    z=dict(z);z['closed_movement_m']=z['movement_m']+math.dist(z['centers'][-1],here)
    z['worst_eq_m']=z['closed_movement_m']+15*(z['count']-1)+25.
    return z


def active_resolver_worst_case_eq_m(track,current_position,fan_limit=8):
    """Loose bound for standalone unchanged G40 completion under its invariants.

    All source actions stay within 3500m of first anchor: source <=1500 from O,
    every signal anchor <=1500 from source; SPF stays in range; center/guaranteed
    clears stay close to source; G40 strip excursion <=500m. Full-wedge fallback
    uses <=108 cells. At most F fans (2 readings each), F+1 anchor attempts,
    one center probe, and a terminal strip (<=20) or grid (<=108) sweep.
    Future interleaved unrelated-source travel is excluded, just as from Cpair.
    """
    if track.near is not None:return math.dist(current_position,track.near)+25.
    if track.mec and track.mec[1]<=20-1e-6:return math.dist(current_position,track.mec[0])+25.
    if not track.dirs:return math.inf
    o=(track.dirs[0]['x'],track.dirs[0]['y']);F=max(0,fan_limit-track.fan_steps)
    actions=3*F+2+108
    return math.dist(current_position,o)+3500.+7000.*(actions-1)+30.*(2*F+1)+15.*(F+1+108)+10.


def should_defer(task_kind,discovered_count,pair,active_cost,remaining):
    return (task_kind=='fan' and discovered_count<16 and pair is not None
            and pair.site_i in remaining and pair.site_j in remaining
            and pair.worst_case_eq_m+1e-8<active_cost)


def route_length(start,order,points):
    if not order:return 0.
    return math.dist(start,points[order[0]])+sum(math.dist(points[i],points[j]) for i,j in zip(order,order[1:]))


def objective(start,order,points,active_costs,pairs,discovered_count=0):
    rank={s:i for i,s in enumerate(order)};cost=route_length(start,order,points);selected={}
    for c,active in active_costs.items():
        valid=[] if discovered_count>=16 else [p for p in pairs.get(c,()) if p.site_i in rank and p.site_j in rank and rank[p.site_i]<rank[p.site_j]]
        best=min(valid,key=lambda p:(p.worst_case_eq_m,p.site_i,p.site_j)) if valid else None
        if best is not None and best.worst_case_eq_m+1e-8<active:
            cost+=best.worst_case_eq_m;selected[c]=best
        else:cost+=active
    return cost,selected


def local_search(start,initial,points,active_costs,pairs,rounds=4,max_evaluations=4000):
    order=list(initial);score,chosen=objective(start,order,points,active_costs,pairs);evaluations=1
    for _ in range(rounds):
        best=(score,order,chosen);visited={tuple(order)};n=len(order)
        def neighbors():
            for i in range(n):
                for j in range(i+1,n):yield order[:i]+order[i:j+1][::-1]+order[j+1:]
            for i in range(n):
                rest=order[:i]+order[i+1:]
                for j in range(n):yield rest[:j]+[order[i]]+rest[j:]
            for i in range(n):
                for j in range(i+1,n):
                    z=order[:];z[i],z[j]=z[j],z[i];yield z
        for z in neighbors():
            if tuple(z) in visited:continue
            if evaluations>=max_evaluations:break
            visited.add(tuple(z));value,sel=objective(start,z,points,active_costs,pairs);evaluations+=1
            if value<best[0]-1e-8:best=(value,z,sel)
        if best[0]>=score-1e-8:break
        score,order,chosen=best
        if evaluations>=max_evaluations:break
    return {'order':order,'score':score,'selected':chosen,'evaluations':evaluations}
