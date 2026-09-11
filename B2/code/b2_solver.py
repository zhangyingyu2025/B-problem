"""B2 deterministic, conservative minimax placement; Python standard library.

The upper/lower envelopes are mathematical set-containment bounds evaluated
in floating point. Decimal rechecks are numerical verification, not directed
interval certificates and not continuous-placement global optimality proofs.
"""
import math
from domains import Problem, finite
from geometry import GeometryError, metrics, precise_metrics


def evaluate_candidate(problem, point, max_intervals=192, abs_tolerance_m=1.0,
                       rel_tolerance=0.002, high_precision=True):
    p=tuple(finite(v,'candidate') for v in point)
    if len(p)!=2: raise ValueError('candidate requires two coordinates')
    if isinstance(max_intervals,bool) or not isinstance(max_intervals,int) or max_intervals<4:
        raise ValueError('max_intervals must be an integer >=4')
    abs_tolerance_m=finite(abs_tolerance_m,'abs_tolerance_m')
    rel_tolerance=finite(rel_tolerance,'rel_tolerance')
    if abs_tolerance_m<=0 or rel_tolerance<0: raise ValueError('invalid tolerances')
    base={"point_local_m":p,"point_world_m":problem.to_world(p),"movement_m":math.hypot(*p)}
    if problem.status!='ready': return dict(base,status=problem.status)
    if not problem.effective_domain(p):
        return dict(base,status='outside_implemented_effective_domain',
                    analytic_safe=problem.analytic_safe(p),
                    note='not a proof that the point lies outside complete C_safe or complete effective domain')
    interval=problem.bearing_interval(p)
    low,high=interval['observed_deg'];cache={};leaves=[]

    def sample(theta):
        if theta not in cache: cache[theta]=metrics(p,theta,problem.alpha)
        return cache[theta]

    def leaf(lo,hi):
        for angle in (lo,(lo+hi)/2,hi): sample(angle)
        half=problem.alpha+(hi-lo)/2
        if half>=90: raise GeometryError('envelope wedge half-width >=90 degrees')
        value=metrics(p,(lo+hi)/2,problem.alpha,half)
        return {"lo":lo,"hi":hi,"diameter_m":value['diameter_m'],"mec_radius_m":value['mec_radius_m']}

    keys=('diameter_m','mec_radius_m')
    try:
        start=min(8,max_intervals)
        grid=[low+(high-low)*i/start for i in range(start+1)]
        leaves=[leaf(a,b) for a,b in zip(grid,grid[1:])]
        while True:
            witnesses={k:max(cache,key=lambda theta:cache[theta][k]) for k in keys}
            lower={k:cache[witnesses[k]][k] for k in keys}
            upper={k:max(item[k] for item in leaves) for k in keys}
            tolerances={k:max(abs_tolerance_m,rel_tolerance*lower[k]) for k in keys}
            if any(upper[k]<lower[k]-1e-7*max(1,upper[k]) for k in keys):
                raise GeometryError('envelope upper bound below sample lower bound')
            if all(upper[k]-lower[k]<=tolerances[k] for k in keys):
                status='tolerance_reached_float';break
            if len(leaves)>=max_intervals:
                status='interval_budget_reached';break
            index=max(range(len(leaves)),key=lambda i:max((leaves[i][k]-lower[k])/tolerances[k] for k in keys))
            item=leaves.pop(index);mid=(item['lo']+item['hi'])/2
            leaves.extend((leaf(item['lo'],mid),leaf(mid,item['hi'])))
        output=dict(base,status=status,source_interval=interval,
                    evaluation_count=len(cache),interval_count=len(leaves),
                    bound_semantics='float evaluation of analytical H envelopes, not rigorous interval arithmetic',
                    metric_scope='H contains K; bounds bracket J_H, only upper bounds transfer to J_K',
                    high_precision_check={"status":"not_requested"})
        for k,label in [('diameter_m','worst_diameter'),('mec_radius_m','worst_mec_radius')]:
            theta=witnesses[k];value=cache[theta]
            output[label]={"sample_lower_m":lower[k],"envelope_upper_m":upper[k],
                           "gap_m":max(0,upper[k]-lower[k]),"tolerance_m":tolerances[k],
                           "worst_sample_observation_local_deg":theta,
                           "worst_sample_observation_world_deg":(theta+problem.theta)%360,
                           "physical_witness":problem.witness(p,theta,interval),
                           "sample_geometry_local":value}
        # Supremum radius <=20 implies every future region can individually
        # fit in a 20 m disk. Centers may depend on the future observation.
        radius=output['worst_mec_radius']
        threshold=20.0
        output['clearance_20m_assessment']=(
            'supported_by_conservative_float_upper' if radius['envelope_upper_m']<threshold-1e-7 else
            'counterexample_observation_found' if radius['sample_lower_m']>threshold+1e-7 and radius['physical_witness'] else
            'unresolved')
        output['clearance_note']='one center chosen AFTER the second reading; no shared center for all readings; numerical assessment only'
        if high_precision:
            checks=[]
            requests={(witnesses[k],problem.alpha,'worst_sample') for k in keys}
            for k in keys:
                item=max(leaves,key=lambda x:x[k])
                requests.add(((item['lo']+item['hi'])/2,problem.alpha+(item['hi']-item['lo'])/2,'upper_envelope'))
            for theta,half,kind in sorted(requests):
                reference=precise_metrics(p,theta,problem.alpha,half)
                fast=metrics(p,theta,problem.alpha,half)
                if reference['status']!='bounded': raise GeometryError('high precision region classification differs')
                deltas={k:abs(reference[k]-fast[k]) for k in keys}
                if any(deltas[k]>1e-7+1e-9*max(1,reference[k]) for k in keys):
                    raise GeometryError('float and 65-digit metric discrepancy')
                checks.append({"kind":kind,"theta_local_deg":theta,"second_half_width_deg":half,
                               "absolute_differences_m":deltas,"reference":reference})
            output['high_precision_check']={"status":"passed","digits":65,"checks":checks,
                                            "scope":"selected worst samples and active upper envelopes, not all angles"}
        return output
    except (ArithmeticError,OverflowError) as exc:
        return dict(base,status='numerically_uncertain',diagnostic=str(exc),source_interval=interval,
                    worst_diameter=None,worst_mec_radius=None)


def solve_b2(observation, error_deg=1.0, *, options=None, physical=None):
    options={} if options is None else dict(options)
    physical={} if physical is None else dict(physical)
    allowed={'grid_step_m','refinement_levels','max_candidates','max_intervals','final_intervals',
             'abs_tolerance_m','rel_tolerance','near_optimal_fraction','candidate_points_local'}
    if set(options)-allowed: raise ValueError('unknown options: '+str(sorted(set(options)-allowed)))
    problem=Problem(observation,error_deg,**physical)
    step=finite(options.get('grid_step_m',problem.rmin/10),'grid_step_m')
    eta=finite(options.get('near_optimal_fraction',0.05),'near_optimal_fraction')
    if step<=0 or not 0<=eta<=1: raise ValueError('invalid grid step or near-optimal fraction')
    def integer(name,default,minimum):
        x=options.get(name,default)
        if isinstance(x,bool) or not isinstance(x,int) or x<minimum: raise ValueError(name+' invalid')
        return x
    levels=integer('refinement_levels',2,0);budget=integer('max_candidates',180,1)
    intervals=integer('max_intervals',192,4);final_intervals=integer('final_intervals',768,4)
    atol=finite(options.get('abs_tolerance_m',1.0),'abs_tolerance_m')
    rtol=finite(options.get('rel_tolerance',0.002),'rel_tolerance')
    if atol<=0 or rtol<0: raise ValueError('invalid tolerances')
    result={"schema_version":1,"status":problem.status,"observation":observation,
            "error_deg":error_deg,"physical_parameters":{"domain_center":problem.domain_center,
                "domain_radius":problem.radius,"min_range":problem.rmin,"max_range":problem.rmax,
                "near_range":problem.near,"angle_margin_deg":problem.margin},
            "objective_priority":["worst localization diameter","movement for numerical ties only"],
            "auxiliary_metric":"worst minimum enclosing circle radius; not a replacement primary metric",
            "domains":problem.domain_descriptions(),"candidates":[],"recommended":None,
            "search":{"grid_step_m":step,"refinement_levels":levels,"max_candidates":budget,
                      "max_intervals":intervals,"final_intervals":final_intervals,
                      "abs_tolerance_m":atol,"rel_tolerance":rtol,"near_optimal_fraction":eta},
            "limitations":["not a global continuous placement optimum",
                           "H outer approximation includes the first near disk",
                           "float envelopes and pointwise Decimal rechecks are not interval certificates",
                           "full C_safe boundary is implicit, not computed"]}
    if problem.status!='ready': return result
    evaluated={};truncated=False
    def valid(item): return item.get('worst_diameter') is not None and 'worst_diameter' in item
    def rank(item):
        # R_min is reported independently and never silently replaces D.
        return (item['worst_diameter']['envelope_upper_m'],item['movement_m'],item['point_local_m'])
    def add(p):
        nonlocal truncated
        p=tuple(float(v) for v in p)
        key=tuple(round(v,10) for v in p)
        if key in evaluated or not problem.effective_domain(p): return
        if len(evaluated)>=budget: truncated=True;return
        evaluated[key]=evaluate_candidate(problem,p,intervals,atol,rtol,False)
    supplied=options.get('candidate_points_local')
    if supplied is not None:
        if not isinstance(supplied,list) or not supplied: raise ValueError('candidate_points_local must be a nonempty list')
        for p in supplied:
            if not isinstance(p,(list,tuple)) or len(p)!=2: raise ValueError('invalid candidate point')
            p=tuple(finite(v,'candidate coordinate') for v in p)
            add(p)
    else:
        if math.ceil(problem.rmin/step)>1000: raise ValueError('grid too fine; use refinement or explicit candidates')
        count=math.ceil(problem.rmin/step)
        # Both sides evaluated before moving to the next a/b position.
        for i in range(1,count+1):
            for j in range(1,count+1):
                for sign in (-1,1): add((i*step,sign*j*step))
                if truncated: break
            if truncated: break
    for level in range(levels):
        good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
        leaders=[]
        # Preserve both lobes; D0 clipping may break their symmetry.
        for sign in (-1,1): leaders += [v for v in good if v['point_local_m'][1]*sign>0][:2]
        local_step=step/(2**(level+1))
        for leader in leaders:
            a,b=leader['point_local_m']
            for dx in (-local_step,0,local_step):
                for dy in (-local_step,0,local_step): add((a+dx,b+dy))
    good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
    if not good:
        result.update(status='no_candidate_in_search',candidates=list(evaluated.values()))
        return result
    # Re-evaluate leading candidates at the final budget. Check the eventual
    # winner even if its refined score changes the ordering.
    checked=set()
    for item in good[:min(6,len(good))]:
        key=tuple(round(v,10) for v in item['point_local_m'])
        evaluated[key]=evaluate_candidate(problem,item['point_local_m'],final_intervals,atol,rtol,True)
        checked.add(key)
    while True:
        good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
        if not good: break
        best=good[0];key=tuple(round(v,10) for v in best['point_local_m'])
        if key in checked: break
        evaluated[key]=evaluate_candidate(problem,best['point_local_m'],final_intervals,atol,rtol,True)
        checked.add(key)
    result['candidates']=list(evaluated.values())
    result['search']['evaluated_candidates']=len(evaluated)
    result['search']['candidate_budget_reached']=truncated
    if not good:
        result['status']='numerically_uncertain';return result
    best=good[0];best_upper=best['worst_diameter']['envelope_upper_m']
    # Treat only a sub-micrometre numeric tie as a distance tie; do not use a
    # wide near-optimal band to trade primary precision for travel distance.
    tied=[v for v in good if v['worst_diameter']['envelope_upper_m']<=best_upper+1e-7
          and v['high_precision_check']['status']=='passed']
    if tied: best=min(tied,key=lambda v:(v['movement_m'],v['point_local_m']))
    best_upper=best['worst_diameter']['envelope_upper_m']
    result['recommended']=best
    result['preferred_candidates_local_m']=[v['point_local_m'] for v in good
        if v['worst_diameter']['envelope_upper_m']<=(1+eta)*best_upper]
    result['unresolved_competitors_local_m']=[v['point_local_m'] for v in good
        if v['worst_diameter']['sample_lower_m']<=best_upper and v is not best]
    result['preferred_region_semantics']='finite candidates within eta of best evaluated upper score; not a certificate of true eta-optimality'
    result['status']='completed_with_numerical_bounds'
    return result
