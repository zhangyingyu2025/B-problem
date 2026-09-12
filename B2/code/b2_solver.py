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



def _stage_record(step, good, previous=None):
    best=good[0]
    upper=best['worst_diameter']['envelope_upper_m']
    lower=best['worst_diameter']['sample_lower_m']
    record={"step_m":step,"best_point_local_m":list(best['point_local_m']),
            "best_upper_m":upper,"best_lower_m":lower,"candidate_count":len(good)}
    if previous is not None:
        prev_upper=previous['best_upper_m'];prev_point=previous['best_point_local_m']
        record['relative_best_upper_change']=abs(upper-prev_upper)/max(1e-12,abs(prev_upper))
        record['best_point_shift_m']=math.dist(best['point_local_m'],prev_point)
    return record


def solve_b2(observation, error_deg=1.0, *, options=None, physical=None):
    """Solve B2 with separate inner-reading and outer-placement refinement.

    The default placement search now uses a multi-start 50/25/12.5/6.25 m
    hierarchy.  Multiple competing basins on both lobes are retained at every
    level.  `outer_convergence` explicitly reports whether the best objective
    and best point stabilized under mesh refinement; a non-stable search is
    returned as ``needs_refinement`` rather than silently freezing a grid
    artefact.  Explicit `candidate_points_local` keeps the finite-candidate
    semantics used by earlier tests and reproducibility scripts.
    """
    options={} if options is None else dict(options)
    physical={} if physical is None else dict(physical)
    allowed={'grid_step_m','refinement_levels','max_candidates','max_intervals','final_intervals',
             'abs_tolerance_m','rel_tolerance','near_optimal_fraction','candidate_points_local',
             'placement_steps_m','outer_keep_per_lobe','outer_rel_tolerance',
             'outer_abs_tolerance_m','outer_point_tolerance_m','inner_recheck_intervals'}
    if set(options)-allowed: raise ValueError('unknown options: '+str(sorted(set(options)-allowed)))
    problem=Problem(observation,error_deg,**physical)
    eta=finite(options.get('near_optimal_fraction',0.05),'near_optimal_fraction')
    if not 0<=eta<=1: raise ValueError('invalid near-optimal fraction')
    def integer(name,default,minimum):
        x=options.get(name,default)
        if isinstance(x,bool) or not isinstance(x,int) or x<minimum: raise ValueError(name+' invalid')
        return x
    intervals=integer('max_intervals',96,4);final_intervals=integer('final_intervals',768,4)
    atol=finite(options.get('abs_tolerance_m',1.0),'abs_tolerance_m')
    rtol=finite(options.get('rel_tolerance',0.002),'rel_tolerance')
    if atol<=0 or rtol<0: raise ValueError('invalid tolerances')
    outer_keep=integer('outer_keep_per_lobe',8,1)
    outer_rtol=finite(options.get('outer_rel_tolerance',0.02),'outer_rel_tolerance')
    outer_atol=finite(options.get('outer_abs_tolerance_m',1.0),'outer_abs_tolerance_m')
    outer_ptol=finite(options.get('outer_point_tolerance_m',12.5),'outer_point_tolerance_m')
    if outer_rtol<0 or outer_atol<0 or outer_ptol<0: raise ValueError('invalid outer convergence tolerance')
    recheck_intervals=integer('inner_recheck_intervals',min(3072,max(1024,2*final_intervals)),4)

    supplied=options.get('candidate_points_local')
    if supplied is None:
        requested_levels=integer('refinement_levels',3,0)
        if 'placement_steps_m' in options:
            raw_steps=options['placement_steps_m']
        else:
            base=finite(options.get('grid_step_m',50.0),'grid_step_m')
            raw_steps=[base/(2**i) for i in range(requested_levels+1)]
        if not isinstance(raw_steps,(list,tuple)) or not raw_steps: raise ValueError('placement_steps_m must be nonempty')
        steps=[finite(v,'placement step') for v in raw_steps]
        if any(v<=0 for v in steps) or any(steps[i+1]>=steps[i] for i in range(len(steps)-1)):
            raise ValueError('placement_steps_m must be strictly decreasing positive values')
        budget=integer('max_candidates',2500,1)
        legacy_step=steps[0];levels=len(steps)-1
    else:
        # Explicit finite-candidate mode remains backward compatible.
        legacy_step=finite(options.get('grid_step_m',problem.rmin/10),'grid_step_m')
        levels=integer('refinement_levels',0,0);budget=integer('max_candidates',180,1)
        steps=[]

    result={"schema_version":2,"status":problem.status,"observation":observation,
            "error_deg":error_deg,"physical_parameters":{"domain_center":problem.domain_center,
                "domain_radius":problem.radius,"min_range":problem.rmin,"max_range":problem.rmax,
                "near_range":problem.near,"angle_margin_deg":problem.margin},
            "objective_priority":["worst localization diameter","movement for numerical ties only"],
            "auxiliary_metric":"worst minimum enclosing circle radius; not a replacement primary metric",
            "domains":problem.domain_descriptions(),"candidates":[],"recommended":None,
            "search":{"mode":"finite_candidates" if supplied is not None else "multistart_mesh_convergence",
                      "placement_steps_m":steps,"grid_step_m":legacy_step,
                      "refinement_levels":levels,"max_candidates":budget,
                      "max_intervals":intervals,"final_intervals":final_intervals,
                      "inner_recheck_intervals":recheck_intervals,
                      "abs_tolerance_m":atol,"rel_tolerance":rtol,"near_optimal_fraction":eta,
                      "outer_keep_per_lobe":outer_keep,"outer_rel_tolerance":outer_rtol,
                      "outer_abs_tolerance_m":outer_atol,"outer_point_tolerance_m":outer_ptol},
            "outer_convergence":{"status":"not_applicable" if supplied is not None else "not_run","stages":[]},
            "inner_convergence":{"status":"not_run"},
            "limitations":["not a rigorous continuous-placement global optimum certificate",
                           "H outer approximation includes the first near disk",
                           "float envelopes and pointwise Decimal rechecks are not interval certificates",
                           "full C_safe boundary is implicit, not computed"]}
    if problem.status!='ready': return result
    evaluated={};truncated=False
    def valid(item): return item.get('worst_diameter') is not None and 'worst_diameter' in item
    def rank(item): return (item['worst_diameter']['envelope_upper_m'],item['movement_m'],item['point_local_m'])
    def add(p, max_int=intervals):
        nonlocal truncated
        p=tuple(float(v) for v in p);key=tuple(round(v,10) for v in p)
        if key in evaluated or not problem.effective_domain(p): return False
        if len(evaluated)>=budget: truncated=True;return False
        evaluated[key]=evaluate_candidate(problem,p,max_int,atol,rtol,False)
        return True

    if supplied is not None:
        if not isinstance(supplied,list) or not supplied: raise ValueError('candidate_points_local must be a nonempty list')
        for p in supplied:
            if not isinstance(p,(list,tuple)) or len(p)!=2: raise ValueError('invalid candidate point')
            add(tuple(finite(v,'candidate coordinate') for v in p))
        # Preserve the old explicit-candidate local-refinement behavior.
        for level in range(levels):
            good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
            leaders=[]
            for sign in (-1,1): leaders += [v for v in good if v['point_local_m'][1]*sign>0][:2]
            local_step=legacy_step/(2**(level+1))
            for leader in leaders:
                a,b=leader['point_local_m']
                for dx in (-local_step,0,local_step):
                    for dy in (-local_step,0,local_step): add((a+dx,b+dy))
    else:
        # Global first mesh.  Candidate coordinates are intentionally bounded
        # by C_cert's |p|<r_min property, so no arbitrary spatial clipping is introduced.
        step=steps[0];count=math.ceil(problem.rmin/step)
        for i in range(0,count+1):
            a=i*step
            for j in range(1,count+1):
                b=j*step
                add((a,-b));add((a,b))
                if truncated: break
            if truncated: break
        good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
        if good: result['outer_convergence']['stages'].append(_stage_record(step,good))

        # Multi-basin refinement: retain several leaders on each lobe plus any
        # candidate whose sampled lower bound still overlaps the incumbent upper.
        for step in steps[1:]:
            good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
            if not good: break
            best_upper=good[0]['worst_diameter']['envelope_upper_m']
            retain=[]
            for sign in (-1,1):
                lobe=[v for v in good if v['point_local_m'][1]*sign>0]
                overlap=[v for v in lobe if v['worst_diameter']['sample_lower_m']<=best_upper+max(outer_atol,outer_rtol*best_upper)]
                pool=overlap[:2*outer_keep] if overlap else lobe[:outer_keep]
                seen=set()
                for v in lobe[:outer_keep]+pool:
                    key=tuple(v['point_local_m'])
                    if key not in seen: retain.append(v);seen.add(key)
            # The previous mesh spacing is twice this step in the default hierarchy.
            # A 5x5 patch spans +/- one previous cell and can move to adjacent basins.
            for leader in retain:
                a,b=leader['point_local_m']
                for ia in range(-2,3):
                    for ib in range(-2,3): add((a+ia*step,b+ib*step))
            good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
            prev=result['outer_convergence']['stages'][-1] if result['outer_convergence']['stages'] else None
            if good: result['outer_convergence']['stages'].append(_stage_record(step,good,prev))

    good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
    if not good:
        result.update(status='no_candidate_in_search',candidates=list(evaluated.values()))
        return result

    # Re-evaluate leading candidates with the final inner-reading budget.
    checked=set()
    for item in good[:min(10,len(good))]:
        key=tuple(round(v,10) for v in item['point_local_m'])
        evaluated[key]=evaluate_candidate(problem,item['point_local_m'],final_intervals,atol,rtol,True);checked.add(key)
    while True:
        good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
        best=good[0];key=tuple(round(v,10) for v in best['point_local_m'])
        if key in checked: break
        evaluated[key]=evaluate_candidate(problem,best['point_local_m'],final_intervals,atol,rtol,True);checked.add(key)

    # One independent inner-resolution recheck at the eventual winner separates
    # reading-envelope error from placement-grid error.
    good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
    best=good[0]
    inner_hi=evaluate_candidate(problem,best['point_local_m'],recheck_intervals,atol,rtol,True)
    if valid(inner_hi):
        base_u=best['worst_diameter']['envelope_upper_m'];hi_u=inner_hi['worst_diameter']['envelope_upper_m']
        change=abs(hi_u-base_u);tol=max(atol,rtol*inner_hi['worst_diameter']['sample_lower_m'])
        result['inner_convergence']={"status":"stable" if change<=tol else "needs_refinement",
            "base_intervals":final_intervals,"recheck_intervals":recheck_intervals,
            "base_upper_m":base_u,"recheck_upper_m":hi_u,"absolute_change_m":change,"tolerance_m":tol}
        key=tuple(round(v,10) for v in best['point_local_m']);evaluated[key]=inner_hi
    else:
        result['inner_convergence']={"status":"numerically_uncertain","recheck_intervals":recheck_intervals}

    good=sorted((v for v in evaluated.values() if valid(v)),key=rank)
    result['candidates']=list(evaluated.values());result['search']['evaluated_candidates']=len(evaluated)
    result['search']['candidate_budget_reached']=truncated
    best=good[0];best_upper=best['worst_diameter']['envelope_upper_m']
    tied=[v for v in good if v['worst_diameter']['envelope_upper_m']<=best_upper+1e-7
          and v['high_precision_check']['status']=='passed']
    if tied: best=min(tied,key=lambda v:(v['movement_m'],v['point_local_m']))
    best_upper=best['worst_diameter']['envelope_upper_m'];result['recommended']=best
    result['preferred_candidates_local_m']=[v['point_local_m'] for v in good
        if v['worst_diameter']['envelope_upper_m']<=(1+eta)*best_upper]
    result['unresolved_competitors_local_m']=[v['point_local_m'] for v in good
        if v['worst_diameter']['sample_lower_m']<=best_upper and v is not best]
    result['preferred_region_semantics']='finite candidates within eta of best evaluated upper score; not a certificate of true eta-optimality'

    if supplied is None:
        stages=result['outer_convergence']['stages']
        if len(stages)>=2:
            last=stages[-1];rel=last.get('relative_best_upper_change',math.inf);shift=last.get('best_point_shift_m',math.inf)
            obj_tol=max(outer_atol,outer_rtol*max(1e-12,last['best_upper_m']))
            abs_obj=abs(last['best_upper_m']-stages[-2]['best_upper_m'])
            stable=(abs_obj<=obj_tol and shift<=max(outer_ptol,last['step_m']*2) and not truncated)
            result['outer_convergence'].update({"status":"stable" if stable else "needs_refinement",
                "last_absolute_objective_change_m":abs_obj,"last_relative_objective_change":rel,
                "last_best_point_shift_m":shift,"objective_tolerance_m":obj_tol,
                "point_tolerance_m":max(outer_ptol,last['step_m']*2)})
        else:
            result['outer_convergence']['status']='needs_refinement'
    result['status']=('needs_refinement' if supplied is None and
                      (result['outer_convergence']['status']!='stable' or result['inner_convergence']['status']!='stable')
                      else 'completed_with_numerical_bounds')
    return result
