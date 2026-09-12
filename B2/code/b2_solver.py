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
    """Record one outer mesh stage for auditable convergence checks."""
    best=good[0]
    upper=best['worst_diameter']['envelope_upper_m']
    lower=best['worst_diameter']['sample_lower_m']
    record={"step_m":step,"best_point_local_m":list(best['point_local_m']),
            "best_upper_m":upper,"best_lower_m":lower,
            "candidate_count":len(good)}
    if previous is not None:
        previous_upper=previous['best_upper_m']
        record['relative_best_upper_change']=(
            abs(upper-previous_upper)/max(1e-12,abs(previous_upper))
        )
        record['best_point_shift_m']=math.dist(
            best['point_local_m'],previous['best_point_local_m']
        )
    return record


def solve_b2(observation, error_deg=1.0, *, options=None, physical=None):
    options={} if options is None else dict(options)
    physical={} if physical is None else dict(physical)
    allowed={'grid_step_m','refinement_levels','max_candidates','max_intervals','final_intervals',
             'abs_tolerance_m','rel_tolerance','near_optimal_fraction','candidate_points_local',
             'outer_keep_per_lobe','outer_rel_tolerance','outer_abs_tolerance_m',
             'outer_point_tolerance_m','inner_recheck_intervals','placement_steps_m'}
    if set(options)-allowed: raise ValueError('unknown options: '+str(sorted(set(options)-allowed)))
    problem=Problem(observation,error_deg,**physical)
    step=finite(options.get('grid_step_m',problem.rmin/10),'grid_step_m')
    eta=finite(options.get('near_optimal_fraction',0.05),'near_optimal_fraction')
    if step<=0 or not 0<=eta<=1: raise ValueError('invalid grid step or near-optimal fraction')
    def integer(name,default,minimum):
        x=options.get(name,default)
        if isinstance(x,bool) or not isinstance(x,int) or x<minimum: raise ValueError(name+' invalid')
        return x
    levels=integer('refinement_levels',7,0);budget=integer('max_candidates',1200,1)
    intervals=integer('max_intervals',192,4);final_intervals=integer('final_intervals',768,4)
    outer_keep=integer('outer_keep_per_lobe',2,1)
    atol=finite(options.get('abs_tolerance_m',1.0),'abs_tolerance_m')
    rtol=finite(options.get('rel_tolerance',0.002),'rel_tolerance')
    outer_rtol=finite(options.get('outer_rel_tolerance',0.02),'outer_rel_tolerance')
    outer_atol=finite(options.get('outer_abs_tolerance_m',1.0),'outer_abs_tolerance_m')
    outer_ptol=finite(options.get('outer_point_tolerance_m',12.5),'outer_point_tolerance_m')
    recheck_intervals=integer(
        'inner_recheck_intervals',min(3072,max(1024,2*final_intervals)),4
    )
    if atol<=0 or rtol<0 or outer_rtol<0 or outer_atol<0 or outer_ptol<0:
        raise ValueError('invalid tolerances')
    supplied=options.get('candidate_points_local')
    raw_placement_steps=options.get(
        'placement_steps_m',(50.0,25.0,12.5,6.25)
    )
    if not isinstance(raw_placement_steps,(list,tuple)) or not raw_placement_steps:
        raise ValueError('placement_steps_m must be a nonempty list')
    placement_steps=[
        finite(value,'placement step') for value in raw_placement_steps
    ]
    if (any(value<=0 for value in placement_steps)
            or any(placement_steps[i+1]>=placement_steps[i]
                   for i in range(len(placement_steps)-1))):
        raise ValueError('placement_steps_m must be strictly decreasing positive values')
    result={"schema_version":2,"status":problem.status,"observation":observation,
            "error_deg":error_deg,"physical_parameters":{"domain_center":problem.domain_center,
                "domain_radius":problem.radius,"min_range":problem.rmin,"max_range":problem.rmax,
                "near_range":problem.near,"angle_margin_deg":problem.margin},
            "objective_priority":["worst localization diameter","movement for numerical ties only"],
            "auxiliary_metric":"worst minimum enclosing circle radius; not a replacement primary metric",
            "domains":problem.domain_descriptions(),"candidates":[],"recommended":None,
            "search":{"mode":("finite_candidates" if supplied is not None
                              else "multistart_mesh_convergence"),
                      "grid_step_m":step,"placement_steps_m":(
                          [] if supplied is not None else placement_steps),
                      "refinement_levels":levels,"max_candidates":budget,
                      "max_intervals":intervals,"final_intervals":final_intervals,
                      "inner_recheck_intervals":recheck_intervals,
                      "outer_keep_per_lobe":outer_keep,
                      "outer_rel_tolerance":outer_rtol,
                      "outer_abs_tolerance_m":outer_atol,
                      "outer_point_tolerance_m":outer_ptol,
                      "abs_tolerance_m":atol,"rel_tolerance":rtol,"near_optimal_fraction":eta},
            "outer_convergence":{
                "status":"not_applicable" if supplied is not None else "not_run",
                "stages":[]
            },
            "inner_convergence":{"status":"not_run"},
            "limitations":["not a global continuous placement optimum",
                           "H outer approximation includes the first near disk",
                           "float envelopes and pointwise Decimal rechecks are not interval certificates",
                           "full C_safe boundary is implicit, not computed"]}
    if problem.status!='ready': return result
    evaluated={};truncated=False
    def valid(item):
        return item.get('worst_diameter') is not None and 'worst_diameter' in item
    def upper_key(item):
        # Optimistic ranking for a minimisation problem: the envelope upper
        # endpoint is the only candidate score that can safely exclude a point
        # when it lies below another candidate's lower endpoint.
        return (item['worst_diameter']['envelope_upper_m'],
                item['movement_m'],item['point_local_m'])
    def lower_key(item):
        # Also retain leaders of the pessimistic endpoint. This prevents a
        # noisy upper envelope from trapping the local search in one basin.
        return (item['worst_diameter']['sample_lower_m'],
                item['worst_diameter']['envelope_upper_m'],
                item['movement_m'],item['point_local_m'])
    def add(p):
        nonlocal truncated
        p=tuple(float(v) for v in p)
        key=tuple(round(v,10) for v in p)
        if key in evaluated or not problem.effective_domain(p): return key
        if len(evaluated)>=budget: truncated=True;return key
        evaluated[key]=evaluate_candidate(problem,p,intervals,atol,rtol,False)
        return key

    def add_on_segment(inside,outside):
        """Project to the implemented effective-domain boundary by bisection."""
        a=tuple(float(v) for v in inside);b=tuple(float(v) for v in outside)
        key=tuple(round(v,10) for v in a)
        if key not in evaluated or problem.effective_domain(b):
            return
        lo,hi=0.0,1.0
        for _ in range(80):
            mid=(lo+hi)/2
            q=(a[0]+mid*(b[0]-a[0]),a[1]+mid*(b[1]-a[1]))
            if problem.effective_domain(q): lo=mid
            else: hi=mid
        # Nudge one floating-point scale inside the boundary so the candidate
        # is evaluated in the same strict effective domain used by add().
        for fraction in (lo,lo-1e-12,lo+1e-12):
            add((a[0]+fraction*(b[0]-a[0]),
                 a[1]+fraction*(b[1]-a[1])))
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
    def pattern_search():
        """Independent deterministic compass searches on both objective bounds."""
        offsets_square=tuple((dx,dy) for dx in (-1,0,1) for dy in (-1,0,1)
                             if (dx,dy)!=(0,0))
        offsets_8=tuple((math.cos(2*math.pi*i/8),math.sin(2*math.pi*i/8))
                        for i in range(8))
        offsets_16=tuple((math.cos(2*math.pi*i/16),math.sin(2*math.pi*i/16))
                         for i in range(16))
        def select_starts(items):
            items=sorted((v for v in items if valid(v)),key=upper_key)
            selected=[];chosen=set()
            # Keep independent starts on each side of the first-bearing axis.
            # Each branch is advanced separately so a boundary point cannot
            # divert another branch away from a deeper interior basin.
            for sign in (-1,1):
                lobe=[v for v in items if v['point_local_m'][1]*sign>0]
                for key in (upper_key,lower_key):
                    for item in sorted(lobe,key=key)[:1]:
                        marker=tuple(round(v,10) for v in item['point_local_m'])
                        if marker not in chosen:
                            chosen.add(marker);selected.append(item)
            return selected

        initial=select_starts(evaluated.values())
        starts=initial
        for divisor in (4,8,16):
            stencil_step=step/divisor
            for start in starts:
                a,b=start['point_local_m']
                for i in range(-2,3):
                    for j in range(-2,3):
                        if i or j:
                            add((a+i*stencil_step,b+j*stencil_step))
                if truncated: return
            starts=select_starts(evaluated.values())
        path_starts=[];chosen=set()
        for item in initial+starts:
            marker=tuple(round(v,10) for v in item['point_local_m'])
            if marker not in chosen:
                chosen.add(marker);path_starts.append(item)
        starts=path_starts
        for start in starts:
            for offsets in (offsets_square,offsets_8,offsets_16):
                current=start
                for level in range(levels):
                    a,b=current['point_local_m']
                    local_step=step/(2**(level+1))
                    neighbours=[]
                    for dx,dy in offsets:
                        neighbour=(a+dx*local_step,b+dy*local_step)
                        before=len(evaluated)
                        marker=add(neighbour)
                        if marker in evaluated:
                            neighbours.append(evaluated[marker])
                        if len(evaluated)==before:
                            add_on_segment(current['point_local_m'],neighbour)
                        if truncated: return
                    if neighbours:
                        candidate=min(neighbours,key=upper_key)
                        if upper_key(candidate)<upper_key(current):
                            current=candidate

    def mesh_refinement():
        """Retain several leaders per lobe and refine on decreasing meshes."""
        def select_starts(items, keep):
            ranked=sorted((item for item in items if valid(item)),key=upper_key)
            selected=[];chosen=set()
            for sign in (-1,1):
                lobe=[item for item in ranked if item['point_local_m'][1]*sign>0]
                for key in (upper_key,lower_key):
                    for item in sorted(lobe,key=key)[:keep]:
                        marker=tuple(round(v,10) for v in item['point_local_m'])
                        if marker not in chosen:
                            chosen.add(marker);selected.append(item)
            return selected

        for mesh_step in placement_steps:
            good=sorted((item for item in evaluated.values() if valid(item)),
                        key=upper_key)
            if not good:
                break
            for start in select_starts(good,outer_keep):
                a,b=start['point_local_m']
                for ia in (-1,0,1):
                    for ib in (-1,0,1):
                        if not ia and not ib:
                            continue
                        add((a+ia*mesh_step,b+ib*mesh_step))
                        if truncated:
                            break
                    if truncated:
                        break
                if truncated:
                    break
            good=sorted((item for item in evaluated.values() if valid(item)),
                        key=upper_key)
            previous=result['outer_convergence']['stages'][-1] if (
                result['outer_convergence']['stages']
            ) else None
            if good:
                result['outer_convergence']['stages'].append(
                    _stage_record(mesh_step,good,previous)
                )
            if truncated:
                break

    if supplied is None:
        pattern_search()
        if not truncated:
            mesh_refinement()
    good=sorted((v for v in evaluated.values() if valid(v)),key=upper_key)
    if not good:
        result.update(status='no_candidate_in_search',candidates=list(evaluated.values()))
        return result
    # Re-evaluate both optimistic and pessimistic leaders. A point can look
    # weak because its adaptive envelope is loose; the lower-bound ranking
    # keeps such candidates available for the final, denser comparison.
    leading=[];seen=set()
    for key in (upper_key,lower_key):
        for item in sorted(good,key=key)[:24]:
            marker=tuple(round(v,10) for v in item['point_local_m'])
            if marker not in seen:
                seen.add(marker);leading.append(item)
    for item in leading:
        key=tuple(round(v,10) for v in item['point_local_m'])
        evaluated[key]=evaluate_candidate(
            problem,item['point_local_m'],final_intervals,atol,rtol,True)
    good=sorted((v for v in evaluated.values() if valid(v)),key=upper_key)
    if not good:
        result['status']='numerically_uncertain';return result
    # A distance tie is accepted only when both the pessimistic and optimistic
    # endpoints are numerically equal. A wide near-optimal band must not trade
    # a real primary-metric difference for a shorter move.
    def endpoint_tie(value,reference):
        return abs(value-reference)<=1e-7+1e-9*max(1,abs(reference))
    def select_recommended(items):
        incumbent=items[0]
        best_lower=incumbent['worst_diameter']['sample_lower_m']
        best_upper=incumbent['worst_diameter']['envelope_upper_m']
        tied=[v for v in items
              if endpoint_tie(v['worst_diameter']['sample_lower_m'],best_lower)
              and endpoint_tie(v['worst_diameter']['envelope_upper_m'],best_upper)
              and v['high_precision_check']['status']=='passed']
        return min(tied,key=lambda v:(v['movement_m'],v['point_local_m']))

    best=select_recommended(good)
    audit_key=tuple(round(v,10) for v in best['point_local_m'])
    # Re-evaluate the actual selected point at the final placement resolution,
    # then separate its angular-envelope error with one denser inner recheck.
    base=evaluate_candidate(
        problem,best['point_local_m'],final_intervals,atol,rtol,True
    )
    rechecked=evaluate_candidate(
        problem,best['point_local_m'],recheck_intervals,atol,rtol,True
    )
    if valid(base) and valid(rechecked):
        evaluated[audit_key]=base
        base_upper=base['worst_diameter']['envelope_upper_m']
        recheck_upper=rechecked['worst_diameter']['envelope_upper_m']
        change=abs(recheck_upper-base_upper)
        tolerance=max(
            atol,rtol*rechecked['worst_diameter']['sample_lower_m']
        )
        evaluated[audit_key]=rechecked
        good=sorted((v for v in evaluated.values() if valid(v)),key=upper_key)
        final_best=select_recommended(good)
        final_key=tuple(round(v,10) for v in final_best['point_local_m'])
        ranking_unchanged=final_key==audit_key
        result['inner_convergence']={
            "status":(
                "stable" if change<=tolerance and ranking_unchanged
                else "needs_refinement"
            ),
            "base_intervals":final_intervals,
            "recheck_intervals":recheck_intervals,
            "audited_point_local_m":list(best['point_local_m']),
            "final_recommended_after_recheck_local_m":list(
                final_best['point_local_m']
            ),
            "ranking_unchanged_after_recheck":ranking_unchanged,
            "base_upper_m":base_upper,
            "recheck_upper_m":recheck_upper,
            "absolute_change_m":change,
            "tolerance_m":tolerance,
            "semantics":"final selected point; angular interval refinement only"
        }
    else:
        result['inner_convergence']={
            "status":"numerically_uncertain",
            "base_intervals":final_intervals,
            "recheck_intervals":recheck_intervals,
            "audited_point_local_m":list(best['point_local_m']),
            "semantics":"final selected point; angular interval refinement only"
        }
    result['candidates']=list(evaluated.values())
    result['search']['evaluated_candidates']=len(evaluated)
    result['search']['candidate_budget_reached']=truncated
    best=select_recommended(good)
    best_lower=best['worst_diameter']['sample_lower_m']
    best_upper=best['worst_diameter']['envelope_upper_m']
    result['recommended']=best
    result['preferred_candidates_local_m']=[v['point_local_m'] for v in good
        if v['worst_diameter']['envelope_upper_m']<=(1+eta)*best_upper]
    result['unresolved_competitors_local_m']=[v['point_local_m'] for v in good
        if (v['worst_diameter']['sample_lower_m']<=best_upper+1e-7
            and best_lower<=v['worst_diameter']['envelope_upper_m']+1e-7
            and v is not best)]
    result['selection_assessment']={
        'unresolved_competitor_count':len(result['unresolved_competitors_local_m']),
        'status':('intervals_overlap' if result['unresolved_competitors_local_m']
                  else 'no_observed_interval_overlap'),
        'outer_convergence_status':result['outer_convergence']['status'],
        'inner_convergence_status':result['inner_convergence']['status'],
        'semantics':'finite candidates and floating-point envelopes only; no continuous global-optimality claim'}
    result['preferred_region_semantics']='finite candidates within eta of best evaluated upper score; not a certificate of true eta-optimality'

    if supplied is None:
        stages=result['outer_convergence']['stages']
        if len(stages)>=2:
            last=stages[-1]
            previous=stages[-2]
            absolute_change=abs(last['best_upper_m']-previous['best_upper_m'])
            objective_tolerance=max(
                outer_atol,outer_rtol*max(1e-12,last['best_upper_m'])
            )
            point_tolerance=max(outer_ptol,last['step_m']*2)
            stable=(
                absolute_change<=objective_tolerance
                and last.get('best_point_shift_m',math.inf)<=point_tolerance
                and not truncated
            )
            result['outer_convergence'].update({
                "status":"stable" if stable else "needs_refinement",
                "last_absolute_objective_change_m":absolute_change,
                "last_relative_objective_change":last.get(
                    'relative_best_upper_change',math.inf
                ),
                "last_best_point_shift_m":last.get(
                    'best_point_shift_m',math.inf
                ),
                "objective_tolerance_m":objective_tolerance,
                "point_tolerance_m":point_tolerance,
                "semantics":"multi-start mesh refinement; not a continuous global proof"
            })
        else:
            result['outer_convergence']['status']='needs_refinement'

    needs_refinement=(
        truncated
        or result['inner_convergence']['status']!='stable'
        or (supplied is None
            and result['outer_convergence']['status']!='stable')
    )
    result['status']=(
        'needs_refinement' if needs_refinement
        else 'completed_with_numerical_bounds'
    )
    return result
