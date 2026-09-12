"""R event route with a cap on opportunistic transit bearings per source."""
import math

def advance(solver,action,remaining,cap):
    from e12_coverage_hook import E11
    kind,key,target=action;start=solver.env.pos
    if math.dist(start,target)<1e-7:return True
    planned=[]
    for channel,track in solver.tracks.items():
        owner_second_bearing=kind=='supp' and channel==key and len(track['dirs'])==1
        if channel in solver.env.cleared or (kind!='cover' and channel==key and not owner_second_bearing) or solver.localized(track):continue
        slots=max(0,cap-len(track['dirs']))
        if slots<=0:continue
        planned.extend((u,channel,point) for u,point,_ in solver.segment_points(track,start,target,min(2,slots)))
    seen=set()
    for u,channel,point in sorted(planned):
        marker=(channel,round(point[0],6),round(point[1],6))
        if marker in seen or channel in solver.env.cleared:continue
        seen.add(marker)
        # Recheck cap because an earlier planned point for this same source may have added a bearing.
        if len(solver.tracks[channel]['dirs'])>=cap:continue
        before_eligible={n[1] for n in solver.eligible_targets()};before_cleared=set(solver.env.cleared)
        previous_directions=len(solver.tracks[channel]['dirs']);solver.transit_attempts+=1;solver.joint_transit+=1
        res=solver.measure(point,channel,None,'joint_route_transit')
        if res['measure_result']=='direction':solver.transit_dirs+=1
        solver.maybe_clear_near(solver.tracks[channel])
        new_targets={n[1] for n in solver.eligible_targets()}-before_eligible
        new_bearing=len(solver.tracks[channel]['dirs'])>previous_directions
        if kind=='supp' and channel==key and new_bearing:
            solver.env.c.record({'kind':'E13_owner_second_bearing','old_action':action,'position':solver.env.pos,
                                 'new_radius_m':solver.tracks[channel]['mec'][1] if solver.tracks[channel].get('mec') else None})
            return False
        if not new_targets and solver.env.cleared==before_cleared:continue
        nodes=solver.joint_nodes(remaining)
        if not nodes:return False
        next_action=E11.e5.fast_open_route(solver.env.pos,nodes)[0]
        if next_action[:2]!=action[:2]:
            solver.env.c.record({'kind':'E13_transit_replan','old_action':action,'new_action':next_action,
                                 'new_clear_channels':sorted(new_targets),'position':solver.env.pos})
            return False
    return True

def run(solver,remaining_cover,cap=3):
    from e12_coverage_hook import E11
    remaining=list(remaining_cover)
    for _ in range(220):
        nodes=solver.joint_nodes(remaining)
        if not nodes:return
        action=E11.e5.fast_open_route(solver.env.pos,nodes)[0]
        if not advance(solver,action,remaining,cap):continue
        kind,key,point=action
        if kind=='cover':
            solver.scan_site(key,point);remaining=[x for x in remaining if x[0]!=key]
            if solver.discovery_complete() and len(solver.tracks)>=16:remaining=[]
        elif kind=='clear':
            if key not in solver.env.cleared and solver.local_clear_from_mec(solver.tracks[key],solver.radius_limit):solver.joint_clears+=1
        elif key not in solver.env.cleared:
            solver.measure(point,key,None,'joint_safe_supplement');solver.stats['supplemental_points']+=1;solver.joint_supp+=1
            solver.joint_supp_count[key]=solver.joint_supp_count.get(key,0)+1;solver.maybe_clear_near(solver.tracks[key])
        solver.joint_steps+=1
        if not remaining and solver.discovery_complete() and not solver.eligible_targets() and not solver.broad_tracks():return
    raise RuntimeError('cap route finite decision guard exceeded')
