"""G41 Phase A: certificate-vertex pair costs and deterministic route search."""
import math
import time

from solver_g35 import G40PassiveSafe
from solver_g1 import E11
from jcr_cost import (safe_site_info,resolver_pair,active_resolver_worst_case_eq_m,
                      local_search,route_length,should_defer,direction_plan,radial_centers)


class G41JCR(G40PassiveSafe):
    def __init__(self,*args,jcr_enabled=True,**kwargs):
        super().__init__(*args,**kwargs)
        self.jcr_enabled=jcr_enabled;self.pending={};self.planned={};self.observed={};self._last_deferral={}
        self.stats.update({k:0 for k in (
            'jcr_replans','jcr_local_search_evaluations','jcr_route_length_delta_m',
            'jcr_pairs_considered','jcr_pairs_selected','jcr_pair_deferrals','jcr_pair_resolutions',
            'jcr_pair_abandoned_after_early_stop','jcr_active_fallbacks','jcr_predicted_eq_m_saved',
            'jcr_extra_measurements','jcr_extra_switches','jcr_route_clear_attempts',
            'jcr_route_clear_successes','jcr_invariant_failures','jcr_certified_clear_attempts',
            'jcr_planner_budget_exhausted')})
        self.stats['jcr_realized_movement_saved_m']=None  # reporter fills counterfactual paired delta
        self.stats['planner_wall_time_s']=0.

    def observe(self,p,c,reason,site=None,backside=False):
        body=super().observe(p,c,reason,site=site,backside=backside)
        if self.jcr_enabled:self.observed[c,tuple(p)]=dict(body)
        return body

    def cancel_certificate_pairs(self):
        if len(self.tracks)<16:return
        channels=set(self.pending)|set(self.planned)
        channels-={c for c in channels if c in self.cleared}
        if channels:
            self.stats['jcr_pair_abandoned_after_early_stop']+=len(channels)
            self.port.record({'kind':'G41_cancel_certificate_pairs','channels':sorted(channels),'discovered_count':16})
        self.pending.clear();self.planned.clear()

    def plan_route(self,remaining,tasks):
        start_clock=time.perf_counter();self.stats['jcr_replans']+=1
        points=self.mesh['vertices'];initial=[t[1] for t in E11.e5.fast_open_route(self.port.position,[('cover',i,points[i]) for i in remaining])]
        active={};options={}
        for c,task in tasks.items():
            t=self.tracks[c]
            active[c]=active_resolver_worst_case_eq_m(t,self.port.position,self.fan_limit)
            if task[0]!='fan' or c in self.pending:continue
            obs=t.dirs[0]
            sites={i:safe_site_info(obs,points[i],channel=c) for i in remaining if not self._already_sampled(t,points[i])}
            sites={i:s for i,s in sites.items() if s.safe and s.side and s.predicted_dir_clear_count is not None}
            pairs=[]
            for i,a in sorted(sites.items()):
                for j,b in sorted(sites.items()):
                    if a.side*b.side!=-1:continue
                    self.stats['jcr_pairs_considered']+=1
                    pair=resolver_pair(obs,i,a,j,b)
                    if pair is not None:pairs.append(pair)
            options[c]=pairs
        if any(options.values()):
            result=local_search(self.port.position,initial,points,active,options)
            self.stats['jcr_local_search_evaluations']+=result['evaluations']
            order=result['order'];selected=result['selected']
        else:order=initial;selected={}
        delta=route_length(self.port.position,order,points)-route_length(self.port.position,initial,points)
        self.stats['jcr_route_length_delta_m']+=delta
        self.planned={c:p for c,p in selected.items() if should_defer(tasks[c][0],len(self.tracks),p,active[c],remaining)}
        self.stats['jcr_pairs_selected']+=len(self.planned)
        for c,p in self.planned.items():
            marker=(p.site_i,p.site_j)
            if self._last_deferral.get(c)==marker:continue
            self._last_deferral[c]=marker
            self.stats['jcr_pair_deferrals']+=1
            self.stats['jcr_predicted_eq_m_saved']+=active[c]-p.worst_case_eq_m
            self.port.record({'kind':'G41_pair_deferral','channel':c,'active_wc_eq_m':active[c],
                              'pair_wc_eq_m':p.worst_case_eq_m,'pair_sites':list(marker),'route_delta_m':delta,
                              'branches_eq_m':p.branch_costs_eq_m,'prediction_kind':'upper-bound difference, not realized saving'})
        self.stats['planner_wall_time_s']+=time.perf_counter()-start_clock
        if self.stats['planner_wall_time_s']>30.:
            self.stats['jcr_planner_budget_exhausted']+=1
            self.port.record({'kind':'G41_planner_budget','seconds':self.stats['planner_wall_time_s']})
            self.jcr_enabled=False;self.pending.clear();self.planned.clear()
        return order

    def _invariant_fallback(self,c,reason):
        self.stats['jcr_invariant_failures']+=1;self.stats['jcr_active_fallbacks']+=1
        self.port.record({'kind':'G41_invariant_failure','channel':c,'reason':reason,'recovery':'unchanged finite fallback'})
        self.pending.pop(c,None);self.planned.pop(c,None)
        self.fallback(c)

    def _execute_cover(self,c,plan,bound):
        if plan is None or plan['worst_eq_m']>bound+1e-4:
            self._invariant_fallback(c,'certified branch unavailable or exceeds preregistered bound');return
        self.port.record({'kind':'G41_finite_clear','channel':c,'count':plan['count'],
                          'branch_wc_eq_m':plan['worst_eq_m'],'predicted_branch_wc_eq_m':bound,
                          'centers':plan['centers']})
        for p in plan['centers']:
            self.stats['jcr_certified_clear_attempts']+=1
            if self.clear(p,c,False,'B4_G41_certified_local_clear'):
                self.stats['jcr_pair_resolutions']+=1;self.pending.pop(c,None);self.planned.pop(c,None);return
        self._invariant_fallback(c,'finite cover exhausted without clear')

    def scan_certificate(self,site):
        if not self.jcr_enabled:return super().scan_certificate(site)
        due={c:(rec['pair'],True) for c,rec in self.pending.items() if rec['pair'].site_j==site}
        due.update({c:(p,False) for c,p in self.planned.items() if p.site_i==site and c not in due})
        super().scan_certificate(site)
        if len(self.tracks)==16:
            self.cancel_certificate_pairs();return
        p=tuple(self.mesh['vertices'][site])
        for c,(pair,second) in sorted(due.items()):
            if c in self.cleared:continue
            t=self.tracks[c]
            if t.near is not None or (t.mec and t.mec[1]<=20-1e-6):
                self.pending.pop(c,None);self.planned.pop(c,None);continue
            info=pair.second if second else pair.first
            if not info.safe or p!=info.point:
                self._invariant_fallback(c,'unsafe or stale certificate pair');continue
            obs=dict(t.dirs[0]);body=self.observed.get((c,p))
            if body is None:
                self.stats['jcr_extra_measurements']+=1
                self.stats['jcr_extra_switches']+=int(self.port.channel!=c)
                body=self.observe(p,c,'B4_G41_pair_probe',backside=True)
                self._record_safe_result(c,p,body,'site')
            kind=body['measure_result']
            if kind=='near':
                self.clear(p,c,True,'B4_G41_near_clear');self.stats['jcr_pair_resolutions']+=1
                self.pending.pop(c,None);self.planned.pop(c,None)
            elif kind=='direction':
                plan=direction_plan(obs,info,body,p)
                self._execute_cover(c,plan,info.dir_completion_eq_m)
            elif not second:
                self.pending[c]={'pair':pair,'obs':obs}
            else:
                plan=radial_centers(obs,pair.chord_radius_upper_m,p)
                bound=pair.branch_costs_eq_m[2]-pair.first.extra_measure_eq_m-pair.second.extra_measure_eq_m
                self._execute_cover(c,plan,bound)

    def execute_source(self,task):
        try:return super().execute_source(task)
        except RuntimeError as exc:
            if self.jcr_enabled and task[0]=='strip' and 'certified active strip cover exhausted' in str(exc):
                self._invariant_fallback(task[1],str(exc));return
            raise

    def run_all(self):
        if not self.jcr_enabled:return super().run_all()
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1700):
            self.cancel_certificate_pairs()
            self.pending={c:r for c,r in self.pending.items() if c not in self.cleared and r['pair'].site_j in remaining}
            tasks={c:super(G41JCR,self).source_task(c) for c in self.tracks if c not in self.cleared}
            covers=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            order=self.plan_route(remaining,tasks) if self.jcr_enabled and covers and tasks else []
            nodes=list(covers)
            for c,task in tasks.items():
                if task[0]=='fan' and covers:
                    if self.jcr_enabled and (c in self.pending or c in self.planned):continue
                    if self.has_future_opportunity(c,remaining):
                        self.stats['fan_deferred_cycles']+=1;continue
                if self._defer_clear(task,remaining):
                    self.stats['guaranteed_clear_deferred_cycles']+=1;continue
                nodes.append(task)
            if not nodes:break
            if self.jcr_enabled and self.planned and order:
                # The committed next certificate vertex competes with existing source
                # tasks. After any source action all conditional future costs are rebuilt.
                shortlist=[t for t in nodes if t[0]!='cover']+[('cover',order[0],self.mesh['vertices'][order[0]])]
                task=E11.e5.fast_open_route(self.port.position,shortlist)[0]
            else:task=self.select_task(nodes)
            self.transit_opportunities(task[2],exclude_channel=None if task[0]=='cover' else task[1])
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G41_JCR'})
            if task[0]=='cover':
                self.scan_certificate(task[1]);remaining.remove(task[1])
            else:self.execute_source(task)
        else:raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):raise RuntimeError('incomplete discovery or clearance')
