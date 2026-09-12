"""G16 DEV: replace mandatory inner certificate sites by already-visited source-task points.

The directional absence proof only needs a valid short-edge triangulation whose
vertices were all measured for every still-undiscovered channel.  The 25-role G15
certificate topology is retained exactly.  When the robot happens to finish a source
operation at a point that can replace one of the twelve inner-ring vertices while the
*entire substituted mesh* still passes ``verify_mesh``, we may sweep the currently
unknown channels there and permanently fulfil that certificate role.  Its original
inner-ring site is then deleted from the mandatory route.

No arbitrary no_signal is used geometrically.  A surrogate is accepted only after a
full unknown-channel sweep at that exact point, and the complete substituted mesh is
re-verified by the exact rational certificate checker.  The G15/G0 fan and fallback
chain is unchanged.
"""
from __future__ import annotations

import math

from discovery_certificate import verify_mesh
from solver_g15 import G15TightMesh
from solver_g1 import E11


class G16SurrogateMesh(G15TightMesh):
    # Seconds of a full sweep are compared with certificate-route distance saving.
    # 5 s measurement + at most 1 s channel switch = <=30 m of travel-equivalent
    # at 5 m/s.  The extra margin avoids substitutions with merely numerical gain.
    sweep_equivalent_m_per_channel = 30.0
    route_saving_margin_m = 50.0

    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.role_points=list(self.mesh['vertices'])
        self.fulfilled_roles=set()
        self.stats.update({'surrogate_sweeps':0,'surrogate_roles':0,
                           'surrogate_measurements':0,'surrogate_detections':0,
                           'surrogate_route_saving_estimate_m':0.0,
                           'surrogate_candidates_rejected_certificate':0,
                           'surrogate_candidates_rejected_cost':0})

    def _unknown_channels(self):
        return [c for c in range(1,21) if c not in self.tracks and c not in self.absent]

    def _mark_role(self,role):
        self.fulfilled_roles.add(role)
        if len(self.fulfilled_roles)==len(self.role_points) and len(self.tracks)<16:
            # A channel still unknown at this instant has been unknown at every prior
            # full role sweep, hence returned no_signal at every vertex of the exact
            # verified substituted mesh.
            for c in self._unknown_channels():
                self.absent.add(c)

    def scan_certificate(self,site):
        super().scan_certificate(site)
        self._mark_role(site)

    def _mesh_with(self,role,p):
        vertices=list(self.role_points); vertices[role]=tuple(p)
        candidate=dict(self.mesh); candidate['vertices']=vertices
        verify_mesh(candidate)
        return candidate

    def _certificate_route_cost(self,remaining):
        if not remaining:return 0.0
        nodes=[('cover',i,self.mesh['vertices'][i]) for i in remaining]
        route=E11.e5.fast_open_route(self.port.position,nodes)
        return E11.e5.route_cost(self.port.position,route)

    def _best_surrogate(self,remaining):
        # Only the twelve inner roles can be replaced; center and outer boundary stay
        # fixed, so disk containment is never weakened by a surrogate.
        p=tuple(self.port.position)
        inner=[i for i in remaining if 1<=i<=12]
        if not inner:return None
        unknown=self._unknown_channels()
        if not unknown:return None
        base=self._certificate_route_cost(remaining)
        needed=self.sweep_equivalent_m_per_channel*len(unknown)+self.route_saving_margin_m
        options=[]
        for role in inner:
            try:
                candidate=self._mesh_with(role,p)
            except Exception:
                self.stats['surrogate_candidates_rejected_certificate']+=1
                continue
            alt_remaining=[i for i in remaining if i!=role]
            saving=base-self._certificate_route_cost(alt_remaining)
            if saving+1e-8<needed:
                self.stats['surrogate_candidates_rejected_cost']+=1
                continue
            options.append((saving,role,candidate))
        return max(options,key=lambda x:x[0]) if options else None

    def try_surrogate(self,remaining):
        if self.discovery_complete():return
        best=self._best_surrogate(remaining)
        if best is None:return
        saving,role,candidate=best
        p=tuple(self.port.position)
        # Measure exactly the channels that are unknown at the start.  If some are
        # detected, they cease to matter for absence; every channel still unknown at
        # the end has necessarily returned no_signal at this surrogate point.
        unknown=self._unknown_channels()
        unknown.sort(key=lambda c:(c!=self.port.channel,c))
        self.stats['surrogate_sweeps']+=1
        for c in unknown:
            if len(self.tracks)==16:break
            body=self.observe(p,c,'B4_certificate_surrogate')
            self.stats['surrogate_measurements']+=1
            if body['measure_result']!='no_signal':self.stats['surrogate_detections']+=1
        self.role_points[role]=p
        # Store the already-verified complete candidate vertex set.  Rebuild from the
        # current role_points to guard against accidental divergence.
        self.mesh=dict(self.mesh); self.mesh['vertices']=list(self.role_points)
        verify_mesh(self.mesh)
        self._mark_role(role)
        self.stats['surrogate_roles']+=1
        self.stats['surrogate_route_saving_estimate_m']+=saving
        if role in remaining:remaining.remove(role)
        self.port.record({'kind':'B4_surrogate_role','role':role,'position':p,
                          'estimated_route_saving_m':saving,'unknown_swept':len(unknown)})

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        for _ in range(1900):
            nodes=[] if self.discovery_complete() else [('cover',i,self.mesh['vertices'][i]) for i in remaining]
            for c in self.tracks:
                if c in self.cleared:continue
                task=self.source_task(c)
                if task[0]=='fan' and remaining and not self.discovery_complete() and self.has_future_opportunity(c,remaining):
                    self.stats['fan_deferred_cycles']+=1;continue
                nodes.append(task)
            if not nodes:break
            task=self.select_task(nodes); dest=task[2]
            exclude=None if task[0]=='cover' else task[1]
            self.transit_opportunities(dest,exclude_channel=exclude)
            self.port.record({'kind':'B4_schedule','task':task,'active_tasks':len(nodes),'policy':'G16_surrogate'})
            if task[0]=='cover':
                self.scan_certificate(task[1]);remaining.remove(task[1])
            else:
                self.execute_source(task)
                # The source operation has already paid for the current location.
                # Try to turn it into one mandatory inner certificate vertex.
                self.try_surrogate(remaining)
        else:raise RuntimeError('finite decision guard')
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise RuntimeError('incomplete discovery or clearance')


class G16Aggressive(G16SurrogateMesh):
    route_saving_margin_m=0.0

class G16Conservative(G16SurrogateMesh):
    route_saving_margin_m=150.0

class G16CheapProbe(G16SurrogateMesh):
    # Optimistic version used only to explore sensitivity; 25 m/channel corresponds
    # to the pure 5 s measurement cost and ignores switches in the threshold.
    sweep_equivalent_m_per_channel=25.0
    route_saving_margin_m=25.0
