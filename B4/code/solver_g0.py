"""G0 correctness baseline; decisions consume only the RobotPort interface."""
from dataclasses import dataclass, field
import math

from b4_geometry import geometry, lower_distance, fallback_cover
from discovery_mesh import mesh25
from discovery_certificate import verify_mesh, absent_certified
from paired_fan import paired_fan


class InvariantFailure(RuntimeError):
    pass


class RobotPort:
    """Observation/action facade; no source-case metadata in the public API."""
    def __init__(self, client):
        self.__client=client
    @property
    def position(self): return self.__client.position
    @property
    def channel(self): return self.__client.channel
    @property
    def virtual_time_s(self): return self.__client.virtual_time_s
    def measure(self,p,c,reason): return self.__client.measure(p,c,reason)
    def clear(self,p,c,reason): return self.__client.clear(p,c,reason)
    def record(self,event): return self.__client.record(event)


@dataclass
class SourceTrackB4:
    channel: int
    dirs: list = field(default_factory=list)
    signals: list = field(default_factory=list)
    ambiguous_no_signal: list = field(default_factory=list)
    proven_backside: list = field(default_factory=list)
    clear_fail_points: list = field(default_factory=list)
    near: object = None
    geometry: object = None
    mec: object = None
    type_state: str = 'unknown'
    last_signal_point: object = None
    last_signal_bearing: object = None
    fan_steps: int = 0


class G0Solver:
    def __init__(self, port, mesh=None, fan_limit=8):
        self.port=port; self.mesh=mesh or mesh25(); self.certificate=verify_mesh(self.mesh)
        self.fan_limit=fan_limit; self.tracks={}; self.cleared=set(); self.absent=set()
        self.checked={c:set() for c in range(1,21)}
        self.stats={'certificate_sites_visited':0,'fan_steps':0,'fan_first_success':0,
                    'fan_second_required':0,'fan_invariant_failures':0,'guaranteed_clear_count':0,
                    'guaranteed_clear_failures':0,'fallback_clear_cover_count':0,'fallback_triggers':0}
        self.detection_times={}

    def discovery_complete(self):
        return len(self.tracks)==16 or all(c in self.tracks or c in self.absent for c in range(1,21))

    def observe(self,p,c,reason,site=None,backside=False):
        body=self.port.measure(p,c,reason); kind=body['measure_result']
        if c in self.cleared:
            return body
        if kind=='no_signal':
            if c in self.tracks:
                t=self.tracks[c]; t.ambiguous_no_signal.append(tuple(p))
                if backside:
                    t.proven_backside.append(tuple(p)); t.type_state='directional'
            elif site is not None:
                self.checked[c].add(site)
                if absent_certified(self.checked[c],self.mesh): self.absent.add(c)
            return body
        if c not in self.tracks:
            self.tracks[c]=SourceTrackB4(c); self.detection_times[c]=self.port.virtual_time_s
        t=self.tracks[c]; t.last_signal_point=tuple(p)
        if tuple(p) not in t.signals: t.signals.append(tuple(p))
        if kind=='near':
            t.near=tuple(p)
        else:
            t.last_signal_bearing=body['svd_deg']
            obs={'x':p[0],'y':p[1],'svd_deg':body['svd_deg']}
            if obs not in t.dirs: t.dirs.append(obs)
            g=geometry(t.dirs,t.signals)
            if g['status']!='bounded':
                raise InvariantFailure('B4 outer geometry numerically uncertain; preserve trace')
            t.geometry=g; t.mec=g['mec']
        return body

    def clear(self,p,c,guaranteed=False,reason='B4_clear'):
        body=self.port.clear(p,c,reason); success=body['clear_result']=='success'
        if guaranteed:
            self.stats['guaranteed_clear_count']+=1
            if not success:
                self.stats['guaranteed_clear_failures']+=1
                raise InvariantFailure('guaranteed B4 clear failed')
        if success: self.cleared.add(c)
        else: self.tracks[c].clear_fail_points.append(tuple(p))
        return success

    def resolve(self,c):
        t=self.tracks[c]
        while c not in self.cleared:
            if t.near is not None:
                self.clear(t.near,c,True,'B4_near_clear'); return
            if t.mec and t.mec[1] <= 20-1e-6:
                self.clear(t.mec[0],c,True,'B4_guaranteed_clear'); return
            if t.fan_steps>=self.fan_limit:
                self.fallback(c); return
            anchor=t.last_signal_point
            lb=lower_distance(anchor,t.geometry['polygon'])
            if lb <= 20:
                if anchor not in t.clear_fail_points and self.clear(anchor,c,False,'B4_anchor_clear'):
                    return
                lb=max(lb,20.)
            pair=paired_fan(anchor,t.last_signal_bearing,lb)
            # Candidate order changes only travel; the theorem is symmetric.
            pair=sorted(pair,key=lambda p:math.dist(self.port.position,p))
            t.fan_steps+=1; self.stats['fan_steps']+=1
            self.port.record({'kind':'B4_fan','channel':c,'anchor':anchor,'lower_bound_m':lb,'points':pair})
            first=self.observe(pair[0],c,'B4_fan_first',backside=True)
            if first['measure_result']=='no_signal':
                self.stats['fan_second_required']+=1
                second=self.observe(pair[1],c,'B4_fan_second',backside=True)
                if second['measure_result']=='no_signal':
                    self.stats['fan_invariant_failures']+=1
                    raise InvariantFailure('both SPF45 branches returned no_signal')
            else:
                self.stats['fan_first_success']+=1

    def fallback(self,c):
        t=self.tracks[c]; self.stats['fallback_triggers']+=1
        todo=fallback_cover(t.dirs[0],t.geometry['polygon'] if t.geometry else None)
        for _ in range(len(todo)):
            p=min(todo,key=lambda p:math.dist(self.port.position,p)); todo.remove(p)
            self.stats['fallback_clear_cover_count']+=1
            if self.clear(p,c,False,'B4_finite_fallback'): return
        raise InvariantFailure('finite coverage exhausted without clearing discovered source')

    def run_all(self):
        remaining=list(range(len(self.mesh['vertices'])))
        while remaining and not self.discovery_complete():
            site=min(remaining,key=lambda i:math.dist(self.port.position,self.mesh['vertices'][i]))
            remaining.remove(site); p=self.mesh['vertices'][site]
            unknown=[c for c in range(1,21) if c not in self.tracks and c not in self.absent]
            unknown.sort(key=lambda c:(c!=self.port.channel,c))
            self.stats['certificate_sites_visited']+=1
            for c in unknown:
                if len(self.tracks)==16: break
                self.observe(p,c,'B4_certificate',site=site)
            # Interleave discovered-source completion and certificate progress.
            while any(c not in self.cleared for c in self.tracks):
                c=min((c for c in self.tracks if c not in self.cleared),
                      key=lambda c:math.dist(self.port.position,self.tracks[c].mec[0] if self.tracks[c].mec else self.tracks[c].near))
                self.resolve(c)
        if not self.discovery_complete() or self.cleared!=set(self.tracks):
            raise InvariantFailure('incomplete discovery or clearance')
