"""A/B/C strategies sharing discovery certificates and finite clearance fallback."""
import math
from channel_manager import ChannelManager,channel_order
from coverage import sites,clearance_grid
from protocol_adapter import ProtocolError,BudgetExceeded
from scheduler import insertion_distance,useful_onroute


class StrategyError(RuntimeError): pass


class Solver:
    def __init__(self,client,strategy='C',rho=1130,rotation_deg=0,onroute_limit=3,
                 insertion_limit_m=160,max_actions=4000,min_sources=10):
        if strategy not in ('A','B','C'): raise ValueError('strategy must be A, B or C')
        self.client=client;self.strategy=strategy;self.sites=sites(rho,rotation_deg)
        self.manager=ChannelManager();self.seen_measurements=set();self.onroute_limit=onroute_limit
        self.insertion_limit=insertion_limit_m;self.max_actions=max_actions;self.min_sources=min_sources
        self.stats={'b2_calls':0,'onroute_observations':0,'onroute_attempts':0,'grid_clear_attempts':0,
                    'guaranteed_clear_failures':0,'discovery_measurements':0}

    def _guard(self):
        if self.client.stats['measurements']+self.client.stats['clear_attempts']>=self.max_actions:
            raise BudgetExceeded('action count limit reached')

    def detect(self,point,channel,reason,site=None):
        self._guard();key=(channel,round(point[0],7),round(point[1],7))
        if key in self.seen_measurements: return None
        response=self.client.measure(point,channel,reason)
        self.seen_measurements.add(key)
        track=self.manager.register(channel,site,response,point)
        if reason=='coverage_discovery': self.stats['discovery_measurements']+=1
        if reason=='onroute_localization':
            self.stats['onroute_attempts']+=1
            self.stats['onroute_observations']+=int(response['measure_result']=='direction')
        if track:
            self.client.record({'kind':'track','reason':reason,**track.snapshot()})
            if response['measure_result']=='near':
                if not self.clear(track,point,'near_guarantee'): raise StrategyError('near clear unexpectedly failed')
        return track

    def clear(self,track,point,reason):
        if track.cleared: return True
        self._guard();response=self.client.clear(point,track.channel,reason)
        if reason=='finite_cover_grid': self.stats['grid_clear_attempts']+=1
        if response['clear_result']=='success':
            track.cleared=True;track.near_position=None
            self.client.record({'kind':'track','reason':reason,**track.snapshot()});return True
        if reason!='finite_cover_grid':
            self.stats['guaranteed_clear_failures']+=1;track.invalid_clear_certificate=True
            track.diagnostics.append({'code':'certified_clear_failed','position':point})
        return False

    def finish_track(self,track):
        if track.cleared: return
        point=track.clear_point(self.client.position)
        if point is not None and self.clear(track,point,'mec_or_current_guarantee'): return
        if not track.b2_used:
            point=track.supplemental_point();self.stats['b2_calls']+=1
            if point is not None:
                self.detect(point,track.channel,'b2_fallback')
                if track.cleared: return
                clearpoint=track.clear_point(self.client.position)
                if clearpoint is not None and self.clear(track,clearpoint,'mec_or_current_guarantee'): return
        if not track.observations: raise StrategyError('cannot form finite source rectangle without direction')
        grid=clearance_grid(track.observations[0],track.trusted_vertices(),self.client.position)
        self.client.record({'kind':'grid_plan','channel':track.channel,'point_count':len(grid),
                            'cell_max_m':28,'reason':'finite termination fallback after at most one B2'})
        for point in grid:
            if self.clear(track,point,'finite_cover_grid'): return
        raise StrategyError('all finite-cover clear points failed; do not claim completion')

    def insert_clearable(self,next_site):
        while True:
            choices=[]
            for track in self.manager.tracks.values():
                point=track.clear_point(self.client.position)
                if point is not None:
                    delta=insertion_distance(self.client.position,point,next_site)
                    if delta<=self.insertion_limit: choices.append((delta,track.channel,point))
            if not choices: return
            _,channel,point=min(choices)
            if not self.clear(self.manager.tracks[channel],point,'low_detour_clear'): return

    def run(self):
        status='incomplete';error=None
        try:
            self.client.enter()
            for index,site in enumerate(self.sites):
                if self.manager.discovery_complete(): break
                for channel in channel_order(self.manager.discovery_channels(),self.client.channel,index%2==1):
                    if self.manager.discovery_complete(): break
                    track=self.detect(site,channel,'coverage_discovery',index)
                    if track and self.strategy=='B': self.finish_track(track)
                if self.strategy=='C':
                    candidates=[t for t in self.manager.tracks.values() if useful_onroute(t,site)]
                    candidates.sort(key=lambda t:(len(t.observations),t.channel))
                    for track in candidates[:self.onroute_limit]: self.detect(site,track.channel,'onroute_localization')
                    next_site=self.sites[index+1] if index+1<len(self.sites) else None
                    self.insert_clearable(next_site)
            if not self.manager.discovery_complete(): raise StrategyError('coverage evidence incomplete')
            if not self.min_sources<=len(self.manager.tracks)<=16:
                raise StrategyError('discovered count outside problem bounds')
            while any(not t.cleared for t in self.manager.tracks.values()):
                pending=[t for t in self.manager.tracks.values() if not t.cleared]
                # Finish cheap certified clear tasks before costly active locating.
                def priority(t):
                    point=t.clear_point(self.client.position)
                    if point is not None: return (0,math.dist(self.client.position,point)/5+5,t.channel)
                    first=t.observations[0]
                    return (1,math.dist(self.client.position,(first['x'],first['y']))/5,t.channel)
                self.finish_track(min(pending,key=priority))
            status='completed'
            self.client.exit()
        except (ProtocolError,StrategyError,ArithmeticError,ValueError) as exc:
            status='incomplete';error=f'{type(exc).__name__}: {exc}'
            self.client.record({'kind':'run_failure','error':error})
            # No fresh action after ambiguous transport failures. On a known
            # local budget stop, exit only while the real deadline is still open.
            if isinstance(exc,(BudgetExceeded,StrategyError)) and self.client.active and self.client.clock()<self.client.deadline-3:
                try: self.client.exit()
                except ProtocolError: pass
        cleared=sum(t.cleared for t in self.manager.tracks.values())
        return {'status':status,'error':error,'strategy':self.strategy,'discovered_count':len(self.manager.tracks),
                'cleared_count':cleared,'discovery_complete':self.manager.discovery_complete(),
                'discovery_evidence':self.manager.evidence(),'tracks':[t.snapshot() for t in self.manager.tracks.values()],
                'metrics':{**self.client.stats,**self.stats,'total_virtual_time_s':self.client.virtual_time_s,
                           'mean_time_per_clear_s':self.client.virtual_time_s/cleared if cleared else None,
                           'wall_time_s':0 if self.client.started is None else self.client.clock()-self.client.started},
                'official_rehearsal':False}
