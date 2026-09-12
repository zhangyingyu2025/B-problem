"""G8 DEV: zero-detour unknown-channel probes while travelling, plus G4 localization samples.

A transit no_signal is deliberately NOT absence evidence. A successful transit probe only
discovers the channel earlier; full absence still requires the unchanged 25-site certificate.
"""
from collections import defaultdict
import math
from solver_g4 import G4TransitSolver

class _G8ProbeBase(G4TransitSolver):
    transit_discovery_per_leg=2
    probe_fraction=.5
    min_probe_leg_m=250.
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self._probe_counts=defaultdict(int)
        self.stats.update({'transit_discovery_measurements':0,'transit_discoveries':0,
                           'transit_discovery_no_signal':0})
    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position; length=math.dist(start,destination)
        if length<1e-7:return
        # Build the same known-source opportunity plan as G4, but retain u.
        options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c,'known')
            if best is not None:options.append(best)
        chosen=sorted(options,reverse=True)[:self.transit_per_leg]
        # Unknown probes: one common point, channels balanced by prior probe count.
        if length>=self.min_probe_leg_m and not self.discovery_complete():
            u=self.probe_fraction;p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
            unknown=[c for c in range(1,21) if c not in self.tracks and c not in self.absent]
            unknown.sort(key=lambda c:(self._probe_counts[c],len(self.checked[c]),c))
            for c in unknown[:self.transit_discovery_per_leg]:
                chosen.append((0.,u,p,c,'unknown'))
        chosen.sort(key=lambda x:(x[1],x[4]!='unknown'))
        for _,u,p,c,typ in chosen:
            if typ=='unknown':
                if c in self.tracks or c in self.absent:continue
                self._probe_counts[c]+=1
                body=self.observe(p,c,'B4_transit_discovery')
                self.stats['transit_discovery_measurements']+=1
                if body['measure_result']=='no_signal':self.stats['transit_discovery_no_signal']+=1
                else:self.stats['transit_discoveries']+=1
            else:
                if c in self.cleared or self.tracks[c].near is not None or self._already_sampled(self.tracks[c],p):continue
                body=self.observe(p,c,'B4_transit_opportunity')
                self.stats['transit_opportunity_measurements']+=1
                kind=body['measure_result']
                if kind=='direction':self.stats['transit_opportunity_directions']+=1
                elif kind=='near':self.stats['transit_opportunity_near']+=1
                else:self.stats['transit_opportunity_no_signal']+=1
class G8Probe1(_G8ProbeBase):transit_discovery_per_leg=1
class G8Probe2(_G8ProbeBase):transit_discovery_per_leg=2
class G8Probe3(_G8ProbeBase):transit_discovery_per_leg=3
class G8Probe4(_G8ProbeBase):transit_discovery_per_leg=4
